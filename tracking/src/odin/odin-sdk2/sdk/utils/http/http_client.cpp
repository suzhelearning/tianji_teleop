#include "http_client.h"
#include "certs/certs.h"
#include "http_public.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <libgen.h>
#include <unistd.h>

#include "mongoose/mongoose.h"

namespace http {

/* ------------------------------------------------------------------ */
/*  Internal: stream_state (reused from legacy net_client)              */
/* ------------------------------------------------------------------ */

struct stream_state {
  char *s_url;

  char *fpath;
  char output_dir[MG_PATH_MAX];
  char resolved_path[MG_PATH_MAX];
  uint64_t fsize;
  uint64_t offset;
  uint64_t expected;
  uint64_t received;
  void *fd;

  uint64_t deadline;
  int http_status;

  struct handle_opts *user_opts;

  struct {
    uint8_t parsed_request : 1;
    uint8_t done : 1;
    uint8_t success : 1;
  } flags;
};

/* ------------------------------------------------------------------ */
/*  TLS helper (only used for https:// URLs)                            */
/* ------------------------------------------------------------------ */

static void handle_tls_accept(struct mg_connection *c) {
  struct mg_tls_opts opts {};
  opts.ca = mg_str(certs::TLS_CA);
  opts.cert = mg_str(certs::TLS_CLIENT_CERT);
  opts.key = mg_str(certs::TLS_CLIENT_KEY);
  MG_INFO(("[Client] TLS init with embedded certs"));
  mg_tls_init(c, &opts);
}

/* ------------------------------------------------------------------ */
/*  download handler (reused — works for HTTP and HTTPS)                */
/* ------------------------------------------------------------------ */

static void handle_download(struct mg_connection *c, int ev, void *ev_data) {
  struct stream_state *ss = (struct stream_state *)c->fn_data;
  struct handle_opts *user_opts = ss->user_opts;
  struct mg_fs *fs = &mg_fs_posix;

  if (ev == MG_EV_HTTP_HDRS) {
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
    ss->http_status = mg_http_status(hm);

    struct mg_str *cl = mg_http_get_header(hm, "Content-Length");
    if (cl) {
      char tmp[32];
      size_t n = cl->len < sizeof(tmp) - 1 ? cl->len : sizeof(tmp) - 1;
      memcpy(tmp, cl->buf, n);
      tmp[n] = '\0';
      ss->expected = strtoull(tmp, NULL, 10);
    }

    if (ss->http_status == 200) {
      const char *fallback = "download";
      struct mg_str *cd = mg_http_get_header(hm, "Content-Disposition");
      if (cd) {
        struct mg_str fn = mg_http_get_header_var(*cd, mg_str("filename"));
        if (fn.buf && fn.len > 0) {
          size_t copy_len = fn.len < MG_PATH_MAX - 1 ? fn.len : MG_PATH_MAX - 1;
          snprintf(ss->resolved_path, sizeof(ss->resolved_path), "%s%.*s", ss->output_dir,
                   (int)copy_len, fn.buf);
        } else {
          snprintf(ss->resolved_path, sizeof(ss->resolved_path), "%s%s", ss->output_dir, fallback);
        }
      } else {
        snprintf(ss->resolved_path, sizeof(ss->resolved_path), "%s%s", ss->output_dir, fallback);
      }

      ss->fd = mg_fs_posix.op(ss->resolved_path, MG_FS_WRITE);
      if (ss->fd == NULL) {
        MG_ERROR(("Failed to open %s for writing", ss->resolved_path));
        mg_error(c, "open failed");
        return;
      }
      MG_DEBUG(("Saving to: %s", ss->resolved_path));
    }

    mg_iobuf_del(&c->recv, 0, hm->head.len);
  }

  if (ss->http_status == 200 && ss->expected > 0 && c->recv.len > 0) {
    ss->received += c->recv.len;
    MG_DEBUG(("Got chunk: %lu bytes, %lu/%lu total", (unsigned long)c->recv.len,
              (unsigned long)ss->received, (unsigned long)ss->expected));
    if (ss->fd) fs->wr(ss->fd, c->recv.buf, c->recv.len);
    c->recv.len = 0;
    if (user_opts && user_opts->progress_download_callback) {
      user_opts->progress_download_callback(ss->received, ss->expected);
    }
    if (ss->received >= ss->expected) {
      MG_DEBUG(("Download complete: %lu bytes", (unsigned long)ss->received));
      if (ss->fd) {
        fs->cl(ss->fd);
        ss->fd = NULL;
      }
      ss->flags.done = 1;
      c->is_draining = 1;
    }
  }
}

static void download_handler(struct mg_connection *c, int ev, void *ev_data) {
  struct stream_state *ss = (struct stream_state *)c->fn_data;

  if (ev == MG_EV_OPEN) {
    ss->deadline = mg_millis() + 5000;
  } else if (ev == MG_EV_POLL) {
    if (mg_millis() > ss->deadline && (c->is_connecting || c->is_resolving)) {
      mg_error(c, "Connect timeout");
    }
  } else if (ev == MG_EV_CONNECT) {
    if (c->is_tls) handle_tls_accept(c);
    struct mg_str host = mg_url_host(ss->s_url);
    mg_printf(c,
              "GET %s HTTP/1.0\r\n"
              "Host: %.*s\r\n"
              "Connection: close\r\n"
              "\r\n",
              mg_url_uri(ss->s_url), (int)host.len, host.buf);
  } else if (ev == MG_EV_HTTP_HDRS || ev == MG_EV_READ) {
    handle_download(c, ev, ev_data);
  } else if (ev == MG_EV_CLOSE) {
    struct mg_fs *fs = &mg_fs_posix;
    if (ss->fd) {
      fs->cl(ss->fd);
      ss->fd = NULL;
    }
    ss->flags.done = 1;
  } else if (ev == MG_EV_ERROR) {
    MG_ERROR(("download error: %s", (const char *)ev_data));
    struct mg_fs *fs = &mg_fs_posix;
    if (ss->fd) {
      fs->cl(ss->fd);
      ss->fd = NULL;
    }
    ss->flags.done = 1;
  }
}

/* ------------------------------------------------------------------ */
/*  upload handler (reused — streaming POST, works for HTTP and HTTPS)  */
/* ------------------------------------------------------------------ */

static void upload_handler(struct mg_connection *c, int ev, void *ev_data) {
  struct stream_state *ss = (struct stream_state *)c->fn_data;
  struct handle_opts *user_opts = ss->user_opts;
  struct mg_fs *fs = &mg_fs_posix;

  if (ev == MG_EV_OPEN) {
    ss->deadline = mg_millis() + 5000;
  } else if (ev == MG_EV_POLL) {
    if (mg_millis() > ss->deadline && (c->is_connecting || c->is_resolving)) {
      mg_error(c, "Connect timeout");
    }
  } else if (ev == MG_EV_CONNECT) {
    if (c->is_tls) handle_tls_accept(c);
    struct mg_str host = mg_url_host(ss->s_url);
    mg_printf(c,
              "POST %s HTTP/1.0\r\n"
              "Host: %.*s\r\n"
              "Content-Type: octet-stream\r\n"
              "Content-Length: %lu\r\n"
              "Connection: close\r\n"
              "\r\n",
              mg_url_uri(ss->s_url), (int)host.len, host.buf, (unsigned long)ss->fsize);
  } else if (ev == MG_EV_WRITE) {
    if (c->send.len < MG_IO_SIZE && ss->offset < ss->fsize) {
      uint8_t *buf = (uint8_t *)alloca(MG_IO_SIZE);
      uint64_t send_len = MG_IO_SIZE - c->send.len;
      uint64_t remain_size = ss->fsize - ss->offset;
      uint64_t read_len = remain_size < send_len ? remain_size : send_len;
      if (read_len > 0) {
        fs->rd(ss->fd, buf, read_len);
        mg_send(c, buf, read_len);
        ss->offset += read_len;
        MG_DEBUG(("sent %u bytes", (unsigned)read_len));
        if (user_opts && user_opts->progress_upload_callback) {
          user_opts->progress_upload_callback(ss->offset, ss->fsize);
        }
      }
    }
  } else if (ev == MG_EV_HTTP_MSG) {
    MG_DEBUG(("MSG"));
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
    ss->http_status = mg_http_status(hm);
    (void)hm;
    c->is_draining = 1;
    if (ss->fd) {
      fs->cl(ss->fd);
      ss->fd = NULL;
    }
    ss->flags.done = 1;
  } else if (ev == MG_EV_ERROR) {
    MG_ERROR(("upload ERROR"));
    if (ss->fd) {
      fs->cl(ss->fd);
      ss->fd = NULL;
    }
    ss->flags.done = 1;
  }
}

/* ------------------------------------------------------------------ */
/*  simple POST handler (reused for trigger/reset/reboot)               */
/* ------------------------------------------------------------------ */

static void simple_post_handler(struct mg_connection *c, int ev, void *ev_data) {
  struct stream_state *ss = (struct stream_state *)c->fn_data;

  if (ev == MG_EV_OPEN) {
    ss->deadline = mg_millis() + 5000;
  } else if (ev == MG_EV_POLL) {
    if (mg_millis() > ss->deadline && (c->is_connecting || c->is_resolving)) {
      mg_error(c, "Connect timeout");
    }
  } else if (ev == MG_EV_CONNECT) {
    if (c->is_tls) handle_tls_accept(c);
    struct mg_str host = mg_url_host(ss->s_url);
    mg_printf(c,
              "POST %s HTTP/1.0\r\n"
              "Host: %.*s\r\n"
              "Content-Length: 0\r\n"
              "Connection: close\r\n"
              "\r\n",
              mg_url_uri(ss->s_url), (int)host.len, host.buf);
  } else if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
    ss->http_status = mg_http_status(hm);
    MG_DEBUG(("Response: %.*s", (int)hm->body.len, hm->body.buf));
    ss->flags.done = 1;
    c->is_draining = 1;
  } else if (ev == MG_EV_ERROR) {
    ss->flags.done = 1;
  }
}

/* ------------------------------------------------------------------ */
/*  simple_request handler (GET or POST with body, returns response)    */
/* ------------------------------------------------------------------ */

struct simple_req_state {
  const char *full_url;
  const char *method;
  std::string extra_headers;
  const void *body;
  size_t body_len;

  int http_status;
  std::string response_body;
  bool done;
  int64_t deadline;
};

static void simple_request_handler(struct mg_connection *c, int ev, void *ev_data) {
  struct simple_req_state *rs = (struct simple_req_state *)c->fn_data;

  if (ev == MG_EV_OPEN) {
    /* nothing */
  } else if (ev == MG_EV_POLL) {
    if (mg_millis() > (uint64_t)rs->deadline && (c->is_connecting || c->is_resolving)) {
      mg_error(c, "Connect timeout");
    }
  } else if (ev == MG_EV_CONNECT) {
    if (c->is_tls) handle_tls_accept(c);
    struct mg_str host = mg_url_host(rs->full_url);
    if (rs->body && rs->body_len > 0) {
      mg_printf(c,
                "%s %s HTTP/1.0\r\n"
                "Host: %.*s\r\n"
                "Content-Length: %lu\r\n"
                "%s"
                "Connection: close\r\n"
                "\r\n",
                rs->method, mg_url_uri(rs->full_url), (int)host.len, host.buf,
                (unsigned long)rs->body_len, rs->extra_headers.c_str());
      mg_send(c, rs->body, rs->body_len);
    } else {
      mg_printf(c,
                "%s %s HTTP/1.0\r\n"
                "Host: %.*s\r\n"
                "Content-Length: 0\r\n"
                "%s"
                "Connection: close\r\n"
                "\r\n",
                rs->method, mg_url_uri(rs->full_url), (int)host.len, host.buf,
                rs->extra_headers.c_str());
    }
  } else if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *)ev_data;
    rs->http_status = mg_http_status(hm);
    if (hm->body.len > 0) {
      rs->response_body.assign(hm->body.buf, hm->body.len);
    }
    rs->done = true;
    c->is_draining = 1;
  } else if (ev == MG_EV_ERROR) {
    MG_ERROR(("simple_request error: %s", (const char *)ev_data));
    rs->done = true;
  } else if (ev == MG_EV_CLOSE) {
    rs->done = true;
  }
}

/* ------------------------------------------------------------------ */
/*  Client public API                                                   */
/* ------------------------------------------------------------------ */

Client::Client() = default;

Client::Client(const std::string &base_url) : base_url_(base_url) {}

Client::~Client() = default;

/* ------------------------------------------------------------------ */
/*  simple_request — core primitive for new HTTP endpoints               */
/* ------------------------------------------------------------------ */

int Client::simple_request(const std::string &method, const std::string &uri,
                           const std::string &extra_headers, const void *body,
                           size_t body_len, std::string &response_body,
                           int timeout_ms) {
  std::string full_url = base_url_ + uri;

  struct simple_req_state rs = {};
  rs.full_url = full_url.c_str();
  rs.method = method.c_str();
  rs.extra_headers = extra_headers;
  rs.body = body;
  rs.body_len = body_len;

  struct mg_mgr mgr;
  mg_mgr_init(&mgr);

  struct mg_connection *c = mg_http_connect(&mgr, full_url.c_str(), simple_request_handler, &rs);
  if (c == NULL) {
    mg_mgr_free(&mgr);
    return 0;
  }

  rs.deadline = mg_millis() + timeout_ms;
  uint64_t overall = mg_millis() + timeout_ms;
  while (!rs.done && mg_millis() < overall) {
    mg_mgr_poll(&mgr, 50);
  }

  mg_mgr_free(&mgr);
  response_body = rs.response_body;
  return rs.http_status;
}

/* ------------------------------------------------------------------ */
/*  Health & Info                                                        */
/* ------------------------------------------------------------------ */

bool Client::health_check(int timeout_ms) {
  std::string body;
  int status = simple_request("GET", URI_API_HEALTH, "", nullptr, 0, body, timeout_ms);
  return status == 200;
}

bool Client::get_version(std::string &response_json) {
  int status = simple_request("GET", URI_API_VERSION, "", nullptr, 0, response_json, 5000);
  return status == 200;
}

bool Client::get_device_info(std::string &response_json) {
  int status = simple_request("GET", URI_API_DEVICE_INFO, "", nullptr, 0, response_json, 5000);
  return status == 200;
}

/* ------------------------------------------------------------------ */
/*  OTA — new HTTP chunked upload                                       */
/* ------------------------------------------------------------------ */

static const size_t kOtaChunkSize = 1024 * 1024; /* 1 MB */

bool Client::ota_upload(const std::string &file_path, ProgressCallback cb) {
  FILE *fp = fopen(file_path.c_str(), "rb");
  if (!fp) {
    MG_ERROR(("[Client] failed to open: %s", file_path.c_str()));
    return false;
  }

  fseek(fp, 0, SEEK_END);
  long file_size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  if (file_size <= 0) {
    MG_ERROR(("[Client] invalid file size: %s", file_path.c_str()));
    fclose(fp);
    return false;
  }

  /* extract filename */
  std::string filename;
  size_t pos = file_path.find_last_of("/\\");
  filename = (pos != std::string::npos) ? file_path.substr(pos + 1) : file_path;

  size_t total = static_cast<size_t>(file_size);
  size_t offset = 0;
  std::vector<uint8_t> chunk_buf(kOtaChunkSize);
  bool success = true;

  MG_INFO(("[Client] OTA upload %s (%lu bytes) in %lu-byte chunks",
           file_path.c_str(), (unsigned long)total, (unsigned long)kOtaChunkSize));

  while (offset < total) {
    size_t chunk_len = (total - offset) < kOtaChunkSize ? (total - offset) : kOtaChunkSize;
    size_t rd = fread(chunk_buf.data(), 1, chunk_len, fp);
    if (rd != chunk_len) {
      MG_ERROR(("[Client] read error at offset %lu", (unsigned long)offset));
      success = false;
      break;
    }

    size_t end = offset + chunk_len - 1;

    char hdr[512];
    snprintf(hdr, sizeof(hdr),
             "X-Filename: %s\r\n"
             "Content-Range: bytes %lu-%lu/%lu\r\n",
             filename.c_str(), (unsigned long)offset, (unsigned long)end, (unsigned long)total);

    std::string resp;
    int status = simple_request("POST", URI_API_OTA_UPLOAD, hdr,
                                chunk_buf.data(), chunk_len, resp, 30000);
    if (status != 200) {
      MG_ERROR(("[Client] OTA chunk failed at offset %lu, HTTP %d", (unsigned long)offset, status));
      success = false;
      break;
    }

    offset += chunk_len;
    if (cb) cb(offset, total);
  }

  fclose(fp);
  return success;
}

bool Client::ota_trigger() {
  std::string resp;
  int status = simple_request("POST", URI_API_OTA_TRIGGER, "", nullptr, 0, resp, 10000);
  return status == 200;
}

/* ------------------------------------------------------------------ */
/*  OTA status polling                                                  */
/* ------------------------------------------------------------------ */

static std::string json_get_str(const struct mg_str &json, const char *path) {
  struct mg_str val = mg_json_get_tok(json, path);
  if (val.len >= 2 && val.buf[0] == '"') {
    return std::string(val.buf + 1, val.len - 2);
  }
  return "";
}

static int json_get_int(const struct mg_str &json, const char *path) {
  double v = 0;
  if (mg_json_get_num(json, path, &v)) return static_cast<int>(v);
  return 0;
}

bool Client::ota_status(OtaStatus &out) {
  std::string resp;
  int status = simple_request("GET", URI_API_OTA_STATUS, "", nullptr, 0, resp, 5000);
  if (status != 200 || resp.empty()) return false;

  struct mg_str json = mg_str(resp.c_str());
  out.state = json_get_str(json, "$.ota.state");
  out.progress = json_get_int(json, "$.ota.progress");
  out.message = json_get_str(json, "$.ota.message");
  out.error = json_get_str(json, "$.ota.error");
  out.mcu_result = json_get_str(json, "$.ota.mcu_result");

  if (out.state.empty()) {
    out.state = json_get_str(json, "$.state");
    if (out.progress == 0) out.progress = json_get_int(json, "$.progress");
  }

  return !out.state.empty();
}

bool Client::ota_reset() {
  std::string resp;
  int status = simple_request("POST", URI_API_OTA_RESET, "", nullptr, 0, resp, 5000);
  return status == 200;
}

bool Client::ota_wait_complete(int timeout_s, OtaStatusCallback cb) {
  auto start = std::chrono::steady_clock::now();
  std::string prev_state;
  int reboot_polls = 0;
  int spin_idx = 0;
  const char spin_chars[] = {'/', '-', '\\', '|'};
  const int kMaxRebootPolls = 36; /* 36 * 5s = 180s reboot timeout */
  bool log_suppressed = false;

  while (true) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    int elapsed_s = (int)std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    if (elapsed_s >= timeout_s) {
      if (log_suppressed) { mg_log_level = MG_LL_ERROR; log_suppressed = false; }
      std::cerr << "\nOTA timeout after " << timeout_s << "s\n";
      return false;
    }

    /* suppress mongoose logs when device may be unreachable */
    bool expect_unreachable = (prev_state == "INSTALLING_SOC" ||
                               prev_state == "REBOOTING" || reboot_polls > 0);
    if (expect_unreachable && !log_suppressed) {
      mg_log_level = MG_LL_NONE;
      log_suppressed = true;
    }

    OtaStatus st;
    bool ok = ota_status(st);

    if (!ok) {
      /* device unreachable — spinner on same line */
      reboot_polls++;
      std::cout << "\r\33[2KWaiting for device reboot... "
                << spin_chars[spin_idx++ % 4] << std::flush;
      if (reboot_polls > kMaxRebootPolls) {
        if (log_suppressed) { mg_log_level = MG_LL_ERROR; log_suppressed = false; }
        std::cerr << "\nDevice did not recover after reboot (180s timeout)\n";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::seconds(5));
      continue;
    }

    /* device back online — restore log level */
    if (log_suppressed) {
      mg_log_level = MG_LL_ERROR;
      log_suppressed = false;
    }
    if (reboot_polls > 0) {
      std::cout << "\r\33[2KDevice back online.\n";
    }
    reboot_polls = 0;

    /* invoke optional caller callback */
    if (cb) cb(st);

    /* state transition → newline to preserve previous state in history */
    if (st.state != prev_state) {
      if (!prev_state.empty()) {
        std::cout << "\n";
      }
      prev_state = st.state;
    }

    /* terminal states */
    if (st.state == "DONE") {
      std::cout << "\r\33[2KDONE  OTA completed successfully!\n";
      return true;
    }
    if (st.state == "FAILED") {
      std::cerr << "\r\33[2KFAILED";
      if (!st.error.empty()) std::cerr << ": " << st.error;
      std::cerr << "\n";
      return false;
    }

    /* active state — spinner with message on same line */
    std::string display = st.state;
    if (!st.message.empty()) display += "  " + st.message;
    std::cout << "\r\33[2K" << display << " "
              << spin_chars[spin_idx++ % 4] << std::flush;

    /* poll interval */
    bool is_reboot_state = (st.state == "INSTALLING_SOC" || st.state == "REBOOTING");
    int sleep_ms = is_reboot_state ? 5000 : 1000;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
}

/* ------------------------------------------------------------------ */
/*  File Transfer — new HTTP endpoints                                  */
/* ------------------------------------------------------------------ */

bool Client::download_log(const std::string &output_dir) {
  std::string url = base_url_ + URI_API_FILE_DOWNLOAD_LOG;
  return download(url, output_dir);
}

bool Client::download_calib(const std::string &output_dir) {
  std::string url = base_url_ + URI_API_FILE_DOWNLOAD_CALIB;
  return download(url, output_dir);
}

bool Client::download_map(const std::string &output_dir) {
  std::string url = base_url_ + URI_API_FILE_DOWNLOAD_MAP;
  return download(url, output_dir);
}

bool Client::upload_calib(const std::string &file_path) {
  std::string url = base_url_ + URI_API_FILE_UPLOAD_CALIB;
  return upload(url, file_path);
}

/* ------------------------------------------------------------------ */
/*  Network & System                                                    */
/* ------------------------------------------------------------------ */

bool Client::get_network(std::string &response_json) {
  int status = simple_request("GET", URI_API_NETWORK, "", nullptr, 0, response_json, 5000);
  return status == 200;
}

bool Client::set_network(const std::string &request_json) {
  std::string resp;
  int status = simple_request("POST", URI_API_NETWORK,
                              "Content-Type: application/json\r\n",
                              request_json.c_str(), request_json.size(), resp, 5000);
  return status == 200;
}

bool Client::reboot() {
  std::string resp;
  int status = simple_request("POST", URI_API_REBOOT, "", nullptr, 0, resp, 5000);
  return status == 200;
}

/* ------------------------------------------------------------------ */
/*  Reused transport primitives                                         */
/* ------------------------------------------------------------------ */

bool Client::download(const std::string &url, const std::string &output_dir) {
  struct stream_state ss;
  memset(&ss, 0, sizeof(ss));
  ss.s_url = (char *)url.c_str();

  snprintf(ss.output_dir, sizeof(ss.output_dir), "%s%s", output_dir.c_str(),
           (!output_dir.empty() && output_dir.back() != '/') ? "/" : "");

  struct mg_mgr mgr;
  mg_mgr_init(&mgr);
  MG_INFO(("[Client] download %s -> %s", url.c_str(), ss.output_dir));

  ss.user_opts = &handle_options_;
  struct mg_connection *c = mg_http_connect(&mgr, url.c_str(), download_handler, &ss);
  if (c == NULL) {
    mg_mgr_free(&mgr);
    return false;
  }

  int64_t deadline = mg_millis() + download_timeout_ms_;
  while (!ss.flags.done && mg_millis() < (uint64_t)deadline) mg_mgr_poll(&mgr, 50);

  mg_mgr_free(&mgr);

  if (handle_options_.after_download_hook) {
    if (handle_options_.after_download_hook(ss.resolved_path, output_dir.c_str()) != 0) {
      MG_ERROR(("[Client] after_download_hook failed for %s", ss.resolved_path));
      return false;
    }
  }

  return ss.flags.done && ss.http_status == 200;
}

bool Client::upload(const std::string &url, const std::string &local_path) {
  size_t fsize = 0;
  time_t mtime = 0;
  mg_fs_posix.st(local_path.c_str(), &fsize, &mtime);
  if (fsize == 0) {
    MG_ERROR(("[Client] file not found or empty: %s", local_path.c_str()));
    return false;
  }

  if (handle_options_.before_upload_hook) {
    if (handle_options_.before_upload_hook(local_path.c_str()) != 0) {
      MG_ERROR(("[Client] before_upload_hook failed for %s", local_path.c_str()));
      return false;
    }
  }

  struct stream_state us;
  memset(&us, 0, sizeof(us));
  us.s_url = (char *)url.c_str();
  us.fsize = fsize;
  us.fd = mg_fs_posix.op(local_path.c_str(), MG_FS_READ);
  if (us.fd == NULL) {
    MG_ERROR(("[Client] failed to open file: %s (errno %d)", local_path.c_str(), errno));
    return false;
  }

  struct mg_mgr mgr;
  mg_mgr_init(&mgr);
  MG_INFO(("[Client] upload %s (%lu bytes) -> %s", local_path.c_str(), (unsigned long)fsize,
           url.c_str()));

  us.user_opts = &handle_options_;
  struct mg_connection *c = mg_http_connect(&mgr, url.c_str(), upload_handler, &us);
  if (c == NULL) {
    MG_ERROR(("[Client] failed to create connection"));
    mg_fs_posix.cl(us.fd);
    mg_mgr_free(&mgr);
    return false;
  }

  int64_t size_timeout = (int64_t)(fsize / 1024) + 30000;
  int64_t deadline = mg_millis() + size_timeout;
  while (!us.flags.done && mg_millis() < (uint64_t)deadline) mg_mgr_poll(&mgr, 50);

  mg_mgr_free(&mgr);

  if (handle_options_.after_upload_hook) {
    if (handle_options_.after_upload_hook(local_path.c_str()) != 0) {
      MG_ERROR(("[Client] after_upload_hook failed for %s", local_path.c_str()));
      return false;
    }
  }

  return us.flags.done;
}

/* ------------------------------------------------------------------ */
/*  Legacy HTTPS wrappers (deprecated)                                  */
/* ------------------------------------------------------------------ */

bool Client::download_file_legacy(const std::string &base_url, const std::string &name,
                                  const std::string &output_dir) {
  std::string url = base_url + URI_API_FILES_DOWNLOAD_PREFIX + name;
  return download(url, output_dir);
}

bool Client::upload_file_legacy(const std::string &base_url, const std::string &file_path) {
  if (access(file_path.c_str(), F_OK) != 0) {
    MG_ERROR(("[Client] file not found: %s", file_path.c_str()));
    return false;
  }

  char *dup_path = strdup(file_path.c_str());
  char *name = basename(dup_path);
  if (name == nullptr) {
    MG_ERROR(("[Client] invalid file path: %s", file_path.c_str()));
    free(dup_path);
    return false;
  }

  std::string url = base_url + URI_API_FILES_UPLOAD_PREFIX + name;
  free(dup_path);
  return upload(url, file_path);
}

bool Client::trigger_ota_legacy(const std::string &base_url) {
  std::string url = base_url + URI_API_FIRMWARE_UPLOAD;
  struct stream_state ss;
  memset(&ss, 0, sizeof(ss));
  ss.s_url = (char *)url.c_str();

  struct mg_mgr mgr;
  mg_mgr_init(&mgr);
  MG_INFO(("[Client] trigger OTA via %s", url.c_str()));

  ss.user_opts = &handle_options_;
  struct mg_connection *c = mg_http_connect(&mgr, url.c_str(), simple_post_handler, &ss);
  if (c == NULL) {
    MG_ERROR(("[Client] failed to create OTA connection"));
    mg_mgr_free(&mgr);
    return false;
  }

  int64_t deadline = mg_millis() + 20000;
  while (!ss.flags.done && mg_millis() < (uint64_t)deadline) mg_mgr_poll(&mgr, 50);
  mg_mgr_free(&mgr);

  return ss.flags.done && ss.http_status == 200;
}

bool Client::download_device_logs(const std::string &base_url, const std::string &output_path) {
  return download_file_legacy(base_url, DEVICE_LOG_FILE, output_path);
}

bool Client::download_calibration(const std::string &base_url, const std::string &output_path) {
  return download_file_legacy(base_url, CALIBRATION_FILE, output_path);
}

bool Client::download_relocation_map(const std::string &base_url, const std::string &output_path) {
  return download_file_legacy(base_url, RELOCATION_MAP_FILE, output_path);
}

bool Client::upload_firmware_and_OTA(const std::string &base_url, const std::string &file_path) {
  return upload_file_legacy(base_url, file_path) && trigger_ota_legacy(base_url);
}

}  /* namespace http */
