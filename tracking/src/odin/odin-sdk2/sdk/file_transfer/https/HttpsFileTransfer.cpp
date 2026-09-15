#include "HttpsFileTransfer.hpp"
#include "http_client.h"

#include <filesystem>

namespace odin {
namespace sdk {

/* ------------------------------------------------------------------ */
/*  Thread-local bridge: C function pointer <-> std::function          */
/*  Required because handle_opts uses plain C function pointers while  */
/*  IFileTransfer exposes std::function progress callbacks.            */
/* ------------------------------------------------------------------ */
static thread_local FileTransferProgressCallback s_progress_cb;
static thread_local std::string s_resolved_download_path;

static int progress_download_bridge(uint64_t current, uint64_t total) {
  if (s_progress_cb) s_progress_cb(static_cast<size_t>(current), static_cast<size_t>(total));
  return 0;
}

static int capture_resolved_path(const char* resolved, const char*) {
  if (resolved) s_resolved_download_path = resolved;
  return 0;
}

static int progress_upload_bridge(uint64_t current, uint64_t total) {
  if (s_progress_cb) s_progress_cb(static_cast<size_t>(current), static_cast<size_t>(total));
  return 0;
}

/* ------------------------------------------------------------------ */
/*  HttpsFileTransfer implementation                                   */
/* ------------------------------------------------------------------ */

HttpsFileTransfer::HttpsFileTransfer() = default;
HttpsFileTransfer::~HttpsFileTransfer() = default;

bool HttpsFileTransfer::Connect(const std::string& target) {
  if (target.find("http://") == 0) {
    base_url_ = target;
    use_http_ = true;
  } else if (target.find("https://") == 0) {
    base_url_ = target;
    use_http_ = false;
  } else {
    base_url_ = "https://" + target;
    use_http_ = false;
  }
  connected_ = true;
  return true;
}

void HttpsFileTransfer::Disconnect() {
  base_url_.clear();
  connected_ = false;
  use_http_ = false;
}

bool HttpsFileTransfer::IsConnected() const { return connected_; }

TransferResult HttpsFileTransfer::Download(const std::string& remote_name,
                                           const std::string& local_path,
                                           FileTransferProgressCallback cb) {
  TransferResult result;
  if (!connected_) {
    result.error_msg = "Not connected";
    return result;
  }

  namespace fs = std::filesystem;
  std::string output_dir = fs::path(local_path).parent_path().string();
  if (output_dir.empty()) output_dir = ".";

  bool ok = false;

  if (use_http_) {
    /* ---- New HTTP path (port 8080) ---- */
    http::Client client(base_url_);

    http::handle_opts opts{};
    s_progress_cb = cb;
    s_resolved_download_path.clear();
    if (cb) opts.progress_download_callback = progress_download_bridge;
    opts.after_download_hook = capture_resolved_path;
    client.set_handle_opts(opts);

    if (remote_name == "logs") {
      ok = client.download_log(output_dir);
    } else if (remote_name == "calibration") {
      ok = client.download_calib(output_dir);
    } else if (remote_name == "relocation_map") {
      ok = client.download_map(output_dir);
    } else {
      result.error_msg = "Unknown remote file: " + remote_name;
      s_progress_cb = nullptr;
      return result;
    }
    s_progress_cb = nullptr;
  } else {
    /* ---- Legacy HTTPS path ---- */
    http::Client client;

    http::handle_opts opts{};
    s_progress_cb = cb;
    s_resolved_download_path.clear();
    if (cb) opts.progress_download_callback = progress_download_bridge;
    opts.after_download_hook = capture_resolved_path;
    client.set_handle_opts(opts);

    if (remote_name == "logs") {
      ok = client.download_device_logs(base_url_, output_dir);
    } else if (remote_name == "calibration") {
      ok = client.download_calibration(base_url_, output_dir);
    } else if (remote_name == "relocation_map") {
      ok = client.download_relocation_map(base_url_, output_dir);
    } else {
      result.error_msg = "Unknown remote file: " + remote_name;
      s_progress_cb = nullptr;
      return result;
    }
    s_progress_cb = nullptr;
  }

  result.success = ok;
  if (!ok) {
    result.error_msg = "Download failed: " + remote_name;
    return result;
  }

  /* rename downloaded file to the caller-expected path if they differ */
  if (!s_resolved_download_path.empty() && s_resolved_download_path != local_path) {
    std::error_code ec;
    fs::rename(s_resolved_download_path, local_path, ec);
    if (ec) {
      result.success = false;
      result.error_msg = "rename failed: " + ec.message();
    }
  }
  return result;
}

TransferResult HttpsFileTransfer::Upload(const std::string& local_path,
                                         const std::string& remote_name,
                                         FileTransferProgressCallback cb) {
  TransferResult result;
  if (!connected_) {
    result.error_msg = "Not connected";
    return result;
  }

  bool ok = false;

  if (use_http_) {
    /* ---- New HTTP path (port 8080) ---- */
    http::Client client(base_url_);

    http::handle_opts opts{};
    s_progress_cb = cb;
    if (cb) opts.progress_upload_callback = progress_upload_bridge;
    client.set_handle_opts(opts);

    if (remote_name == "firmware") {
      ok = client.ota_upload(local_path, progress_upload_bridge);
    } else if (remote_name == "calibration") {
      ok = client.upload_calib(local_path);
    } else {
      result.error_msg = "Unsupported upload type: " + remote_name;
      s_progress_cb = nullptr;
      return result;
    }
    s_progress_cb = nullptr;
  } else {
    /* ---- Legacy HTTPS path ---- */
    http::Client client;

    http::handle_opts opts{};
    s_progress_cb = cb;
    if (cb) opts.progress_upload_callback = progress_upload_bridge;
    client.set_handle_opts(opts);

    if (remote_name == "firmware") {
      ok = client.upload_firmware_and_OTA(base_url_, local_path);
    } else {
      result.error_msg = "Unsupported upload type: " + remote_name;
      s_progress_cb = nullptr;
      return result;
    }
    s_progress_cb = nullptr;
  }

  result.success = ok;
  if (!ok) result.error_msg = "Upload failed: " + remote_name;
  return result;
}

void HttpsFileTransfer::SetVerifySignature(bool enable) { verify_signature_ = enable; }

}  // namespace sdk
}  // namespace odin
