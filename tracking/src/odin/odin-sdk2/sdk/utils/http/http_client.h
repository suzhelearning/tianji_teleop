#pragma once

#include <string>
#include <functional>

#include "http_public.h"

namespace http {

/**
 * @brief Unified HTTP/HTTPS client for ODIN device communication.
 *
 * Supports both plain HTTP (new scheme, port 8080) and HTTPS with mTLS
 * (legacy scheme). Protocol is auto-detected from the base_url scheme.
 * Uses mongoose internally for all network I/O.
 */
class Client {
 public:
  /**
   * @param base_url  Scheme + host + port, e.g. "http://192.168.1.200:8080"
   *                  or "https://192.168.1.200:4433"
   */
  explicit Client(const std::string &base_url);

  /** Default constructor — base_url must be passed per-method (legacy API). */
  Client();
  ~Client();

  Client(const Client &) = delete;
  Client &operator=(const Client &) = delete;

  /* ---------------------------------------------------------------- */
  /*  Health & Info                                                      */
  /* ---------------------------------------------------------------- */

  /** @brief GET /api/health — returns true if server responded 200 */
  bool health_check(int timeout_ms = 3000);

  /** @brief GET /api/version — raw JSON response */
  bool get_version(std::string &response_json);

  /** @brief GET /api/device_info — raw JSON response */
  bool get_device_info(std::string &response_json);

  /* ---------------------------------------------------------------- */
  /*  OTA — new HTTP chunked scheme                                     */
  /* ---------------------------------------------------------------- */

  /**
   * @brief Upload firmware in 1 MB chunks to POST /api/ota/upload
   *
   * Each chunk is a separate HTTP request with X-Filename + Content-Range.
   * @param file_path  Local firmware file path
   * @param cb         Optional progress callback (uploaded, total)
   */
  bool ota_upload(const std::string &file_path, ProgressCallback cb = nullptr);

  /** @brief POST /api/ota/trigger */
  bool ota_trigger();

  /** @brief GET /api/ota/status → parsed OtaStatus */
  bool ota_status(OtaStatus &out);

  /** @brief POST /api/ota/reset */
  bool ota_reset();

  /**
   * @brief Poll OTA status until terminal state (DONE / FAILED).
   *
   * Interval: 2 s normal, 5 s during reboot states, 180 s reboot timeout.
   * Prints state transitions to stdout.
   *
   * @param timeout_s  Overall timeout in seconds (default 300)
   * @param cb         Optional callback invoked on every successful poll
   * @return true if final state == "DONE"
   */
  bool ota_wait_complete(int timeout_s = 300, OtaStatusCallback cb = nullptr);

  /* ---------------------------------------------------------------- */
  /*  File Transfer — new HTTP endpoints                                */
  /* ---------------------------------------------------------------- */

  /** @brief GET /api/file/download/log */
  bool download_log(const std::string &output_dir);

  /** @brief GET /api/file/download/calib */
  bool download_calib(const std::string &output_dir);

  /** @brief GET /api/file/download/map */
  bool download_map(const std::string &output_dir);

  /** @brief POST /api/file/upload/calib */
  bool upload_calib(const std::string &file_path);

  /* ---------------------------------------------------------------- */
  /*  Network & System                                                  */
  /* ---------------------------------------------------------------- */

  /** @brief GET /api/network — raw JSON */
  bool get_network(std::string &response_json);

  /** @brief POST /api/network — send JSON config */
  bool set_network(const std::string &request_json);

  /** @brief POST /api/reboot */
  bool reboot();

  /* ---------------------------------------------------------------- */
  /*  Legacy HTTPS API (deprecated — will be removed in ~2 versions)    */
  /* ---------------------------------------------------------------- */

  bool download_device_logs(const std::string &base_url, const std::string &output_dir);
  bool download_calibration(const std::string &base_url, const std::string &output_dir);
  bool download_relocation_map(const std::string &base_url, const std::string &output_dir);
  bool upload_firmware_and_OTA(const std::string &base_url, const std::string &file_path);

  /* ---------------------------------------------------------------- */
  /*  Configuration                                                     */
  /* ---------------------------------------------------------------- */

  void set_handle_opts(const handle_opts &opts) { handle_options_ = opts; }

 private:
  std::string base_url_;
  uint64_t download_timeout_ms_ = 10000;
  struct handle_opts handle_options_;

  /* --- reused transport primitives (from legacy net_client) --- */

  /** Streaming GET → save to file with progress (works for HTTP and HTTPS) */
  bool download(const std::string &url, const std::string &output_dir);

  /** Streaming POST → read file body with progress (works for HTTP and HTTPS) */
  bool upload(const std::string &url, const std::string &local_path);

  /** Legacy download helper: GET /api/download/<name> */
  bool download_file_legacy(const std::string &base_url, const std::string &name,
                            const std::string &output_dir);

  /** Legacy upload helper: POST /api/upload/<basename> */
  bool upload_file_legacy(const std::string &base_url, const std::string &file_path);

  /** Legacy OTA trigger: POST /api/OTA */
  bool trigger_ota_legacy(const std::string &base_url);

  /* --- new transport primitives --- */

  /**
   * @brief Simple HTTP request returning status code + response body.
   * @return HTTP status code, or 0 on connection error
   */
  int simple_request(const std::string &method, const std::string &uri,
                     const std::string &extra_headers, const void *body,
                     size_t body_len, std::string &response_body,
                     int timeout_ms = 5000);
};

}  /* namespace http */

/* ------------------------------------------------------------------ */
/*  Backward compatibility alias (deprecated)                           */
/* ------------------------------------------------------------------ */

namespace crypto {
namespace net {
using Client = ::http::Client;
}  /* namespace net */
}  /* namespace crypto */
