#pragma once

#include <cstdint>
#include <string>

/* forward declarations from mongoose (avoid #include "mongoose/mongoose.h") */
extern "C" {
extern int mg_log_level;
void mg_log_set_fn(void (*fn)(char, void *), void *param);
}

namespace http {

/* ------------------------------------------------------------------ */
/*  Legacy HTTPS API endpoints (deprecated, will be removed)            */
/* ------------------------------------------------------------------ */

#define URI_API_HEALTH "/api/health"
#define URI_API_FILES_UPLOAD_PREFIX "/api/upload/"
#define URI_API_FILES_DOWNLOAD_PREFIX "/api/download/"
#define URI_API_FIRMWARE_UPLOAD "/api/OTA"

#define DEVICE_LOG_FILE "logs"
#define CALIBRATION_FILE "calibration"
#define RELOCATION_MAP_FILE "relocation_map"

/* ------------------------------------------------------------------ */
/*  New HTTP API endpoints (matching server http_api.h)                 */
/* ------------------------------------------------------------------ */

#define URI_API_VERSION "/api/version"
#define URI_API_DEVICE_INFO "/api/device_info"
#define URI_API_HEARTBEAT "/api/heartbeat"

#define URI_API_OTA_UPLOAD "/api/ota/upload"
#define URI_API_OTA_TRIGGER "/api/ota/trigger"
#define URI_API_OTA_STATUS "/api/ota/status"
#define URI_API_OTA_RESET "/api/ota/reset"

#define URI_API_FILE_DOWNLOAD_LOG "/api/file/download/log"
#define URI_API_FILE_DOWNLOAD_CALIB "/api/file/download/calib"
#define URI_API_FILE_DOWNLOAD_MAP "/api/file/download/map"
#define URI_API_FILE_UPLOAD_CALIB "/api/file/upload/calib"

#define URI_API_NETWORK "/api/network"
#define URI_API_REBOOT "/api/reboot"

/* ------------------------------------------------------------------ */
/*  OTA status (returned by GET /api/ota/status)                        */
/* ------------------------------------------------------------------ */

struct OtaStatus {
  std::string state; /* IDLE, UPLOADING, VERIFYING, INSTALLING_MCU,
                        INSTALLING_SOC, REBOOTING, POST_VERIFY, DONE, FAILED */
  int progress;      /* 0-100 */
  std::string message;
  std::string error;
  std::string mcu_result;
};

/** @brief Callback invoked on each OTA status poll during ota_wait_complete */
typedef void (*OtaStatusCallback)(const OtaStatus &status);

/* ------------------------------------------------------------------ */
/*  Callback types                                                      */
/* ------------------------------------------------------------------ */

struct handle_opts {
  char upload_dir[4096];
  char download_dir[4096];

  int (*before_download_hook)(const char *fname, char *fpath_out);
  int (*progress_download_callback)(uint64_t downloaded, uint64_t total);
  int (*after_download_hook)(const char *fname, const char *fpath_out);

  int (*before_upload_hook)(const char *fpath);
  int (*progress_upload_callback)(uint64_t uploaded, uint64_t total);
  int (*after_upload_hook)(const char *fpath);

  int (*ota_callback)(void);
};

using ProgressCallback = int (*)(uint64_t current, uint64_t total);

/* ------------------------------------------------------------------ */
/*  Mongoose log level wrapper                                          */
/* ------------------------------------------------------------------ */

enum LogLevel {
  kLogNone = 0,    /* MG_LL_NONE */
  kLogError = 1,   /* MG_LL_ERROR */
  kLogInfo = 2,    /* MG_LL_INFO */
  kLogDebug = 3,   /* MG_LL_DEBUG */
  kLogVerbose = 4, /* MG_LL_VERBOSE */
};

using LogFn = void (*)(char, void *);

inline void log_set_level(int level) { mg_log_level = level; }

inline void log_set_fn(LogFn fn, void *param) { mg_log_set_fn(fn, param); }

}  /* namespace http */

/* ------------------------------------------------------------------ */
/*  Backward compatibility aliases (deprecated)                         */
/* ------------------------------------------------------------------ */

namespace crypto {
namespace net {
using handle_opts = ::http::handle_opts;
using ProgressCallback = ::http::ProgressCallback;
}  /* namespace net */
}  /* namespace crypto */
