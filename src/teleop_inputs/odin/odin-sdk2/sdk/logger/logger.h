#pragma once
#include <time.h>
#include <stdint.h>
#include <assert.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif
/* logger module
@@ support change log level by deliver env.
@@ log format: [time]<filename:func:line><level>: string
 */

typedef int (*log_func)(const char *fmt, ...);

extern unsigned int logger_level;
extern log_func g_logPrint;
extern int g_enableSyslog;

// ANSI color codes
#define COLOR_RESET "\033[0m"
#define COLOR_RED "\033[1;31m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_PURPLE "\033[1;35m"
#define COLOR_CYAN "\033[1;36m"
#define COLOR_WHITE "\033[1;37m"

#define LOG_LEVEL_BUG 0
#define LOG_LEVEL_ERROR 0
#define LOG_LEVEL_WARN 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3

#define BUG "BUG"
#define ERROR "ERROR"
#define WARN "WARN"
#define INFO "INFO"
#define DEBUG "DEBUG"

#define __BI_FILENAME__ \
  (__builtin_strrchr(__FILE__, '/') ? __builtin_strrchr(__FILE__, '/') + 1 : __FILE__)

#define GET_TIMESTAMP(buffer)                            \
  do {                                                   \
    struct timespec ts;                                  \
    clock_gettime(CLOCK_REALTIME, &ts);                  \
    struct tm tm_info;                                   \
    localtime_r(&ts.tv_sec, &tm_info);                   \
    strftime(buffer, 20, "%Y:%m:%d:%H:%M:%S", &tm_info); \
  } while (0)

#define LOG_COLOR_BUG COLOR_PURPLE
#define LOG_COLOR_ERROR COLOR_RED
#define LOG_COLOR_WARN COLOR_YELLOW
#define LOG_COLOR_INFO COLOR_GREEN
#define LOG_COLOR_DEBUG COLOR_BLUE

#define LOG_INFO(format, ...)                                                          \
  do {                                                                                 \
    if (logger_level < LOG_LEVEL_INFO) {                                               \
      break;                                                                           \
    }                                                                                  \
    char timestamp[20];                                                                \
    GET_TIMESTAMP(timestamp);                                                          \
    g_logPrint(COLOR_GREEN "[%s][%s][%s:%s:%d]: " format COLOR_RESET, timestamp, INFO, \
               __BI_FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);                \
  } while (0)

#define LOG_WARN(format, ...)                                                           \
  do {                                                                                  \
    if (logger_level < LOG_LEVEL_WARN) {                                                \
      break;                                                                            \
    }                                                                                   \
    char timestamp[20];                                                                 \
    GET_TIMESTAMP(timestamp);                                                           \
    g_logPrint(COLOR_YELLOW "[%s][%s][%s:%s:%d]: " format COLOR_RESET, timestamp, WARN, \
               __BI_FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);                 \
  } while (0)

#define LOG_ERROR(format, ...)                                                        \
  do {                                                                                \
    char timestamp[20];                                                               \
    GET_TIMESTAMP(timestamp);                                                         \
    g_logPrint(COLOR_RED "[%s][%s][%s:%s:%d]: " format COLOR_RESET, timestamp, ERROR, \
               __BI_FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);               \
  } while (0)

#define LOG_BUG(format, ...)                                                           \
  do {                                                                                 \
    char timestamp[20];                                                                \
    GET_TIMESTAMP(timestamp);                                                          \
    g_logPrint(COLOR_PURPLE "[%s][%s][%s:%s:%d]: " format COLOR_RESET, timestamp, BUG, \
               __BI_FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);                \
  } while (0)

#define LOG_DEBUG(format, ...)                                                         \
  do {                                                                                 \
    if (logger_level < LOG_LEVEL_DEBUG) {                                              \
      break;                                                                           \
    }                                                                                  \
    char timestamp[20];                                                                \
    GET_TIMESTAMP(timestamp);                                                          \
    g_logPrint(COLOR_BLUE "[%s][%s][%s:%s:%d]: " format COLOR_RESET, timestamp, DEBUG, \
               __BI_FILENAME__, __FUNCTION__, __LINE__, ##__VA_ARGS__);                \
  } while (0)

#define DEBUGSIT                                                                                  \
  if (logger_level < LOG_LEVEL_DEBUG) {                                                           \
    break;                                                                                        \
  }                                                                                               \
  char timestamp[20];                                                                             \
  GET_TIMESTAMP(timestamp);                                                                       \
  g_fLogPrint(COLOR_BLUE "[%s][%s][%s:%s:%d]:.\n" COLOR_RESET, timestamp, DEBUG, __BI_FILENAME__, \
              __FUNCTION__, __LINE__);                                                            \
  }                                                                                               \
  while (0)

int log_config(unsigned int level, log_func func);

typedef struct {
  struct timespec start;
} cost_handle_t;

void cost_start(cost_handle_t *h);
void cost_end(cost_handle_t *h);

typedef struct {
  struct timespec start;
  struct timespec last;
  struct timespec end;
  int print_enable;
  int count;
  int count_target;
  uint64_t total_ns;
} ts_handle_t;

static inline void _ts(ts_handle_t *handle, const char *func, int line, const char *str) {
  if (!handle->print_enable) return;

  clock_gettime(CLOCK_MONOTONIC, &handle->end);

  uint64_t duration_ns = (handle->end.tv_sec - handle->last.tv_sec) * 1000000000ULL +
                         (handle->end.tv_nsec - handle->last.tv_nsec);

  if (handle->count == 0) {
    handle->start = handle->last = handle->end;
    handle->total_ns = 0;
  }

  handle->total_ns += duration_ns;
  handle->count++;
  handle->last = handle->end;

  if (handle->count >= handle->count_target) {
    double avg_ms = (double)handle->total_ns / handle->count / 1e6;
    double total_ms = (double)handle->total_ns / 1e6;

    g_logPrint("<%s:%d>: %s AVG[%.3f ms] TOTAL[%.3f ms] COUNT[%d]\n", func, line, str, avg_ms,
               total_ms, handle->count);

    handle->count = 0;
    handle->total_ns = 0;
  }
}

static inline void ts_reset(ts_handle_t *handle) {
  clock_gettime(CLOCK_MONOTONIC, &handle->start);
  handle->last = handle->end = handle->start;
}

#define tsprint(handle, str) _ts(handle, __FUNCTION__, __LINE__, str)
#define tsobject(name, enable, n)               \
  static ts_handle_t obj_##name = {0};          \
  ts_handle_t *name = &obj_##name;              \
  name->print_enable = !!(enable);              \
  name->count_target = (n);                     \
  clock_gettime(CLOCK_MONOTONIC, &name->start); \
  name->last = name->end = name->start;

#ifdef __cplusplus
}
#endif