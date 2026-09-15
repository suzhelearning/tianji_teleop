#include <stdio.h>
#include <stdlib.h>
#include "logger.h"

unsigned int logger_level = LOG_LEVEL_WARN;
log_func g_logPrint;

int log_config(unsigned int level, log_func func) {
  char *envLogLevel = NULL;
  if (!func) {
    return -1;
  }
  g_logPrint = func;

  if (NULL != (envLogLevel = getenv("LOG_LEVEL"))) {
    logger_level = atoi(envLogLevel);
    if (logger_level > LOG_LEVEL_DEBUG) {
      LOG_ERROR("env LOG_LEVEL log level %d illegal\n", logger_level);
      return -1;
    }
    LOG_WARN("Use env LOG_LEVEL log level %d\n", logger_level);
  } else {
    if (level > LOG_LEVEL_DEBUG) {
      LOG_ERROR("init loglevel %u illegal\n", level);
      return -1;
    }
    logger_level = level;
    LOG_WARN("init loglevel %d.\n", logger_level);
  }
  return 0;
}

void cost_start(cost_handle_t *h) { clock_gettime(CLOCK_MONOTONIC, &h->start); }

void cost_end(cost_handle_t *h) {
  struct timespec end;
  clock_gettime(CLOCK_MONOTONIC, &end);

  uint64_t duration_ns =
      (end.tv_sec - h->start.tv_sec) * 1000000000ULL + (end.tv_nsec - h->start.tv_nsec);

  printf("cost: %.3f ms\n", duration_ns / 1e6);
}
