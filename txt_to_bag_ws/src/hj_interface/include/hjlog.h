#pragma once
#include <sys/types.h>

#define TRACE_LOG 0
#define DEBUG_LOG 1
#define INFO_LOG  2
#define WARN_LOG  3
#define ERROR_LOG 4
#define FATAL_LOG 5

ssize_t hjlog_append(void*, unsigned int,
               const char*, unsigned int, bool, const char *format, ...);
ssize_t msg_append(void*, const char *format, ...);

#ifdef ENABLE_CSTLOG
#define HJ_CST_TRACE(log, fmt, ...)      \
    HJ_LOG_PRIN(log, TRACE_LOG, false, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_DEBUG(log, fmt, ...)      \
    HJ_LOG_PRIN(log, DEBUG_LOG, false, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_INFO(log, fmt, ...)      \
    HJ_LOG_PRIN(log, INFO_LOG, false, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_WARN(log, fmt, ...)      \
    HJ_LOG_PRIN(log, WARN_LOG, false, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_ERROR(log, fmt, ...)      \
    HJ_LOG_PRIN(log, ERROR_LOG, false, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_FATAL(log, fmt, ...)      \
    HJ_LOG_PRIN(log, FATAL_LOG, false, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)


#define HJ_CST_TIME_TRACE(log, fmt, ...)      \
    HJ_LOG_PRIN(log, TRACE_LOG, true, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_TIME_DEBUG(log, fmt, ...)      \
    HJ_LOG_PRIN(log, DEBUG_LOG, true, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_TIME_INFO(log, fmt, ...)      \
    HJ_LOG_PRIN(log, INFO_LOG, true, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_TIME_WARN(log, fmt, ...)      \
    HJ_LOG_PRIN(log, WARN_LOG, true, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_TIME_ERROR(log, fmt, ...)      \
    HJ_LOG_PRIN(log, ERROR_LOG, true, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)
#define HJ_CST_TIME_FATAL(log, fmt, ...)      \
    HJ_LOG_PRIN(log, FATAL_LOG, true, __LINE__,  __func__, __FILE__, fmt, ##__VA_ARGS__)


#define HJ_LOG_PRIN(log, level, time, line, func, file, fmt, ...)                         \
    do {                                                                                  \
        hjlog_append(log, level, file, line, time, fmt, ##__VA_ARGS__);            \
    } while(0);

#else
#define HJ_CST_TRACE(log, fmt, ...)    
#define HJ_CST_DEBUG(log, fmt, ...)    
#define HJ_CST_INFO(log, fmt, ...)   
#define HJ_CST_WARN(log, fmt, ...)   
#define HJ_CST_ERROR(log, fmt, ...)   
#define HJ_CST_FATAL(log, fmt, ...)

#define HJ_CST_TIME_TRACE(log, fmt, ...)
#define HJ_CST_TIME_DEBUG(log, fmt, ...)
#define HJ_CST_TIME_INFO(log, fmt, ...)
#define HJ_CST_TIME_WARN(log, fmt, ...)
#define HJ_CST_TIME_ERROR(log, fmt, ...)
#define HJ_CST_TIME_FATAL(log, fmt, ...)

#endif


/**
 * @brief Create customer logger handler.
 *
 * @param file Absolute log name, prefix only take "/tmp/logging" and "/userdata/hj/log"
 * @param level Log level.
 * @param size Max single file size, in bytes.
 * @param num Max roll file number.
 * @return The logger handler, success: handler pointer, fail:NULL
 */
void* hj_cst_log_add(const char* file, unsigned int level, unsigned int size, unsigned int num);

/**
 * @brief Delete customer logger handler.
 *
 * @param log Customer logger handler created.
 */
void hj_cst_log_del(void* log);

/**
 * @brief Set customer log level.
 *
 * @param log Customer logger handler created.
 * @param level Log level.
 */
void hj_log_set_level(void* log, unsigned int level); 