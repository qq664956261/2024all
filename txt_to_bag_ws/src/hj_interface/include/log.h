#ifndef HJ_INTERFACE_INCLUDE_LOG_H
#define HJ_INTERFACE_INCLUDE_LOG_H
#include "ros/ros.h"

#define LOG_USE_ROS 1
#define LOG_USE_LOG4CXX 2
#define LOG_USE_NONE 3
#define LOG_USE_RELEASE 4

#ifndef CONFIG_ENABLE_LOG
#define CONFIG_ENABLE_LOG LOG_USE_ROS
#endif
#if CONFIG_ENABLE_LOG == LOG_USE_ROS
#define HJ_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define HJ_INFO(...) ROS_INFO(__VA_ARGS__)
#define HJ_WARN(...) ROS_WARN(__VA_ARGS__)
#define HJ_ERROR(...)           \
  ROS_ERROR(__VA_ARGS__);       \
  fprintf(stderr, __VA_ARGS__); \
  fprintf(stderr, "\r\n")
#define HJ_EFATAL(...)          \
  ROS_FATAL(__VA_ARGS__);       \
  fprintf(stderr, __VA_ARGS__); \
  fprintf(stderr, "\r\n")
#define HJ_IMPORTANT(...)           \
  ROS_INFO(__VA_ARGS__);       \
  fprintf(stderr, __VA_ARGS__); \
  fprintf(stderr, "\r\n")
#define HJ_DEBUG_STREAM(args) ROS_DEBUG_STREAM(args)
#define HJ_INFO_STREAM(args) ROS_INFO_STREAM(args)
#define HJ_WARN_STREAM(args) ROS_WARN_STREAM(args)
#define HJ_ERROR_STREAM(args) \
  ROS_ERROR_STREAM(args);     \
  std::cerr << args << std::endl
#define HJ_EFATAL_STREAM(args) \
  ROS_FATAL_STREAM(args);     \
  std::cerr << args << std::endl
#define HJ_IMPORTANT_STREAM(args) \
  ROS_INFO_STREAM(args);      \
  std::cerr << args << std::endl

#define HJ_DEBUG_THROTTLE(...) ROS_DEBUG_THROTTLE(__VA_ARGS__)
#define HJ_INFO_THROTTLE(...) ROS_INFO_THROTTLE(__VA_ARGS__)
#define HJ_WARN_THROTTLE(...) ROS_WARN_THROTTLE(__VA_ARGS__)
#define HJ_ERROR_THROTTLE(...)     \
  ROS_ERROR_THROTTLE(__VA_ARGS__); \
  fprintf(stderr, __VA_ARGS__);    \
  fprintf(stderr, "\r\n")
#define HJ_EFATAL_THROTTLE(...)    \
  ROS_FATAL_THROTTLE(__VA_ARGS__); \
  fprintf(stderr, __VA_ARGS__);    \
  fprintf(stderr, "\r\n")

//#define HJ_DEBUG_STREAM_THROTTLE(args) ROS_DEBUG_STREAM_THROTTLE(args)
//#define HJ_INFO_STREAM_THROTTLE(args) ROS_INFO_STREAM_THROTTLE(args)
//#define HJ_WARN_STREAM_THROTTLE(args) ROS_WARN_STREAM_THROTTLE(args)
//#define HJ_ERROR_STREAM_THROTTLE(args) ROS_ERROR_STREAM_THROTTLE(args)
//#define HJ_EFATAL_STREAM_THROTTLE(args) ROS_FATAL__STREAM_THROTTLE(args)

#define _CHECK2(expr, msg) do{\
if(!(expr)){\
    std::stringstream ss_msg_fatal;\
    ss_msg_fatal << "Check failed: " << #expr << " @ " << __FILE__ << ":" << __LINE__ << "\n" << msg;\
    throw std::runtime_error(ss_msg_fatal.str());\
}}while(0)

#endif
#if CONFIG_ENABLE_LOG == LOG_USE_NONE
#define HJ_DEBUG(...)
#define HJ_INFO(...)
#define HJ_WARN(...)
#define HJ_ERROR(...)
#define HJ_EFATAL(...)
#define HJ_IMPORTANT(...)

#define HJ_DEBUG_STREAM(args)
#define HJ_INFO_STREAM(args)
#define HJ_WARN_STREAM(args)
#define HJ_ERROR_STREAM(args)
#define HJ_EFATAL_STREAM(args)
#define HJ_IMPORTANT_STREAM(args)

#define HJ_DEBUG_THROTTLE(...)
#define HJ_INFO_THROTTLE(...)
#define HJ_WARN_THROTTLE(...)
#define HJ_ERROR_THROTTLE(...)
#define HJ_EFATAL_THROTTLE(...)

#define _CHECK2(expr, msg)
#endif
#if CONFIG_ENABLE_LOG == LOG_USE_LOG4CXX
#include <my_log4cxx/log4cxx/basicconfigurator.h>
#include <my_log4cxx/log4cxx/logger.h>
#include <my_log4cxx/log4cxx/logmanager.h>
#include <my_log4cxx/log4cxx/propertyconfigurator.h>

#define HJ_DEBUG(...)   // LOG4CXX_DEBUG( log4cxx::Logger::getRootLogger() , __VA_ARGS__);
#define HJ_INFO(...)    // LOG4CXX_INFO( log4cxx::Logger::getRootLogger() , __VA_ARGS__);
#define HJ_WARN(...)    // LOG4CXX_WARN( log4cxx::Logger::getRootLogger() , __VA_ARGS__);
#define HJ_ERROR(...)   // LOG4CXX_ERROR( log4cxx::Logger::getRootLogger() , __VA_ARGS__);
#define HJ_EFATAL(...)  // LOG4CXX_FATAL( log4cxx::Logger::getRootLogger() , __VA_ARGS__);
#endif

#if CONFIG_ENABLE_LOG == LOG_USE_RELEASE

#define HJ_DEBUG(...)
#define HJ_INFO(...)
#define HJ_WARN(...)
#define HJ_ERROR(...)           \
  ROS_ERROR(__VA_ARGS__);       \
  fprintf(stderr, __VA_ARGS__); \
  fprintf(stderr, "\r\n")
#define HJ_EFATAL(...)          \
  ROS_FATAL(__VA_ARGS__);       \
  fprintf(stderr, __VA_ARGS__); \
  fprintf(stderr, "\r\n")
#define HJ_IMPORTANT(...)           \
  fprintf(stderr, __VA_ARGS__); \
  fprintf(stderr, "\r\n")
#define HJ_DEBUG_STREAM(args)
#define HJ_INFO_STREAM(args)
#define HJ_WARN_STREAM(args)
#define HJ_ERROR_STREAM(args) \
  ROS_ERROR_STREAM(args);     \
  std::cerr << args << std::endl
#define HJ_EFATAL_STREAM(args) \
  ROS_FATAL_STREAM(args);     \
  std::cerr << args << std::endl
#define HJ_IMPORTANT_STREAM(args) \
  std::cerr << args << std::endl

#define HJ_DEBUG_THROTTLE(...)
#define HJ_INFO_THROTTLE(...)
#define HJ_WARN_THROTTLE(...)
#define HJ_ERROR_THROTTLE(...)     \
  ROS_ERROR_THROTTLE(__VA_ARGS__); \
  fprintf(stderr, __VA_ARGS__);    \
  fprintf(stderr, "\r\n")
#define HJ_EFATAL_THROTTLE(...)    \
  ROS_FATAL_THROTTLE(__VA_ARGS__); \
  fprintf(stderr, __VA_ARGS__);    \
  fprintf(stderr, "\r\n")

#define _CHECK2(expr, msg) do{\
if(!(expr)){\
    std::stringstream ss_msg_fatal;\
    ss_msg_fatal << "Check failed: " << #expr << " @ " << __FILE__ << ":" << __LINE__ << "\n" << msg;\
    throw std::runtime_error(ss_msg_fatal.str());\
}}while(0)

#endif
#define HJ_CHECK1(expr) _CHECK2(expr, "")


#define HJ_CHECK_EQ3(lhs, rhs, msg) _CHECK2((lhs) == (rhs), msg)
#define HJ_CHECK_NE3(lhs, rhs, msg) _CHECK2((lhs) != (rhs), msg)
#define HJ_CHECK_LE3(lhs, rhs, msg) _CHECK2((lhs) <= (rhs), msg)
#define HJ_CHECK_LT3(lhs, rhs, msg) _CHECK2((lhs) < (rhs), msg)
#define HJ_CHECK_GE3(lhs, rhs, msg) _CHECK2((lhs) >= (rhs), msg)
#define HJ_CHECK_GT3(lhs, rhs, msg) _CHECK2((lhs) > (rhs), msg)

#define HJ_CHECK_EQ2(lhs, rhs) HJ_CHECK_EQ3(lhs, rhs, "")
#define HJ_CHECK_NE2(lhs, rhs) HJ_CHECK_NE3(lhs, rhs, "")
#define HJ_CHECK_LE2(lhs, rhs) HJ_CHECK_LE3(lhs, rhs, "")
#define HJ_CHECK_LT2(lhs, rhs) HJ_CHECK_LT3(lhs, rhs, "")
#define HJ_CHECK_GE2(lhs, rhs) HJ_CHECK_GE3(lhs, rhs, "")
#define HJ_CHECK_GT2(lhs, rhs) HJ_CHECK_GT3(lhs, rhs, "")

#endif
