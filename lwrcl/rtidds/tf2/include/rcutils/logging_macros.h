#ifndef RCUTILS__LOGGING_MACROS_H_
#define RCUTILS__LOGGING_MACROS_H_

#include <cstdio>

#define RCUTILS_STEADY_TIME 0

#define RCUTILS_LOG_ERROR(...) \
  do { \
    std::fprintf(stderr, __VA_ARGS__); \
    std::fprintf(stderr, "\n"); \
  } while (0)

#define RCUTILS_LOG_WARN(...) \
  do { \
    std::fprintf(stderr, __VA_ARGS__); \
    std::fprintf(stderr, "\n"); \
  } while (0)

#define RCUTILS_LOG_WARN_THROTTLE(clock, duration_ms, ...) \
  do { \
    (void)(clock); \
    (void)(duration_ms); \
    std::fprintf(stderr, __VA_ARGS__); \
    std::fprintf(stderr, "\n"); \
  } while (0)

#endif  // RCUTILS__LOGGING_MACROS_H_
