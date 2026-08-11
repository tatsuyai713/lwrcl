#ifndef RCUTILS__SNPRINTF_H_
#define RCUTILS__SNPRINTF_H_

#include <cstdarg>
#include <cstdio>

inline int rcutils_snprintf(char * buffer, size_t buffer_size, const char * format, ...)
{
  va_list args;
  va_start(args, format);
  int ret = std::vsnprintf(buffer, buffer_size, format, args);
  va_end(args);
  return ret;
}

#endif  // RCUTILS__SNPRINTF_H_
