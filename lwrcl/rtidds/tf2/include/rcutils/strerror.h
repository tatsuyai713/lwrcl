#ifndef RCUTILS__STRERROR_H_
#define RCUTILS__STRERROR_H_

#include <cerrno>
#include <cstring>

inline const char * rcutils_strerror(char * buffer, size_t buffer_size)
{
  if (buffer == nullptr || buffer_size == 0) {
    return "";
  }
#if defined(_GNU_SOURCE)
  const char * msg = strerror_r(errno, buffer, buffer_size);
  if (msg != nullptr) {
    std::strncpy(buffer, msg, buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
  }
#else
  strerror_r(errno, buffer, buffer_size);
#endif
  return buffer;
}

#endif  // RCUTILS__STRERROR_H_
