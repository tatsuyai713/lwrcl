#ifndef RCL_LIKE_WRAPPER_SIGNAL_HANDLER_HPP_
#define RCL_LIKE_WRAPPER_SIGNAL_HANDLER_HPP_

#include <csignal>
#include <functional>

#define SIGNAL_HANDLER_DEFINE()             \
  namespace lwrcl                           \
  {                                         \
    extern void signal_handler(int signal); \
  }

#define SIGNAL_HANDLER_INIT()                                                                            \
  if (std::signal(SIGINT, signal_handler) == SIG_ERR || std::signal(SIGTERM, signal_handler) == SIG_ERR) \
  {                                                                                                      \
    throw std::runtime_error("Failed to set signal handler.");                                           \
  }

#endif // RCL_LIKE_WRAPPER_SIGNAL_HANDLER_HPP_
