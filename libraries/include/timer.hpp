#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>

namespace rcl_like_wrapper
{

  class Timer
  {
  public:
    using CallbackFunction = std::function<void()>;

    Timer(std::chrono::milliseconds period, CallbackFunction callback_function)
        : period_(period), expired_(false), callback_function_(callback_function)
    {
      last_execution_ = std::chrono::steady_clock::now();
    }

    void check_and_call()
    {
      auto now = std::chrono::steady_clock::now();
      if (now - last_execution_ >= period_)
      {
        last_execution_ = now;
        if (callback_function_)
        {
          callback_function_();
        }
      }
    }
    void reset()
    {
      last_execution_ = std::chrono::steady_clock::now();
    }

  private:
    std::chrono::milliseconds period_;
    std::chrono::steady_clock::time_point last_execution_;
    CallbackFunction callback_function_;
    std::atomic<bool> expired_;
  };

} // namespace rcl_like_wrapper
