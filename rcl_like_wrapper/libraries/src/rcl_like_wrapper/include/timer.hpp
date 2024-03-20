#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>

namespace rcl_like_wrapper
{

  class ITimer
  {
  public:
    virtual ~ITimer() = default;
    virtual void check_and_call() = 0;
  };

  template <typename DurationType>
  class Timer : public ITimer
  {
  public:
    using CallbackFunction = std::function<void()>;

    Timer(DurationType period, CallbackFunction callback_function)
        : period_(period), callback_function_(callback_function)
    {
      last_execution_ = std::chrono::steady_clock::now();
    }
    ~Timer()
    {
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
    DurationType period_;
    std::chrono::steady_clock::time_point last_execution_;
    CallbackFunction callback_function_;
  };

} // namespace rcl_like_wrapper
