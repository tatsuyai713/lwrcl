#pragma once

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
        : period_(period), stop_flag_(false), callback_function_(callback_function)
    {
      thread_ = std::thread(&Timer::run, this);
    }

    ~Timer()
    {
      stop_timer();
    }

    void stop_timer()
    {
      if (!stop_flag_.exchange(true) && thread_.joinable())
      {
        thread_.join();
      }
    }

  private:
    void run()
    {
      auto next_time = std::chrono::steady_clock::now() + period_;
      while (!stop_flag_)
      {
        callback_function_();

        next_time += period_;
        auto sleep_time = next_time - std::chrono::steady_clock::now();

        if (sleep_time > std::chrono::milliseconds(0))
        {
          std::this_thread::sleep_for(sleep_time);
        }
      }
    }

    CallbackFunction callback_function_;
    std::thread thread_;
    std::atomic<bool> stop_flag_;
    std::chrono::milliseconds period_;
  };

} // namespace rcl_like_wrapper
