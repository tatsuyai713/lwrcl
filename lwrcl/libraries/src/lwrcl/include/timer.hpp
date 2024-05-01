#ifndef RCL_LIKE_WRAPPER_TIMER_HPP_
#define RCL_LIKE_WRAPPER_TIMER_HPP_

#include <atomic>
#include <functional>
#include <iostream>
#include <thread>
#include <chrono>

#include "fast_dds_header.hpp"
#include "channel.hpp"

namespace lwrcl
{

  class TimerCallback : public ChannelCallback
  {
  public:
    TimerCallback(std::function<void()> callback_function)
        : callback_function_(callback_function) {}

    ~TimerCallback() = default;

    void invoke()
    {
      try
      {
        callback_function_();
      }
      catch (const std::exception &e)
      {
        std::cerr << "Exception during callback invocation: " << e.what() << std::endl;
      }
      catch (...)
      {
        std::cerr << "Unknown exception during callback invocation." << std::endl;
      }
    }

  private:
    std::function<void()> callback_function_;
  };

  class ITimer
  {
  public:
    virtual ~ITimer() = default;
  };

  template <typename DurationType>
  class Timer : public ITimer
  {
  public:
    Timer(DurationType period, std::function<void()> callback_function, Channel<ChannelCallback *> &channel)
        : period_(period), channel_(channel), stop_flag_(false)
    {
      timer_callback_ = std::make_unique<TimerCallback>(callback_function);
      start();
    }

    ~Timer()
    {
      stop();
    }

    void start()
    {

      worker_ = std::thread([this]()
                            { run(); });
    }

    void stop()
    {
      stop_flag_ = true;
      if (worker_.joinable())
      {
        worker_.join(); // Wait for the worker thread to finish
      }
    }

  private:
    void run()
    {
      auto next_execution_time = std::chrono::steady_clock::now() + period_;
      while (!stop_flag_)
      {
        std::this_thread::sleep_until(next_execution_time);
        channel_.produce(timer_callback_.get());
        next_execution_time += period_; // Schedule next execution
      }
    }
    bool stop_flag_;
    DurationType period_;
    std::unique_ptr<TimerCallback> timer_callback_;
    std::thread worker_;
    Channel<ChannelCallback *> &channel_;
  };

} // namespace lwrcl

#endif // RCL_LIKE_WRAPPER_TIMER_HPP_