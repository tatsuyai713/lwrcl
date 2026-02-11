#ifndef LWRCL_TIMER_HPP_
#define LWRCL_TIMER_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "channel.hpp"
#include "clock_time_duration.hpp"

namespace lwrcl
{

  class TimerCallback : public ChannelCallback
  {
  public:
    explicit TimerCallback(std::function<void()> callback_function)
        : callback_function_(callback_function)
    {
    }

    ~TimerCallback() = default;

    TimerCallback(const TimerCallback &) = delete;
    TimerCallback &operator=(const TimerCallback &) = delete;
    TimerCallback(TimerCallback &&) = default;
    TimerCallback &operator=(TimerCallback &&) = default;

    void invoke() override
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

  class ITimerBase
  {
  public:
    virtual ~ITimerBase() = default;
    virtual void start() = 0;
    virtual void stop() = 0;

    ITimerBase(const ITimerBase &) = delete;
    ITimerBase &operator=(const ITimerBase &) = delete;
    ITimerBase(ITimerBase &&) = default;
    ITimerBase &operator=(ITimerBase &&) = default;

  protected:
    ITimerBase() = default;
  };

  class TimerBase : public ITimerBase, public std::enable_shared_from_this<TimerBase>
  {
  public:
    using SharedPtr = std::shared_ptr<TimerBase>;

    TimerBase(
      Duration period, std::function<void()> callback_function,
      CallbackChannel::SharedPtr channel, Clock::ClockType clock_type)
        : ITimerBase(),
          std::enable_shared_from_this<TimerBase>(),
          clock_type_(clock_type),
          period_(period),
          timer_callback_(std::make_shared<TimerCallback>(callback_function)),
          worker_(),
          channel_(channel),
          stop_flag_(false)
    {
      start();
    }

    ~TimerBase() { stop(); }

    TimerBase(const TimerBase &) = delete;
    TimerBase &operator=(const TimerBase &) = delete;
    TimerBase(TimerBase &&) = default;
    TimerBase &operator=(TimerBase &&) = default;

    void start() override
    {
      worker_ = std::thread([this]()
                            { run(); });
    }

    void stop() override
    {
      stop_flag_.store(true);
      if (worker_.joinable())
      {
        worker_.join(); // Wait for the worker thread to finish
      }
    }

  private:
    void run_system_time()
    {
      auto next_execution_time = std::chrono::system_clock::now() + std::chrono::nanoseconds(period_.nanoseconds());
      while (!stop_flag_.load())
      {
        std::this_thread::sleep_until(next_execution_time);
        if (!stop_flag_.load())
        {
          channel_->produce(timer_callback_);
          next_execution_time += std::chrono::nanoseconds(period_.nanoseconds()); // Schedule next execution
        }
      }
    }

    void run_steady_time()
    {
      auto next_execution_time = std::chrono::steady_clock::now() + std::chrono::nanoseconds(period_.nanoseconds());
      while (!stop_flag_.load())
      {
        std::this_thread::sleep_until(next_execution_time);
        if (!stop_flag_.load())
        {
          channel_->produce(timer_callback_);
          next_execution_time += std::chrono::nanoseconds(period_.nanoseconds()); // Schedule next execution
        }
      }
    }

    void run()
    {
      if (clock_type_ == Clock::ClockType::SYSTEM_TIME)
      {
        run_system_time();
      }
      else
      {
        run_steady_time();
      }
    }

    Clock::ClockType clock_type_;
    Duration period_;
    std::shared_ptr<TimerCallback> timer_callback_;
    std::thread worker_;
    CallbackChannel::SharedPtr channel_;
    std::atomic<bool> stop_flag_;
  };

} // namespace lwrcl

#endif // LWRCL_TIMER_HPP_
