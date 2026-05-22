#ifndef LWRCL_TIMER_HPP_
#define LWRCL_TIMER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "fast_dds_header.hpp"
#include "clock_time_duration.hpp"

namespace lwrcl
{

  class ITimerBase
  {
  public:
    virtual ~ITimerBase() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool execute_if_ready() = 0;
    virtual std::chrono::nanoseconds time_until_next_call() const = 0;

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
      std::shared_ptr<std::mutex> node_mutex, Clock::ClockType clock_type)
        : ITimerBase(),
          std::enable_shared_from_this<TimerBase>(),
          period_(period),
          callback_function_(std::move(callback_function)),
          node_mutex_(std::move(node_mutex)),
          clock_(clock_type),
          next_execution_time_(clock_.now() + period_),
          stop_flag_(false),
          is_canceled_(false)
    {
    }

    ~TimerBase() { stop(); }

    TimerBase(const TimerBase &) = delete;
    TimerBase &operator=(const TimerBase &) = delete;
    TimerBase(TimerBase &&) = delete;
    TimerBase &operator=(TimerBase &&) = delete;

    void start() override
    {
      is_canceled_.store(false);
      stop_flag_.store(false);
      std::lock_guard<std::mutex> lock(next_execution_mutex_);
      next_execution_time_ = clock_.now() + period_;
    }

    void stop() override
    {
      stop_flag_.store(true);
    }

    bool execute_if_ready() override
    {
      if (stop_flag_.load() || is_canceled_.load())
      {
        return false;
      }

      auto now = clock_.now();
      {
        std::lock_guard<std::mutex> lock(next_execution_mutex_);
        if (now < next_execution_time_)
        {
          return false;
        }

        if (period_.nanoseconds() > 0)
        {
          do
          {
            next_execution_time_ = next_execution_time_ + period_;
          } while (next_execution_time_ <= now);
        }
        else
        {
          next_execution_time_ = now;
        }
      }

      execute_callback();
      return true;
    }

    std::chrono::nanoseconds time_until_next_call() const override
    {
      if (stop_flag_.load() || is_canceled_.load())
      {
        return std::chrono::nanoseconds::max();
      }
      auto now = clock_.now();
      std::lock_guard<std::mutex> lock(next_execution_mutex_);
      if (now >= next_execution_time_)
      {
        return std::chrono::nanoseconds::zero();
      }
      return std::chrono::nanoseconds((next_execution_time_ - now).nanoseconds());
    }

    // Cancel the timer (alias for stop)
    void cancel()
    {
      is_canceled_.store(true);
      stop();
    }

    // Check if the timer is canceled
    bool is_canceled() const
    {
      return is_canceled_.load();
    }

    // Reset the timer (restart)
    void reset()
    {
      is_canceled_.store(false);
      stop_flag_.store(false);
      std::lock_guard<std::mutex> lock(next_execution_mutex_);
      next_execution_time_ = clock_.now() + period_;
    }

    // Get the timer period
    Duration get_period() const
    {
      return period_;
    }

    // Check if the timer is ready (always returns true for now)
    bool is_ready() const
    {
      return !is_canceled_.load() && !stop_flag_.load();
    }

  private:
    void execute_callback()
    {
      std::unique_lock<std::mutex> callback_lock;
      if (node_mutex_)
      {
        callback_lock = std::unique_lock<std::mutex>(*node_mutex_);
      }
      try { callback_function_(); }
      catch (const std::exception &e) {
        std::fprintf(stderr, "Exception in timer callback: %s\n", e.what());
      } catch (...) {
        std::fprintf(stderr, "Unknown exception in timer callback.\n");
      }
    }

    Duration period_;
    std::function<void()> callback_function_;
    std::shared_ptr<std::mutex> node_mutex_;
    mutable Clock clock_;
    mutable std::mutex next_execution_mutex_;
    Time next_execution_time_;
    std::atomic<bool> stop_flag_;
    std::atomic<bool> is_canceled_;
  };

} // namespace lwrcl

#endif // LWRCL_TIMER_HPP_
