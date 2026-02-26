#ifndef LWRCL_TIMER_HPP_
#define LWRCL_TIMER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "clock_time_duration.hpp"

namespace lwrcl
{

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
      std::shared_ptr<std::mutex> node_mutex, Clock::ClockType clock_type)
        : ITimerBase(),
          std::enable_shared_from_this<TimerBase>(),
          clock_type_(clock_type),
          period_(period),
          callback_function_(std::move(callback_function)),
          worker_(),
          node_mutex_(node_mutex),
          stop_flag_(false),
          is_canceled_(false)
    {
      start();
    }

    ~TimerBase() { stop(); }

    TimerBase(const TimerBase &) = delete;
    TimerBase &operator=(const TimerBase &) = delete;
    TimerBase(TimerBase &&) = delete;
    TimerBase &operator=(TimerBase &&) = delete;

    void start() override
    {
      // Join any previously running thread before starting a new one to avoid
      // std::terminate() when the joinable thread object is overwritten.
      if (worker_.joinable())
      {
        stop_flag_.store(true);
        worker_.join();
      }
      is_canceled_.store(false);
      stop_flag_.store(false);
      worker_ = std::thread([this]()
                            { run(); });
    }

    void stop() override
    {
      stop_flag_.store(true);
      stop_cv_.notify_all();
      if (worker_.joinable())
      {
        worker_.join(); // Wait for the worker thread to finish
      }
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
      stop();
      is_canceled_.store(false);
      stop_flag_.store(false);
      start();
    }

    // Get the timer period
    Duration get_period() const
    {
      return period_;
    }

    // Check if the timer is ready (not canceled and not stopped)
    bool is_ready() const
    {
      return !is_canceled_.load() && !stop_flag_.load();
    }

  private:
    void run_system_time()
    {
      auto next_execution_time = std::chrono::steady_clock::now() + std::chrono::nanoseconds(period_.nanoseconds());
      while (!stop_flag_.load())
      {
        {
          std::unique_lock<std::mutex> lk(stop_mutex_);
          stop_cv_.wait_until(lk, next_execution_time, [this] { return stop_flag_.load(); });
        }
        if (!stop_flag_.load())
        {
          std::lock_guard<std::mutex> lock(*node_mutex_);
          try { callback_function_(); }
          catch (const std::exception &e) {
            std::cerr << "Exception in timer callback: " << e.what() << std::endl;
          } catch (...) {
            std::cerr << "Unknown exception in timer callback." << std::endl;
          }
          next_execution_time += std::chrono::nanoseconds(period_.nanoseconds());
        }
      }
    }

    void run_steady_time()
    {
      auto next_execution_time = std::chrono::steady_clock::now() + std::chrono::nanoseconds(period_.nanoseconds());
      while (!stop_flag_.load())
      {
        {
          std::unique_lock<std::mutex> lk(stop_mutex_);
          stop_cv_.wait_until(lk, next_execution_time, [this] { return stop_flag_.load(); });
        }
        if (!stop_flag_.load())
        {
          std::lock_guard<std::mutex> lock(*node_mutex_);
          try { callback_function_(); }
          catch (const std::exception &e) {
            std::cerr << "Exception in timer callback: " << e.what() << std::endl;
          } catch (...) {
            std::cerr << "Unknown exception in timer callback." << std::endl;
          }
          next_execution_time += std::chrono::nanoseconds(period_.nanoseconds());
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
    std::function<void()> callback_function_;
    std::thread worker_;
    std::shared_ptr<std::mutex> node_mutex_;
    std::mutex stop_mutex_;
    std::condition_variable stop_cv_;
    std::atomic<bool> stop_flag_;
    std::atomic<bool> is_canceled_;
  };

} // namespace lwrcl

#endif // LWRCL_TIMER_HPP_
