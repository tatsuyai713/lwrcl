#include <functional>
#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <time.h>
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
      thread_ = std::thread(&Timer::runThread, this);
      std::cout << "Publisher running." << std::endl;
    }

    ~Timer()
    {
      stop_flag_ = true;
      thread_.join();
    }

    void runThread()
    {
      struct timespec curTime, lastTime;
      clock_gettime(CLOCK_REALTIME, &lastTime);

      auto start_time = std::chrono::steady_clock::now();
      
      while (!stop_flag_)
      {
        clock_gettime(CLOCK_REALTIME, &curTime);
        if (curTime.tv_nsec < lastTime.tv_nsec)
        {
          printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec - 1, curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
        }
        else
        {
          printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec, curTime.tv_nsec - lastTime.tv_nsec);
        }
        lastTime = curTime;

        std::cout << "Message: SENT" << std::endl;
        callback_function_();

        // Calculate next publication time
        start_time += std::chrono::milliseconds(100);

        // Sleep until the next publication time
        std::this_thread::sleep_until(start_time);
      }
    }

  private:
    CallbackFunction callback_function_;
    std::thread thread_;
    bool stop_flag_;
    std::chrono::milliseconds period_;
  };

} // namespace rcl_like_wrapper
