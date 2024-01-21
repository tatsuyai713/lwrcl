#include <functional>
#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <time.h>

namespace rcl_like_wrapper
{

class Timer
{
public:
    // コールバック関数が複数の引数を受け取るように変更
    template <typename Function, typename... Args>
    Timer(std::chrono::milliseconds period, Function&& callback_function, Args&&... args)
        : period_(period), stop_flag_(false)
    {
        // コールバック関数をstd::functionにラップし、引数を束縛
        callback_function_ = std::bind(std::forward<Function>(callback_function), std::forward<Args>(args)...);

        thread_ = std::thread(&Timer::runThread, this);
        std::cout << "Publisher running." << std::endl;
    }

    ~Timer()
    {
        stop_flag_ = true;
        thread_.join(); // スレッドの終了を待機
    }

    void runThread()
    {
        struct timespec curTime, lastTime;
        clock_gettime(CLOCK_REALTIME, &lastTime);

        auto start_time = std::chrono::steady_clock::now();
        uint64_t loop_count = 0;
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

            loop_count++;
            // Calculate next publication time
            auto now_time = std::chrono::steady_clock::now();
            auto next_time = start_time + period_ * loop_count;
            auto remaining_time = next_time - now_time;

            if (remaining_time > std::chrono::milliseconds(0))
            {
                std::this_thread::sleep_for(remaining_time);
            }
        }
    }

private:
    std::function<void()> callback_function_;
    std::thread thread_;
    bool stop_flag_;
    std::chrono::milliseconds period_;
};

} // namespace rcl_like_wrapper
