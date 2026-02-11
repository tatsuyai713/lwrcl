#ifndef LWRCL_CLOCK_TIME_DURATION_HPP_
#define LWRCL_CLOCK_TIME_DURATION_HPP_

#include <chrono>
#include <cstdint>
#include <memory>

namespace lwrcl

{

  class Duration;

  class Time : public std::enable_shared_from_this<Time>
  {
  public:
    using SharedPtr = std::shared_ptr<Time>;
    Time();
    Time(int64_t nanoseconds);
    Time(int32_t seconds, uint32_t nanoseconds);

    Time(const Time &) = default;
    Time &operator=(const Time &) = default;
    Time(Time &&) = default;
    Time &operator=(Time &&) = default;

    ~Time() = default;

    int64_t nanoseconds() const;
    double seconds() const;

    Time operator+(const Duration &rhs) const;
    Time operator-(const Duration &rhs) const;
    Duration operator-(const Time &rhs) const;

    bool operator==(const Time &rhs) const;
    bool operator!=(const Time &rhs) const;
    bool operator<(const Time &rhs) const;
    bool operator<=(const Time &rhs) const;
    bool operator>(const Time &rhs) const;
    bool operator>=(const Time &rhs) const;

  private:
    int64_t nanoseconds_;
  };

  class Duration : public std::enable_shared_from_this<Duration>
  {
  public:
    using SharedPtr = std::shared_ptr<Duration>;

    Duration();
    Duration(int64_t nanoseconds);
    Duration(int32_t seconds, uint32_t nanoseconds);

    template <typename Rep, typename Period>
    Duration(const std::chrono::duration<Rep, Period> &duration)
        : nanoseconds_(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count())
    {
    }

    Duration(const Duration &) = default;
    Duration &operator=(const Duration &) = default;
    Duration(Duration &&) = default;
    Duration &operator=(Duration &&) = default;

    ~Duration() = default;

    int64_t nanoseconds() const;
    double seconds() const;

    Duration operator+(const Duration &rhs) const;
    Duration operator-(const Duration &rhs) const;

    bool operator==(const Duration &rhs) const;
    bool operator!=(const Duration &rhs) const;
    bool operator<(const Duration &rhs) const;
    bool operator<=(const Duration &rhs) const;
    bool operator>(const Duration &rhs) const;
    bool operator>=(const Duration &rhs) const;

  private:
    int64_t nanoseconds_;
  };

  class Clock : public std::enable_shared_from_this<Clock>
  {
  public:
    using SharedPtr = std::shared_ptr<Clock>;

    enum class ClockType
    {
      SYSTEM_TIME,
      STEADY_TIME
    };

    Clock(ClockType type = ClockType::SYSTEM_TIME);

    Clock(const Clock &) = default;
    Clock &operator=(const Clock &) = default;
    Clock(Clock &&) = default;
    Clock &operator=(Clock &&) = default;

    ~Clock() = default;

    Time now();
    ClockType get_clock_type() const;

  private:
    ClockType type_;
  };

} // namespace lwrcl

#endif // LWRCL_CLOCK_TIME_DURATION_HPP_