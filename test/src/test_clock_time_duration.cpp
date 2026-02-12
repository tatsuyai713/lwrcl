/**
 * @file test_clock_time_duration.cpp
 * @brief Test: Clock, Time, Duration arithmetic and comparison
 *
 * Covers: Clock::now, Time arithmetic (+/-), Duration arithmetic,
 *         comparison operators, seconds(), nanoseconds()
 */
#include <gtest/gtest.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

TEST(ClockTest, SystemClockNow) {
  lwrcl::Clock clock(lwrcl::Clock::ClockType::SYSTEM_TIME);
  auto now = clock.now();
  EXPECT_GT(now.nanoseconds(), 0);
  EXPECT_GT(now.seconds(), 0.0);
}

TEST(ClockTest, SteadyClockNow) {
  lwrcl::Clock clock(lwrcl::Clock::ClockType::STEADY_TIME);
  auto now = clock.now();
  EXPECT_GT(now.nanoseconds(), 0);
}

TEST(TimeTest, ConstructFromSeconds) {
  lwrcl::Time t(5, 500000000u); // 5.5 s
  EXPECT_EQ(t.nanoseconds(), 5500000000LL);
  EXPECT_DOUBLE_EQ(t.seconds(), 5.5);
}

TEST(TimeTest, ComparisonOperators) {
  lwrcl::Time t1(1000000000LL); // 1 s
  lwrcl::Time t2(2000000000LL); // 2 s
  EXPECT_TRUE(t1 < t2);
  EXPECT_TRUE(t2 > t1);
  EXPECT_TRUE(t1 != t2);
  EXPECT_TRUE(t1 <= t2);
  EXPECT_TRUE(t2 >= t1);
  EXPECT_TRUE(t1 == lwrcl::Time(1000000000LL));
}

TEST(DurationTest, ConstructFromNanoseconds) {
  lwrcl::Duration d(1000000000LL);
  EXPECT_EQ(d.nanoseconds(), 1000000000LL);
  EXPECT_DOUBLE_EQ(d.seconds(), 1.0);
}

TEST(DurationTest, ConstructFromChrono) {
  lwrcl::Duration d(std::chrono::milliseconds(500));
  EXPECT_EQ(d.nanoseconds(), 500000000LL);
}

TEST(DurationTest, ArithmeticOperators) {
  lwrcl::Duration d1(1000000000LL);
  lwrcl::Duration d2(500000000LL);
  auto sum = d1 + d2;
  EXPECT_EQ(sum.nanoseconds(), 1500000000LL);
  auto diff = d1 - d2;
  EXPECT_EQ(diff.nanoseconds(), 500000000LL);
}

TEST(DurationTest, ComparisonOperators) {
  lwrcl::Duration d1(100);
  lwrcl::Duration d2(200);
  EXPECT_TRUE(d1 < d2);
  EXPECT_TRUE(d1 != d2);
  EXPECT_TRUE(d1 <= d2);
}

TEST(TimeTest, AddDuration) {
  lwrcl::Time t(1000000000LL);
  lwrcl::Duration d(500000000LL);
  auto result = t + d;
  EXPECT_EQ(result.nanoseconds(), 1500000000LL);
}

TEST(TimeTest, SubtractDuration) {
  lwrcl::Time t(2000000000LL);
  lwrcl::Duration d(500000000LL);
  auto result = t - d;
  EXPECT_EQ(result.nanoseconds(), 1500000000LL);
}

TEST(TimeTest, DifferenceTwoTimes) {
  lwrcl::Time t1(2000000000LL);
  lwrcl::Time t2(500000000LL);
  auto diff = t1 - t2;
  EXPECT_EQ(diff.nanoseconds(), 1500000000LL);
}
