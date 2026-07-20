/**
 * @file test_rate.cpp
 * @brief Test: Rate and WallRate sleep timing
 *
 * Covers: Rate::sleep, WallRate::sleep
 */
#include <gtest/gtest.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

TEST(RateTest, WallRateSleep) {
  rclcpp::WallRate rate(rclcpp::Duration(std::chrono::milliseconds(100)));

  auto start = std::chrono::steady_clock::now();
  rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - start;

  // Should sleep approximately 100ms (allow 50ms-300ms range)
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  EXPECT_GE(ms, 50);
  EXPECT_LE(ms, 300);
}

TEST(RateTest, RateSleep) {
  rclcpp::Rate rate(rclcpp::Duration(std::chrono::milliseconds(100)));

  auto start = std::chrono::steady_clock::now();
  rate.sleep();
  auto elapsed = std::chrono::steady_clock::now() - start;

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  EXPECT_GE(ms, 50);
  EXPECT_LE(ms, 300);
}

TEST(RateTest, WallRateHzConstructor) {
  rclcpp::WallRate rate(10.0);

  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(rate.sleep());
  auto elapsed = std::chrono::steady_clock::now() - start;

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  EXPECT_GE(ms, 50);
  EXPECT_LE(ms, 300);
  EXPECT_EQ(std::chrono::milliseconds(100), rate.period());
  EXPECT_EQ(rclcpp::Clock::ClockType::STEADY_TIME, rate.get_type());
}

TEST(RateTest, RateHzConstructor) {
  rclcpp::Rate rate(10.0);

  auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(rate.sleep());
  auto elapsed = std::chrono::steady_clock::now() - start;

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  EXPECT_GE(ms, 50);
  EXPECT_LE(ms, 300);
  EXPECT_EQ(std::chrono::milliseconds(100), rate.period());
  EXPECT_EQ(rclcpp::Clock::ClockType::SYSTEM_TIME, rate.get_type());
}

TEST(RateTest, RateClockConstructorAndReset) {
  auto clock = std::make_shared<rclcpp::Clock>(rclcpp::Clock::ClockType::STEADY_TIME);
  rclcpp::Rate rate(rclcpp::Duration(std::chrono::milliseconds(10)), clock);

  EXPECT_EQ(rclcpp::Clock::ClockType::STEADY_TIME, rate.get_type());
  EXPECT_EQ(std::chrono::milliseconds(10), rate.period());

  rate.reset();
  EXPECT_TRUE(rate.sleep());
}

TEST(RateTest, RejectInvalidRates) {
  EXPECT_THROW(rclcpp::Rate(0.0), std::invalid_argument);
  EXPECT_THROW(rclcpp::WallRate(-1.0), std::invalid_argument);
  EXPECT_THROW(rclcpp::Rate(rclcpp::Duration(std::chrono::nanoseconds(0))), std::invalid_argument);
}
