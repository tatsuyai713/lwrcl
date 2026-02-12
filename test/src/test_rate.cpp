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
