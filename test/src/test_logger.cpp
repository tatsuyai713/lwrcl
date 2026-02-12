/**
 * @file test_logger.cpp
 * @brief Test: Logger, RCLCPP_INFO / WARN / ERROR macros
 *
 * Covers: Logger construction, log calls, macro invocations (no crash)
 */
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

class LoggerTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(LoggerTest, LoggerFromNode) {
  auto node = rclcpp::Node::make_shared("log_test");
  auto logger = node->get_logger();

  // These should not crash
  RCLCPP_DEBUG(logger, "debug %d", 1);
  RCLCPP_INFO(logger, "info %s", "hello");
  RCLCPP_WARN(logger, "warn %.2f", 3.14);
  RCLCPP_ERROR(logger, "error %s %d", "code", 42);

  SUCCEED();
}

TEST_F(LoggerTest, FreeLogFunction) {
  // Free log function
  lwrcl::log(lwrcl::INFO, "Free log test %d", 123);
  SUCCEED();
}
