/**
 * @file test_node.cpp
 * @brief Test: Node creation, name, shared pointer, make_shared overloads
 *
 * Covers: Node::make_shared, get_name, get_logger, get_clock, shutdown
 */
#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class NodeTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(NodeTest, MakeSharedWithName) {
  auto node = rclcpp::Node::make_shared("my_node");
  ASSERT_TRUE(node != nullptr);
  EXPECT_EQ(node->get_name(), "my_node");
}

TEST_F(NodeTest, GetClock) {
  auto node = rclcpp::Node::make_shared("clock_node");
  auto clock = node->get_clock();
  ASSERT_TRUE(clock != nullptr);

  auto now = clock->now();
  // Time should be non-zero (wall-clock)
  EXPECT_GT(now.nanoseconds(), 0);
}

TEST_F(NodeTest, GetLogger) {
  auto node = rclcpp::Node::make_shared("logger_node");
  // Should not throw
  auto logger = node->get_logger();
  // Test log call - just make sure it doesn't crash
  RCLCPP_INFO(logger, "gtest info log from node '%s'", node->get_name().c_str());
}

TEST_F(NodeTest, NodeShutdown) {
  auto node = rclcpp::Node::make_shared("shutdown_node");
  node->shutdown();
  EXPECT_TRUE(node->closed_);
}
