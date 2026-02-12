/**
 * @file test_spin_some.cpp
 * @brief Test: spin_some processes callbacks without blocking
 *
 * Covers: rclcpp::spin_some, non-blocking callback dispatch
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SpinSomeTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(SpinSomeTest, SpinSomeProcessesMessages) {
  auto node = rclcpp::Node::make_shared("test_spin_some");

  auto pub = node->create_publisher<std_msgs::msg::String>("spin_some_topic", 10);
  std::atomic<int> received{0};
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "spin_some_topic", 10,
      [&](std_msgs::msg::String::SharedPtr) { received++; });

  std::this_thread::sleep_for(1s);

  // Publish
  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data() = "spin_some_msg";
  pub->publish(msg);

  // Use spin_some in a polling loop
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
}
