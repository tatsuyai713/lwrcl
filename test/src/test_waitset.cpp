/**
 * @file test_waitset.cpp
 * @brief Test: WaitSet with subscription polling
 *
 * Covers: WaitSet, WaitResult, WaitResultKind, wait_for, add_subscription,
 *         Subscription::take, Subscription::has_message
 */
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class WaitSetTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(WaitSetTest, WaitSetReady) {
  auto node = rclcpp::Node::make_shared("test_waitset");

  auto pub = node->create_publisher<std_msgs::msg::String>("waitset_topic", 10);
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "waitset_topic", 10,
      [](std_msgs::msg::String::SharedPtr) {});

  rclcpp::WaitSet ws;
  ws.add_subscription(sub);

  // Wait for discovery
  std::this_thread::sleep_for(1s);

  // Publish
  auto msg = std_msgs::msg::String();
  msg.data() = "waitset_data";
  pub->publish(msg);

  // Wait on the WaitSet
  auto result = ws.wait(3s);
  EXPECT_TRUE(result.kind() == rclcpp::WaitResultKind::Ready);

  // Take the message
  std_msgs::msg::String out;
  rclcpp::MessageInfo info;
  bool taken = sub->take(out, info);
  EXPECT_TRUE(taken);
  EXPECT_EQ(out.data(), "waitset_data");
}

TEST_F(WaitSetTest, WaitSetTimeout) {
  auto node = rclcpp::Node::make_shared("test_waitset_to");

  auto sub = node->create_subscription<std_msgs::msg::String>(
      "waitset_empty_topic", 10,
      [](std_msgs::msg::String::SharedPtr) {});

  rclcpp::WaitSet ws;
  ws.add_subscription(sub);

  auto result = ws.wait(500ms);
  EXPECT_TRUE(result.kind() == rclcpp::WaitResultKind::Timeout);
}
