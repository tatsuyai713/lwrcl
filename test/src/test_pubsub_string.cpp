/**
 * @file test_pubsub_string.cpp
 * @brief Test: Publisher and Subscriber with std_msgs::String
 *
 * Covers: create_publisher, create_subscription, publish(shared_ptr),
 *         publish(const T&), subscriber callback, get_subscriber_count,
 *         get_publisher_count
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PubSubStringTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

TEST_F(PubSubStringTest, PublishAndReceiveSharedPtr) {
  auto node = rclcpp::Node::make_shared("test_pubsub_string");

  auto pub = node->create_publisher<std_msgs::msg::String>("test_topic_str", 10);
  ASSERT_TRUE(pub != nullptr);

  std::atomic<int> received{0};
  std::string last_msg;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "test_topic_str", 10,
      [&](std_msgs::msg::String::SharedPtr msg) {
        last_msg = msg->data();
        received++;
      });
  ASSERT_TRUE(sub != nullptr);

  // Allow discovery
  std::this_thread::sleep_for(1s);

  auto msg = std::make_shared<std_msgs::msg::String>();
  msg->data() = "hello_gtest";
  pub->publish(msg);

  // Let the message propagate
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
  EXPECT_EQ(last_msg, "hello_gtest");
}

TEST_F(PubSubStringTest, PublishConstRef) {
  auto node = rclcpp::Node::make_shared("test_pubsub_constref");

  auto pub = node->create_publisher<std_msgs::msg::String>("test_topic_constref", 10);

  std::atomic<int> received{0};
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "test_topic_constref", 10,
      [&](std_msgs::msg::String::SharedPtr) { received++; });

  std::this_thread::sleep_for(1s);

  std_msgs::msg::String msg;
  msg.data() = "const_ref_test";
  pub->publish(msg);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
}

TEST_F(PubSubStringTest, SubscriberAndPublisherCount) {
  auto node = rclcpp::Node::make_shared("test_counts");

  auto pub = node->create_publisher<std_msgs::msg::String>("test_topic_count", 10);
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "test_topic_count", 10,
      [](std_msgs::msg::String::SharedPtr) {});

  // Wait for discovery
  std::this_thread::sleep_for(2s);

  EXPECT_GE(pub->get_subscriber_count(), 1);
  EXPECT_GE(sub->get_publisher_count(), 1);
}
