/**
 * @file test_namespace.cpp
 * @brief Test: Node namespace support
 *
 * Covers: Node with namespace, get_namespace, get_fully_qualified_name,
 *         topic name resolution (absolute vs relative)
 *
 * Design: Pub/Sub tests use a SINGLE node so that publisher and subscriber
 * share the same DomainParticipant.  This avoids cross-participant DDS
 * discovery latency that is unreliable in Docker / virtualised environments.
 * The namespace feature being tested here is *topic name resolution*, which
 * is fully exercised on a single node.
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NamespaceTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(NamespaceTest, NodeNameAndNamespace) {
  auto node = rclcpp::Node::make_shared("ns_node", "my_ns");
  EXPECT_EQ(node->get_name(), "ns_node");
  auto ns = node->get_namespace();
  EXPECT_TRUE(ns.find("my_ns") != std::string::npos);
  auto fqn = node->get_fully_qualified_name();
  EXPECT_FALSE(fqn.empty());
  EXPECT_NE(fqn.find("my_ns"), std::string::npos);
  EXPECT_NE(fqn.find("ns_node"), std::string::npos);
}

TEST_F(NamespaceTest, RelativeTopicNameResolved) {
  // Single node with namespace – pub and sub on relative topic "chatter"
  // should both resolve to "rt/test_ns/chatter" and communicate normally.
  auto node = rclcpp::Node::make_shared("ns_pubsub", "test_ns");

  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
  std::atomic<int> received{0};
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [&](std_msgs::msg::String::SharedPtr) { received++; });

  // Verify the resolved DDS topic name contains the namespace
  auto topic_name = pub->get_topic_name();
  EXPECT_NE(topic_name.find("test_ns"), std::string::npos);
  EXPECT_NE(topic_name.find("chatter"), std::string::npos);

  std::thread spin([&node]() { rclcpp::spin(node); });

  // Publish a few times
  auto msg = std_msgs::msg::String();
  msg.data() = "ns_test";
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(100ms);
  }

  node->stop_spin();
  spin.join();

  EXPECT_GE(received.load(), 1);
}

TEST_F(NamespaceTest, AbsoluteTopicBypassesNamespace) {
  // Single node with namespace – absolute topic "/global_topic" should
  // resolve to "rt/global_topic" (no namespace prefix).
  auto node = rclcpp::Node::make_shared("abs_pubsub", "ns_a");

  auto pub = node->create_publisher<std_msgs::msg::String>("/global_topic", 10);
  std::atomic<int> received{0};
  auto sub = node->create_subscription<std_msgs::msg::String>(
      "/global_topic", 10,
      [&](std_msgs::msg::String::SharedPtr) { received++; });

  // Verify the resolved DDS topic name does NOT contain the namespace
  auto topic_name = pub->get_topic_name();
  EXPECT_EQ(topic_name.find("ns_a"), std::string::npos);
  EXPECT_NE(topic_name.find("global_topic"), std::string::npos);

  std::thread spin([&node]() { rclcpp::spin(node); });

  auto msg = std_msgs::msg::String();
  msg.data() = "absolute_msg";
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    std::this_thread::sleep_for(100ms);
  }

  node->stop_spin();
  spin.join();

  EXPECT_GE(received.load(), 1);
}
