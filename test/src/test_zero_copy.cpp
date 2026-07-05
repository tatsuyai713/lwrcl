/**
 * @file test_zero_copy.cpp
 * @brief Test transport fast path with normal ROS-compatible publish API.
 *
 * Covers: normal Publisher::publish(const T&) and subscription callbacks.
 *
 * Supported backends: FastDDS, CycloneDDS, vsomeip
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class TransportFastPathTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TransportFastPathTest, NormalPublishReceivesImage) {
  auto node = rclcpp::Node::make_shared("transport_fast_path_test");

  rclcpp::SensorDataQoS qos;

  std::atomic<int> received{0};
  uint32_t rx_width = 0;
  uint8_t rx_first_byte = 0;

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      "transport_fast_path_image", qos,
      [&](sensor_msgs::msg::Image::SharedPtr msg) {
        rx_width = msg->width;
        if (!msg->data.empty()) {
          rx_first_byte = msg->data[0];
        }
        received++;
      });

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(
      "transport_fast_path_image", qos);

  auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((pub->get_subscription_count() == 0 || sub->get_publisher_count() == 0) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_GT(pub->get_subscription_count(), 0);
  ASSERT_GT(sub->get_publisher_count(), 0);

  sensor_msgs::msg::Image msg;
  msg.header.stamp.sec = 42;
  msg.width = 320;
  msg.height = 240;
  msg.encoding = "rgb8";
  msg.step = 320 * 3;
  msg.data.resize(320 * 240 * 3, 0);
  msg.data[0] = 0x24;

  pub->publish(msg);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
  EXPECT_EQ(rx_width, 320u);
  EXPECT_EQ(rx_first_byte, 0x24u);
}
