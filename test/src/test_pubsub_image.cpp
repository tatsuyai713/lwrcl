/**
 * @file test_pubsub_image.cpp
 * @brief Test: Pub/Sub with sensor_msgs::Image
 *
 * Covers: Large message (Image) publish+subscribe, data integrity check
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class PubSubImageTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PubSubImageTest, ImagePublishAndReceive) {
  auto node = rclcpp::Node::make_shared("test_image_pubsub");

  auto pub = node->create_publisher<sensor_msgs::msg::Image>("test_image_topic", 10);

  std::atomic<int> received{0};
  uint32_t rx_width = 0;
  uint32_t rx_height = 0;
  std::string rx_encoding;

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      "test_image_topic", 10,
      [&](sensor_msgs::msg::Image::SharedPtr msg) {
        rx_width = msg->width();
        rx_height = msg->height();
        rx_encoding = msg->encoding();
        received++;
      });

  std::this_thread::sleep_for(1s);

  // Publish an image
  auto img = std::make_shared<sensor_msgs::msg::Image>();
  img->header().stamp().sec() = 1;
  img->header().stamp().nanosec() = 0;
  img->header().frame_id() = "camera";
  img->height() = 480;
  img->width() = 640;
  img->encoding() = "rgb8";
  img->is_bigendian() = false;
  img->step() = 640 * 3;
  img->data().resize(640 * 480 * 3, 128);
  pub->publish(img);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
  EXPECT_EQ(rx_width, 640u);
  EXPECT_EQ(rx_height, 480u);
  EXPECT_EQ(rx_encoding, "rgb8");
}
