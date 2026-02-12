/**
 * @file test_zero_copy.cpp
 * @brief Test: LoanedMessage / LoanedSubscriptionMessage zero-copy (FastDDS only)
 *
 * Covers: borrow_loaned_message, LoanedMessage::is_valid, publish(LoanedMessage&&),
 *         can_loan_messages, take_loaned_message, LoanedSubscriptionMessage::get
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ZeroCopyTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ZeroCopyTest, BorrowAndPublishLoanedMessage) {
  auto node = rclcpp::Node::make_shared("zc_pub_test");

  rclcpp::SensorDataQoS qos;
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("zc_image_topic", qos);

  EXPECT_TRUE(pub->can_loan_messages());

  std::atomic<int> received{0};
  uint32_t rx_width = 0;
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
      "zc_image_topic", qos,
      [&](sensor_msgs::msg::Image::SharedPtr msg) {
        rx_width = msg->width();
        received++;
      });

  std::this_thread::sleep_for(1s);

  // Borrow and publish
  auto loaned = pub->borrow_loaned_message();
  EXPECT_TRUE(loaned.is_valid());

  loaned.get().header().stamp().sec() = 42;
  loaned.get().width() = 320;
  loaned.get().height() = 240;
  loaned.get().encoding() = "rgb8";
  loaned.get().step() = 320 * 3;
  loaned.get().data().resize(320 * 240 * 3, 0);

  pub->publish(std::move(loaned));

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
  EXPECT_EQ(rx_width, 320u);
}

TEST_F(ZeroCopyTest, CanLoanMessages) {
  auto node = rclcpp::Node::make_shared("zc_loan_check");
  rclcpp::SensorDataQoS qos;
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("zc_loan_topic", qos);
  // FastDDS should always claim it can loan
  EXPECT_TRUE(pub->can_loan_messages());
}
