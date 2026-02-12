/**
 * @file test_multi_executor.cpp
 * @brief Test: MultiThreadedExecutor with multiple nodes
 *
 * Covers: MultiThreadedExecutor::add_node, spin, cancel
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MultiExecutorTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

class MEPubNode : public rclcpp::Node {
public:
  MEPubNode() : Node("me_pub"), count_(0) {
    pub_ = create_publisher<std_msgs::msg::String>("me_topic", 10);
    timer_ = create_wall_timer(100ms, [this]() {
      auto m = std_msgs::msg::String();
      m.data() = "multi_" + std::to_string(count_++);
      pub_->publish(m);
    });
  }
  std::atomic<int> count_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class MESubNode : public rclcpp::Node {
public:
  MESubNode() : Node("me_sub") {
    sub_ = create_subscription<std_msgs::msg::String>(
        "me_topic", 10,
        [this](std_msgs::msg::String::SharedPtr) { received_++; });
  }
  std::atomic<int> received_{0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

TEST_F(MultiExecutorTest, MultiThreadedPubSub) {
  auto pub_node = std::make_shared<MEPubNode>();
  auto sub_node = std::make_shared<MESubNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread exec_thread([&executor]() { executor.spin(); });

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (sub_node->received_.load() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(100ms);
  }

  executor.cancel();
  exec_thread.join();

  EXPECT_GE(sub_node->received_.load(), 3);
}
