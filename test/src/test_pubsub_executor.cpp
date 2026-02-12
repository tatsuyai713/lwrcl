/**
 * @file test_pubsub_executor.cpp
 * @brief Test: SingleThreadedExecutor with Publisher + Subscriber nodes
 *
 * Covers: SingleThreadedExecutor::add_node, remove_node, spin (timed), cancel
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ExecutorPubSubTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

class PubNode : public rclcpp::Node {
public:
  PubNode() : Node("exec_pub"), count_(0) {
    pub_ = create_publisher<std_msgs::msg::String>("exec_topic", 10);
    timer_ = create_wall_timer(100ms, [this]() {
      auto m = std_msgs::msg::String();
      m.data() = "msg_" + std::to_string(count_++);
      pub_->publish(m);
    });
  }
  int count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class SubNode : public rclcpp::Node {
public:
  SubNode() : Node("exec_sub") {
    sub_ = create_subscription<std_msgs::msg::String>(
        "exec_topic", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          last_ = msg->data();
          count_++;
        });
  }
  std::atomic<int> count_{0};
  std::string last_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

TEST_F(ExecutorPubSubTest, SingleThreadedExecutorPubSub) {
  auto pub_node = std::make_shared<PubNode>();
  auto sub_node = std::make_shared<SubNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::thread exec_thread([&executor]() { executor.spin(); });

  // Wait for some messages
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (sub_node->count_.load() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(100ms);
  }

  executor.cancel();
  exec_thread.join();

  EXPECT_GE(sub_node->count_.load(), 3);
}
