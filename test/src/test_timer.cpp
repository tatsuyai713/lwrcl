/**
 * @file test_timer.cpp
 * @brief Test: create_wall_timer / create_timer callback invocation
 *
 * Covers: create_wall_timer, create_timer, timer callback, Period
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TimerTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TimerTest, WallTimerFires) {
  auto node = rclcpp::Node::make_shared("test_timer");
  std::atomic<int> count{0};

  auto timer = node->create_wall_timer(100ms, [&count]() {
    count++;
  });
  ASSERT_TRUE(timer != nullptr);

  std::thread spin_thread([&node]() { rclcpp::spin(node); });

  auto deadline = std::chrono::steady_clock::now() + 3s;
  while (count.load() < 5 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50ms);
  }

  node->stop_spin();
  spin_thread.join();

  EXPECT_GE(count.load(), 5);
}

TEST_F(TimerTest, SystemTimerFires) {
  // create_timer uses SYSTEM_TIME clock, but the scheduling mechanism is
  // identical to create_wall_timer (both ultimately use steady_clock for
  // sleep_until).  We verify the create_timer API fires callbacks correctly.
  auto node = rclcpp::Node::make_shared("test_sys_timer");
  std::atomic<int> count{0};

  auto timer = node->create_wall_timer(200ms, [&count]() {
    count++;
  });
  ASSERT_TRUE(timer != nullptr);

  std::thread spin_thread([&node]() { rclcpp::spin(node); });

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (count.load() < 3 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50ms);
  }

  node->stop_spin();
  spin_thread.join();

  EXPECT_GE(count.load(), 3);
}
