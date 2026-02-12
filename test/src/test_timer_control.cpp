/**
 * @file test_timer_control.cpp
 * @brief Test: Timer cancel / reset / is_canceled / is_ready / get_period
 *
 * Covers: TimerBase::cancel, reset, is_canceled, is_ready, get_period
 */
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TimerControlTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TimerControlTest, CancelStopsTimer) {
  auto node = rclcpp::Node::make_shared("tc_cancel");
  std::atomic<int> count{0};

  auto timer = node->create_wall_timer(50ms, [&count]() { count++; });

  std::thread spin([&node]() { rclcpp::spin(node); });

  // Let it fire a few times
  std::this_thread::sleep_for(500ms);
  int before_cancel = count.load();
  EXPECT_GE(before_cancel, 2);

  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());

  std::this_thread::sleep_for(500ms);
  int after_cancel = count.load();

  // Should not have advanced much (maybe +1 due to race)
  EXPECT_LE(after_cancel - before_cancel, 2);

  node->stop_spin();
  spin.join();
}

TEST_F(TimerControlTest, ResetRestartsTimer) {
  auto node = rclcpp::Node::make_shared("tc_reset");
  std::atomic<int> count{0};

  auto timer = node->create_wall_timer(50ms, [&count]() { count++; });

  std::thread spin([&node]() { rclcpp::spin(node); });

  // Use sleep_for (same pattern as CancelStopsTimer which passes reliably)
  std::this_thread::sleep_for(1s);
  timer->cancel();
  EXPECT_TRUE(timer->is_canceled());
  int after_cancel = count.load();
  EXPECT_GE(after_cancel, 2);

  // Reset and verify timer fires again
  timer->reset();
  EXPECT_FALSE(timer->is_canceled());

  std::this_thread::sleep_for(1s);
  EXPECT_GT(count.load(), after_cancel);

  node->stop_spin();
  spin.join();
}

TEST_F(TimerControlTest, GetPeriod) {
  auto node = rclcpp::Node::make_shared("tc_period");
  auto timer = node->create_wall_timer(100ms, []() {});

  auto period = timer->get_period();
  EXPECT_EQ(period.nanoseconds(), 100000000LL); // 100ms = 100000000ns
}

TEST_F(TimerControlTest, IsReady) {
  auto node = rclcpp::Node::make_shared("tc_ready");
  auto timer = node->create_wall_timer(100ms, []() {});

  EXPECT_TRUE(timer->is_ready());
  timer->cancel();
  EXPECT_FALSE(timer->is_ready());
}
