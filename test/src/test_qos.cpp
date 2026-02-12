/**
 * @file test_qos.cpp
 * @brief Test: QoS class builder methods and RMW profile conversion
 *
 * Covers: QoS depth, keep_last, keep_all, reliability, durability,
 *         to_rmw_qos_profile, KeepLast, KeepAll
 */
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "qos.hpp"  // direct include for QoS (CycloneDDS rclcpp.hpp may not alias it)

using QoS = lwrcl::QoS;

class QoSTest : public ::testing::Test {};

TEST_F(QoSTest, DefaultDepth) {
  QoS qos(10);
  EXPECT_EQ(qos.get_depth(), 10u);
  EXPECT_TRUE(qos.get_history() == QoS::HistoryPolicy::KEEP_LAST);
  EXPECT_TRUE(qos.get_reliability() == QoS::ReliabilityPolicy::RELIABLE);
  EXPECT_TRUE(qos.get_durability() == QoS::DurabilityPolicy::VOLATILE);
}

TEST_F(QoSTest, KeepAll) {
  QoS qos(10);
  qos.keep_all();
  EXPECT_TRUE(qos.get_history() == QoS::HistoryPolicy::KEEP_ALL);
}

TEST_F(QoSTest, ChainedSetters) {
  QoS qos(5);
  qos.reliability(QoS::ReliabilityPolicy::BEST_EFFORT)
     .durability(QoS::DurabilityPolicy::TRANSIENT_LOCAL)
     .keep_last(20);

  EXPECT_EQ(qos.get_depth(), 20u);
  EXPECT_TRUE(qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT);
  EXPECT_TRUE(qos.get_durability() == QoS::DurabilityPolicy::TRANSIENT_LOCAL);
}

TEST_F(QoSTest, ToRMWProfile) {
  QoS qos(10);
  qos.reliability(QoS::ReliabilityPolicy::BEST_EFFORT);
  auto profile = qos.to_rmw_qos_profile();
  EXPECT_EQ(profile.depth, 10u);
  EXPECT_TRUE(profile.reliability == RMWQoSReliabilityPolicy::BEST_EFFORT);
}

TEST_F(QoSTest, KeepLastInitialization) {
  auto init = lwrcl::KeepLast(42);
  EXPECT_EQ(init.depth_, 42u);
  EXPECT_TRUE(init.history_ == RMWQoSHistoryPolicy::KEEP_LAST);
}

TEST_F(QoSTest, KeepAllInitialization) {
  auto init = lwrcl::KeepAll();
  EXPECT_TRUE(init.history_ == RMWQoSHistoryPolicy::KEEP_ALL);
}
