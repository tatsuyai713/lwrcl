/**
 * @file test_qos_presets.cpp
 * @brief Test: QoS preset profiles
 *
 * Covers: SensorDataQoS, SystemDefaultsQoS, ServicesQoS, ParametersQoS,
 *         ParameterEventsQoS, BestEffortQoS, ReliableQoS,
 *         deadline/lifespan/liveliness_lease_duration setters
 */
#include <gtest/gtest.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

TEST(QoSPresetsTest, SensorDataQoS) {
  rclcpp::SensorDataQoS qos;
  EXPECT_EQ(qos.get_depth(), 5u);
  EXPECT_TRUE(qos.get_reliability() == rclcpp::QoS::ReliabilityPolicy::BEST_EFFORT);
  EXPECT_TRUE(qos.get_durability() == rclcpp::QoS::DurabilityPolicy::VOLATILE);
}

TEST(QoSPresetsTest, SystemDefaultsQoS) {
  rclcpp::SystemDefaultsQoS qos;
  EXPECT_EQ(qos.get_depth(), 10u);
  EXPECT_TRUE(qos.get_reliability() == rclcpp::QoS::ReliabilityPolicy::RELIABLE);
}

TEST(QoSPresetsTest, ServicesQoS) {
  rclcpp::ServicesQoS qos;
  EXPECT_TRUE(qos.get_reliability() == rclcpp::QoS::ReliabilityPolicy::RELIABLE);
}

TEST(QoSPresetsTest, ParametersQoS) {
  rclcpp::ParametersQoS qos;
  EXPECT_EQ(qos.get_depth(), 1000u);
}

TEST(QoSPresetsTest, BestEffortQoS) {
  rclcpp::BestEffortQoS qos;
  EXPECT_TRUE(qos.get_reliability() == rclcpp::QoS::ReliabilityPolicy::BEST_EFFORT);
}

TEST(QoSPresetsTest, ReliableQoS) {
  rclcpp::ReliableQoS qos;
  EXPECT_TRUE(qos.get_reliability() == rclcpp::QoS::ReliabilityPolicy::RELIABLE);
}

TEST(QoSPresetsTest, DeadlineSetter) {
  rclcpp::QoS qos(10);
  qos.deadline(100ms);
  auto d = qos.get_deadline();
  EXPECT_FALSE(d.is_infinite());
  EXPECT_EQ(d.sec, 0);
  EXPECT_EQ(d.nsec, 100000000u);
}

TEST(QoSPresetsTest, LifespanSetter) {
  rclcpp::QoS qos(10);
  qos.lifespan(std::chrono::seconds(2));
  auto l = qos.get_lifespan();
  EXPECT_FALSE(l.is_infinite());
  EXPECT_EQ(l.sec, 2);
}

TEST(QoSPresetsTest, LivelinessLeaseDurationSetter) {
  rclcpp::QoS qos(10);
  qos.liveliness_lease_duration(500ms);
  auto ll = qos.get_liveliness_lease_duration();
  EXPECT_FALSE(ll.is_infinite());
  EXPECT_EQ(ll.sec, 0);
  EXPECT_EQ(ll.nsec, 500000000u);
}

TEST(QoSPresetsTest, LivelinessPolicy) {
  rclcpp::QoS qos(10);
  qos.liveliness(rclcpp::QoS::LivelinessPolicy::MANUAL_BY_TOPIC);
  EXPECT_TRUE(qos.get_liveliness() == rclcpp::QoS::LivelinessPolicy::MANUAL_BY_TOPIC);
}
