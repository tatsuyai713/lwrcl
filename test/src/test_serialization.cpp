/**
 * @file test_serialization.cpp
 * @brief Test: SerializedMessage and Serialization<T> encode/decode
 *
 * Covers: SerializedMessage construction, copy, move, reserve, size,
 *         Serialization::serialize_message, deserialize_message
 */
#include <gtest/gtest.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SerializationTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(SerializationTest, DefaultConstruct) {
  lwrcl::SerializedMessage sm;
  EXPECT_EQ(sm.size(), 0u);
}

TEST_F(SerializationTest, ReserveAndCapacity) {
  lwrcl::SerializedMessage sm;
  sm.reserve(256);
  EXPECT_GE(sm.capacity(), 256u);
}

TEST_F(SerializationTest, CopyConstruct) {
  lwrcl::SerializedMessage sm(64);
  std::memset(sm.get_rcl_serialized_message().buffer, 0xAB, 64);

  lwrcl::SerializedMessage sm2(sm);
  EXPECT_EQ(sm2.size(), sm.size());
  EXPECT_EQ(std::memcmp(
      sm.get_rcl_serialized_message().buffer,
      sm2.get_rcl_serialized_message().buffer, sm.size()), 0);
}

TEST_F(SerializationTest, MoveConstruct) {
  lwrcl::SerializedMessage sm(128);
  size_t orig_size = sm.size();
  lwrcl::SerializedMessage sm2(std::move(sm));
  EXPECT_EQ(sm2.size(), orig_size);
  EXPECT_EQ(sm.size(), 0u); // moved-from
}

TEST_F(SerializationTest, SerializeDeserializeString) {
  std_msgs::msg::String original;
  original.data() = "serialize_me";

  lwrcl::SerializedMessage serialized;
  lwrcl::Serialization<std_msgs::msg::String>::serialize_message(&original, &serialized);
  EXPECT_GT(serialized.size(), 0u);

  std_msgs::msg::String deserialized;
  lwrcl::Serialization<std_msgs::msg::String>::deserialize_message(&serialized, &deserialized);
  EXPECT_EQ(deserialized.data(), "serialize_me");
}
