/**
 * @file test_parameter.cpp
 * @brief Test: Parameter declare/get for all supported types
 *
 * Covers: declare_parameter(bool/int/double/string/vectors),
 *         get_parameter, Parameter::as_* methods
 */
#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class ParameterTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ParameterTest, DeclareAndGetBool) {
  auto node = rclcpp::Node::make_shared("param_bool");
  node->declare_parameter("flag", true);
  bool val;
  node->get_parameter("flag", val);
  EXPECT_TRUE(val);
}

TEST_F(ParameterTest, DeclareAndGetInt) {
  auto node = rclcpp::Node::make_shared("param_int");
  node->declare_parameter("count", 42);
  int val;
  node->get_parameter("count", val);
  EXPECT_EQ(val, 42);
}

TEST_F(ParameterTest, DeclareAndGetDouble) {
  auto node = rclcpp::Node::make_shared("param_double");
  node->declare_parameter("ratio", 3.14);
  double val;
  node->get_parameter("ratio", val);
  EXPECT_DOUBLE_EQ(val, 3.14);
}

TEST_F(ParameterTest, DeclareAndGetString) {
  auto node = rclcpp::Node::make_shared("param_str");
  node->declare_parameter("name", std::string("test_value"));
  std::string val;
  node->get_parameter("name", val);
  EXPECT_EQ(val, "test_value");
}

TEST_F(ParameterTest, ParameterObjectTypes) {
  rclcpp::Parameter p_bool("b", true);
  EXPECT_EQ(p_bool.get_name(), "b");
  EXPECT_TRUE(p_bool.as_bool());

  rclcpp::Parameter p_int("i", 99);
  EXPECT_EQ(p_int.as_int(), 99);

  rclcpp::Parameter p_dbl("d", 2.718);
  EXPECT_DOUBLE_EQ(p_dbl.as_double(), 2.718);

  rclcpp::Parameter p_str("s", std::string("hello"));
  EXPECT_EQ(p_str.as_string(), "hello");
}

TEST_F(ParameterTest, DeclareVectorInt) {
  auto node = rclcpp::Node::make_shared("param_vec_int");
  std::vector<int> v = {1, 2, 3};
  node->declare_parameter("vec_int", v);
  auto p = node->get_parameter("vec_int");
  auto result = p.as_integer_array();
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0], 1);
  EXPECT_EQ(result[2], 3);
}

TEST_F(ParameterTest, DeclareVectorDouble) {
  auto node = rclcpp::Node::make_shared("param_vec_dbl");
  std::vector<double> v = {1.1, 2.2};
  node->declare_parameter("vec_dbl", v);
  auto p = node->get_parameter("vec_dbl");
  auto result = p.as_double_array();
  ASSERT_EQ(result.size(), 2u);
  EXPECT_DOUBLE_EQ(result[0], 1.1);
}

TEST_F(ParameterTest, DeclareVectorString) {
  auto node = rclcpp::Node::make_shared("param_vec_str");
  std::vector<std::string> v = {"a", "b", "c"};
  node->declare_parameter("vec_str", v);
  auto p = node->get_parameter("vec_str");
  auto result = p.as_string_array();
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[1], "b");
}
