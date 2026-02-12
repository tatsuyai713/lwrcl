/**
 * @file test_service.cpp
 * @brief Test: Service Server + Client request/response
 *
 * Covers: create_service, create_client, async_send_request,
 *         wait_for_service, spin_until_future_complete
 */
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

using namespace std::chrono_literals;

class ServiceTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ServiceTest, ServiceRequestResponse) {
  // Server node
  auto server_node = rclcpp::Node::make_shared("test_server");
  bool request_received = false;
  auto server = server_node->create_service<sensor_msgs::srv::SetCameraInfo>(
      "test_camera_service",
      [&](const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request>,
          std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response) {
        request_received = true;
        response->success() = true;
        response->status_message() = "OK";
      });

  std::thread server_thread([&server_node]() { rclcpp::spin(server_node); });

  // Client node
  auto client_node = rclcpp::Node::make_shared("test_client");
  auto client = client_node->create_client<sensor_msgs::srv::SetCameraInfo>("test_camera_service");

  bool service_found = client->wait_for_service(5s);
  ASSERT_TRUE(service_found) << "Service server not discovered within timeout";

  auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
  auto future = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(client_node, future, 10s);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  server_node->stop_spin();
  server_thread.join();

  EXPECT_TRUE(request_received);
}
