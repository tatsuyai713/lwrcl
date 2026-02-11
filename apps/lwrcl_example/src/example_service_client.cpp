#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

using namespace std::chrono_literals;

class CameraInfoClient : public rclcpp::Node
{
public:
  CameraInfoClient() : Node("camera_info_client")
  {
    client_ = this->create_client<sensor_msgs::srv::SetCameraInfo>("camera_info_service");
  }

  void process()
  {
    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service is currently not available... waiting.");
    }

    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();

    auto result = client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "Send request.");

    auto return_code = rclcpp::spin_until_future_complete(this->shared_from_this(), result, 10s);

    if (return_code == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Success to call service.");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
    }
  }

private:
  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<CameraInfoClient>();
  client->process();
  rclcpp::shutdown();

  return 0;
}