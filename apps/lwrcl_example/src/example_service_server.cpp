#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

using namespace std::chrono_literals;

class CameraInfoServer : public rclcpp::Node
{
public:
  CameraInfoServer() : Node("camera_info_server")
  {
    server_ = this->create_service<sensor_msgs::srv::SetCameraInfo>("camera_info_service",
      std::bind(&CameraInfoServer::service_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Create CameraInfo service server.");
  }

private:
  void service_callback
  (
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received request.");
  }

  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoServer>());
  rclcpp::shutdown();

  return 0;
}