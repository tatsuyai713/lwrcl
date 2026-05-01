// sensor_bundle_subscriber.cpp
//
// Subscribes to ros_msg_mixed_example::msg::SensorBundle and reads every
// nested field through ROS 2-style direct member access.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "ros_msg_mixed_example/msg/sensor_bundle.hpp"

class SensorBundleSubscriber : public rclcpp::Node
{
public:
  SensorBundleSubscriber()
  : Node("sensor_bundle_subscriber")
  {
    subscription_ = create_subscription<ros_msg_mixed_example::msg::SensorBundle>(
      "sensor_bundle", rclcpp::QoS(10),
      [this](std::shared_ptr<ros_msg_mixed_example::msg::SensorBundle> msg) {
        on_msg(*msg);
      });
  }

private:
  void on_msg(const ros_msg_mixed_example::msg::SensorBundle & msg)
  {
    RCLCPP_INFO(get_logger(),
      "received frame=%s pose=(%.2f,%.2f,%.2f) yaw=%.3f waypoints=%zu desc=%s status=%u",
      msg.header.frame_id.c_str(),
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z,
      msg.pose.yaw,
      msg.waypoints.size(),
      msg.description.c_str(),
      static_cast<unsigned>(msg.status));
  }

  rclcpp::Subscription<ros_msg_mixed_example::msg::SensorBundle>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBundleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
