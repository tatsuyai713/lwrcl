// sensor_bundle_publisher.cpp
//
// Publishes ros_msg_mixed_example::msg::SensorBundle. The .msg files in
// this package mix three reference styles:
//   1. ROS 2 primitives (e.g. float32, string).
//   2. Sibling .msg types in the same package (Pose, Vector3).
//   3. External IDL types from ros-data-types
//      (std_msgs/Header, geometry_msgs/Point, builtin_interfaces/Time).
//
// All three are written in plain ROS 2 ``pkg/Type`` form inside the .msg;
// msg_to_idl.py converts them into the appropriate IDL include lines.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "ros_msg_mixed_example/msg/sensor_bundle.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class SensorBundlePublisher : public rclcpp::Node
{
public:
  SensorBundlePublisher()
  : Node("sensor_bundle_publisher")
  {
    publisher_ = create_publisher<ros_msg_mixed_example::msg::SensorBundle>(
      "sensor_bundle", rclcpp::QoS(10));
    timer_ = create_wall_timer(
      500ms, std::bind(&SensorBundlePublisher::tick, this));
  }

private:
  void tick()
  {
    auto msg = std::make_shared<ros_msg_mixed_example::msg::SensorBundle>();

    // External IDL type used through plain member access.
    msg->header.stamp.sec = static_cast<int32_t>(now_sec());
    msg->header.stamp.nanosec = 0;
    msg->header.frame_id = "base_link";

    // Sibling .msg type with nested external IDL field.
    msg->pose.stamp.sec = msg->header.stamp.sec;
    msg->pose.stamp.nanosec = 0;
    msg->pose.position.x = 0.1 * count_;
    msg->pose.position.y = 0.2 * count_;
    msg->pose.position.z = 0.0;
    msg->pose.yaw = 0.05 * count_;

    msg->waypoints.clear();
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(i);
      p.y = msg->pose.position.y;
      p.z = 0.0;
      msg->waypoints.push_back(p);
    }

    msg->orientation_xyz[0] = 0.0f;
    msg->orientation_xyz[1] = 0.0f;
    msg->orientation_xyz[2] = static_cast<float>(msg->pose.yaw);
    msg->description = "tick-" + std::to_string(count_);
    msg->status = ros_msg_mixed_example::msg::SensorBundle__STATUS_OK;

    RCLCPP_INFO(get_logger(),
      "publish frame=%s pose=(%.2f,%.2f,%.2f) waypoints=%zu",
      msg->header.frame_id.c_str(),
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z,
      msg->waypoints.size());
    publisher_->publish(msg);
    ++count_;
  }

  static long now_sec()
  {
    using namespace std::chrono;
    return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
  }

  rclcpp::Publisher<ros_msg_mixed_example::msg::SensorBundle>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBundlePublisher>());
  rclcpp::shutdown();
  return 0;
}
