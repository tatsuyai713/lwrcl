// sample_data_publisher.cpp
//
// Publishes ros_msg_example::msg::SampleData on a configurable topic.
// The C++ struct is generated from msg/SampleData.msg via the workspace's
// auto .msg -> .idl -> C++ pipeline. After the post-processing step
// (expose_idl_members.py) all members are ordinary public fields, so they
// can be assigned directly using ROS 2-style syntax (no accessor methods).

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

// Generated from msg/SampleData.msg
#include "ros_msg_example/msg/sample_data.hpp"

using namespace std::chrono_literals;

class SampleDataPublisher : public rclcpp::Node
{
public:
  SampleDataPublisher()
  : Node("sample_data_publisher")
  {
    publisher_ =
      create_publisher<ros_msg_example::msg::SampleData>("sample_data", rclcpp::QoS(10));

    timer_ = create_wall_timer(
      500ms, std::bind(&SampleDataPublisher::tick, this));
  }

private:
  void tick()
  {
    auto msg = std::make_shared<ros_msg_example::msg::SampleData>();

    // ROS 2-style direct member assignment (made possible by the patch step
    // that strips DDS accessor methods from the generated headers).
    msg->header.stamp.sec = static_cast<int32_t>(now_sec());
    msg->header.stamp.nanosec = 0;
    msg->header.frame_id = "sensor_link";

    msg->sequence_number = ++count_;
    msg->value = 0.5 * count_;
    msg->label = "sample-" + std::to_string(count_);
    msg->status = ros_msg_example::msg::SampleData__STATUS_OK;

    RCLCPP_INFO(get_logger(),
      "publishing seq=%d value=%.2f label=%s",
      msg->sequence_number, msg->value, msg->label.c_str());
    publisher_->publish(msg);
  }

  static long now_sec()
  {
    using namespace std::chrono;
    return duration_cast<seconds>(system_clock::now().time_since_epoch()).count();
  }

  rclcpp::Publisher<ros_msg_example::msg::SampleData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleDataPublisher>());
  rclcpp::shutdown();
  return 0;
}
