// sample_data_subscriber.cpp
//
// Subscribes to ros_msg_example::msg::SampleData. Reads fields with plain
// member access (post-patch headers expose them as public data members).

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "ros_msg_example/msg/sample_data.hpp"

class SampleDataSubscriber : public rclcpp::Node
{
public:
  SampleDataSubscriber()
  : Node("sample_data_subscriber")
  {
    subscription_ = create_subscription<ros_msg_example::msg::SampleData>(
      "sample_data", rclcpp::QoS(10),
      [this](std::shared_ptr<ros_msg_example::msg::SampleData> msg) {
        on_msg(*msg);
      });
  }

private:
  void on_msg(const ros_msg_example::msg::SampleData & msg)
  {
    const char * status_str = "?";
    switch (msg.status) {
      case ros_msg_example::msg::SampleData__STATUS_OK:    status_str = "OK"; break;
      case ros_msg_example::msg::SampleData__STATUS_WARN:  status_str = "WARN"; break;
      case ros_msg_example::msg::SampleData__STATUS_ERROR: status_str = "ERROR"; break;
    }

    RCLCPP_INFO(get_logger(),
      "received frame=%s seq=%d value=%.2f label=%s status=%s",
      msg.header.frame_id.c_str(),
      msg.sequence_number,
      msg.value,
      msg.label.c_str(),
      status_str);
  }

  rclcpp::Subscription<ros_msg_example::msg::SampleData>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SampleDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}
