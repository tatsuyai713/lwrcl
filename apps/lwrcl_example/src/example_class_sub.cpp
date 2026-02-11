#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : Node("my_subscriber")
  {
    rclcpp::QoS qos_depth(10);
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", qos_depth, std::bind(&MySubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data().c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}