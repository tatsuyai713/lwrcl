#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher"), count_(0)
  {
    rclcpp::QoS qos_depth(10);
    publisher_ =
      this->create_publisher<std_msgs::msg::String>("topic", qos_depth);
    timer_ = this->create_wall_timer(500ms, std::bind(&MyPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data() = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data().c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}