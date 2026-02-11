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

class MyExecutor : public rclcpp::executors::SingleThreadedExecutor
{
public:
  MyExecutor() : SingleThreadedExecutor()
  {
    add_node(std::make_shared<MyPublisher>());
    add_node(std::make_shared<MySubscriber>());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<MyExecutor>();
  executor->spin();
  executor->cancel();
  rclcpp::shutdown();
  return 0;
}