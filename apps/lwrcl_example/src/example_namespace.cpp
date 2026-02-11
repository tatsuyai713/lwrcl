/**
 * @file example_namespace.cpp
 * @brief Example demonstrating Node namespace support
 * 
 * This example shows how to create nodes with namespaces, similar to rclcpp.
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class NamespacedPublisher : public rclcpp::Node
{
public:
  // Constructor with namespace support
  NamespacedPublisher(const std::string &node_name, const std::string &ns)
    : Node(node_name, ns), count_(0)
  {
    // The topic will be resolved with the namespace: /my_namespace/chatter
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&NamespacedPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node name: %s", this->get_name().c_str());
    RCLCPP_INFO(this->get_logger(), "Namespace: %s", this->get_namespace().c_str());
    RCLCPP_INFO(this->get_logger(), "Fully qualified name: %s", this->get_fully_qualified_name().c_str());
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data() = "Hello from " + this->get_fully_qualified_name() + " #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data().c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

class NamespacedSubscriber : public rclcpp::Node
{
public:
  NamespacedSubscriber(const std::string &node_name, const std::string &ns)
    : Node(node_name, ns)
  {
    // Subscribe to absolute topic (ignores namespace)
    subscription_absolute_ = this->create_subscription<std_msgs::msg::String>(
      "/my_namespace/chatter", 10,
      std::bind(&NamespacedSubscriber::absolute_callback, this, std::placeholders::_1));

    // Subscribe to relative topic (uses namespace)
    subscription_relative_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&NamespacedSubscriber::relative_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscriber node: %s", this->get_fully_qualified_name().c_str());
    RCLCPP_INFO(this->get_logger(), "  Absolute topic: /my_namespace/chatter");
    RCLCPP_INFO(this->get_logger(), "  Relative topic: %s/chatter", this->get_namespace().c_str());
  }

private:
  void absolute_callback(std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Absolute] Received: '%s'", msg->data().c_str());
  }

  void relative_callback(std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Relative] Received: '%s'", msg->data().c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_absolute_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_relative_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Create a publisher node with namespace "my_namespace"
  auto publisher_node = std::make_shared<NamespacedPublisher>("talker", "my_namespace");
  
  // Create a subscriber node with namespace "my_namespace"
  auto subscriber_node = std::make_shared<NamespacedSubscriber>("listener", "my_namespace");

  // Add nodes to executor
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  std::cout << "\n=== Namespace Example ===" << std::endl;
  std::cout << "Publisher fully qualified name: " << publisher_node->get_fully_qualified_name() << std::endl;
  std::cout << "Subscriber fully qualified name: " << subscriber_node->get_fully_qualified_name() << std::endl;
  std::cout << "========================\n" << std::endl;

  // Spin the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
