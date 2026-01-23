/**
 * @file example_zero_copy_sub.cpp
 * @brief Example demonstrating zero-copy subscription with LoanedSubscriptionMessage
 * 
 * This example shows how to use the take_loaned_message API for zero-copy
 * subscription, which can improve performance for large messages.
 */

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ZeroCopySubscriber : public rclcpp::Node
{
public:
  ZeroCopySubscriber() : Node("zero_copy_subscriber")
  {
    // Create subscription with SensorDataQoS
    rclcpp::SensorDataQoS qos;
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image", qos,
      std::bind(&ZeroCopySubscriber::callback, this, std::placeholders::_1));

    // Create a timer for polling zero-copy messages
    poll_timer_ = this->create_wall_timer(50ms, std::bind(&ZeroCopySubscriber::poll_loaned_messages, this));

    RCLCPP_INFO(this->get_logger(), "Zero-copy subscriber started");
    RCLCPP_INFO(this->get_logger(), "Can loan messages: %s", 
                subscription_->can_loan_messages() ? "yes" : "no");
  }

private:
  // Traditional callback (data is copied)
  void callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[Callback] Received image: %dx%d, stamp: %d.%09d",
                msg->width(), msg->height(),
                msg->header().stamp().sec(), msg->header().stamp().nanosec());
  }

  // Polling method for zero-copy messages
  void poll_loaned_messages()
  {
    // Try to take a loaned message (zero-copy when supported)
    lwrcl::LoanedSubscriptionMessage<sensor_msgs::msg::Image> loaned_msg;
    
    if (subscription_->take_loaned_message(loaned_msg))
    {
      // Access the message data without copying
      const auto &msg = loaned_msg.get();
      
      RCLCPP_INFO(this->get_logger(), "[Zero-copy] Received image: %dx%d, stamp: %d.%09d",
                  msg.width(), msg.height(),
                  msg.header().stamp().sec(), msg.header().stamp().nanosec());

      // Get sample info (contains reception timestamp, etc.)
      const auto &info = loaned_msg.get_sample_info();
      RCLCPP_DEBUG(this->get_logger(), "  Sample state: valid=%s", 
                   info.valid_data ? "true" : "false");

      // Process the data here...
      // The loaned message will be automatically returned to the middleware
      // when 'loaned_msg' goes out of scope
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  std::cout << "\n=== Zero-Copy Subscription Example ===" << std::endl;
  std::cout << "This subscriber receives messages two ways:" << std::endl;
  std::cout << "  - Traditional callback (with copy)" << std::endl;
  std::cout << "  - Polling with take_loaned_message (zero-copy)" << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Run 'example_zero_copy_pub' in another terminal to send images." << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  rclcpp::spin(std::make_shared<ZeroCopySubscriber>());
  rclcpp::shutdown();
  return 0;
}
