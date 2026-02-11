/**
 * @file example_timer_control.cpp
 * @brief Example demonstrating timer control (cancel, reset, is_canceled)
 * 
 * This example shows how to control timers dynamically, including
 * canceling, resetting, and checking timer state.
 */

#include <chrono>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TimerControlDemo : public rclcpp::Node
{
public:
  TimerControlDemo() : Node("timer_control_demo"), count_(0)
  {
    // Create a publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("timer_output", 10);

    // Create the main periodic timer (1 second interval)
    main_timer_ = this->create_wall_timer(1s, std::bind(&TimerControlDemo::main_timer_callback, this));

    // Create a control timer that demonstrates cancel/reset (runs every 5 seconds)
    control_timer_ = this->create_wall_timer(5s, std::bind(&TimerControlDemo::control_callback, this));

    RCLCPP_INFO(this->get_logger(), "Timer Control Demo started");
    RCLCPP_INFO(this->get_logger(), "Main timer period: %ld ms", 
                main_timer_->get_period().nanoseconds() / 1000000);
  }

private:
  void main_timer_callback()
  {
    count_++;
    
    auto message = std_msgs::msg::String();
    message.data() = "Timer tick #" + std::to_string(count_);
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data().c_str());
    publisher_->publish(message);
  }

  void control_callback()
  {
    static int control_count = 0;
    control_count++;

    RCLCPP_WARN(this->get_logger(), "=== Control callback #%d ===", control_count);

    switch (control_count % 4)
    {
      case 1:
        // Cancel the main timer
        RCLCPP_WARN(this->get_logger(), "Canceling main timer...");
        main_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Main timer is_canceled: %s", 
                    main_timer_->is_canceled() ? "true" : "false");
        break;

      case 2:
        // Reset the main timer (re-enable it)
        RCLCPP_WARN(this->get_logger(), "Resetting main timer...");
        main_timer_->reset();
        RCLCPP_INFO(this->get_logger(), "Main timer is_canceled: %s", 
                    main_timer_->is_canceled() ? "true" : "false");
        break;

      case 3:
        // Check if timer is ready
        RCLCPP_INFO(this->get_logger(), "Timer state check:");
        RCLCPP_INFO(this->get_logger(), "  is_canceled: %s", main_timer_->is_canceled() ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  is_ready: %s", main_timer_->is_ready() ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  period: %ld ms",
                    main_timer_->get_period().nanoseconds() / 1000000);
        break;

      case 0:
        // Show current count
        RCLCPP_INFO(this->get_logger(), "Main timer has fired %zu times", count_);
        break;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  std::cout << "\n=== Timer Control Example ===" << std::endl;
  std::cout << "This example demonstrates:" << std::endl;
  std::cout << "  - timer->cancel(): Stop the timer" << std::endl;
  std::cout << "  - timer->reset(): Restart the timer" << std::endl;
  std::cout << "  - timer->is_canceled(): Check if timer is canceled" << std::endl;
  std::cout << "  - timer->is_ready(): Check if timer is ready to fire" << std::endl;
  std::cout << "  - timer->get_period(): Get the timer period" << std::endl;
  std::cout << "" << std::endl;
  std::cout << "Watch as the main timer gets canceled and reset every 5 seconds." << std::endl;
  std::cout << "==============================\n" << std::endl;
  
  rclcpp::spin(std::make_shared<TimerControlDemo>());
  rclcpp::shutdown();
  return 0;
}
