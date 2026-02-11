/**
 * @file example_zero_copy_pub.cpp
 * @brief Example demonstrating zero-copy publishing with LoanedMessage
 * 
 * This example shows how to use the LoanedMessage API for zero-copy publishing,
 * which can improve performance for large messages like images.
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ZeroCopyPublisher : public rclcpp::Node
{
public:
  ZeroCopyPublisher() : Node("zero_copy_publisher"), count_(0)
  {
    // Create publisher with SensorDataQoS for optimal zero-copy performance
    rclcpp::SensorDataQoS qos;
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", qos);

    timer_ = this->create_wall_timer(100ms, std::bind(&ZeroCopyPublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Zero-copy publisher started");
    RCLCPP_INFO(this->get_logger(), "Can loan messages: %s", 
                publisher_->can_loan_messages() ? "yes" : "no (will use fallback)");
  }

private:
  void timer_callback()
  {
    // Method 1: Traditional publish (with copy)
    if (count_ % 2 == 0)
    {
      auto message = std::make_shared<sensor_msgs::msg::Image>();
      fill_image_data(*message);
      
      RCLCPP_INFO(this->get_logger(), "[Traditional] Publishing image #%zu (%dx%d)", 
                  count_, message->width(), message->height());
      publisher_->publish(message);
    }
    // Method 2: Zero-copy publish using LoanedMessage
    else
    {
      // Borrow a message from the middleware
      // If the middleware supports loaning, the memory is allocated in shared memory
      // If not, it falls back to regular heap allocation
      auto loaned_msg = publisher_->borrow_loaned_message();
      
      if (loaned_msg.is_valid())
      {
        // Fill in the message data directly in the loaned memory
        fill_image_data(loaned_msg.get());
        
        RCLCPP_INFO(this->get_logger(), "[Zero-copy] Publishing image #%zu (%dx%d)", 
                    count_, loaned_msg.get().width(), loaned_msg.get().height());
        
        // Publish the loaned message (ownership is transferred to middleware)
        publisher_->publish(std::move(loaned_msg));
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to borrow loaned message, falling back to traditional publish");
        auto message = std::make_shared<sensor_msgs::msg::Image>();
        fill_image_data(*message);
        publisher_->publish(message);
      }
    }

    count_++;
  }

  void fill_image_data(sensor_msgs::msg::Image &msg)
  {
    msg.header().stamp().sec() = count_;
    msg.header().stamp().nanosec() = 0;
    msg.header().frame_id() = "camera_frame";
    msg.height() = 480;
    msg.width() = 640;
    msg.encoding() = "rgb8";
    msg.is_bigendian() = false;
    msg.step() = 640 * 3;
    
    // Create image data (simulated pattern)
    size_t data_size = 640 * 480 * 3;
    msg.data().resize(data_size);
    
    // Fill with a pattern based on count
    uint8_t pattern = static_cast<uint8_t>(count_ % 256);
    for (size_t i = 0; i < data_size; i++)
    {
      msg.data()[i] = pattern;
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  std::cout << "\n=== Zero-Copy Publishing Example ===" << std::endl;
  std::cout << "This example alternates between:" << std::endl;
  std::cout << "  - Traditional publish (even counts)" << std::endl;
  std::cout << "  - Zero-copy publish using LoanedMessage (odd counts)" << std::endl;
  std::cout << "=====================================\n" << std::endl;
  
  rclcpp::spin(std::make_shared<ZeroCopyPublisher>());
  rclcpp::shutdown();
  return 0;
}
