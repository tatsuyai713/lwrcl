/**
 * @file example_qos_presets.cpp
 * @brief Example demonstrating QoS preset profiles (SensorDataQoS, ReliableQoS, etc.)
 * 
 * This example shows how to use the new QoS preset classes that match rclcpp's API.
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class QoSPresetsDemo : public rclcpp::Node
{
public:
  QoSPresetsDemo() : Node("qos_presets_demo"), count_(0)
  {
    // SensorDataQoS: Best effort, volatile, keep last 5
    // Ideal for sensor data that is time-sensitive
    rclcpp::SensorDataQoS sensor_qos;
    sensor_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "sensor_topic", sensor_qos);

    // ReliableQoS: Reliable delivery
    rclcpp::ReliableQoS reliable_qos;
    reliable_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "reliable_topic", reliable_qos);

    // BestEffortQoS: Best effort delivery
    rclcpp::BestEffortQoS best_effort_qos;
    best_effort_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "best_effort_topic", best_effort_qos);

    // Custom QoS with deadline and lifespan
    // Note: liveliness_lease_duration requires announcement_period to be set
    // in Fast DDS (lease_duration > announcement_period), which is not directly
    // exposed in this API. Using AUTOMATIC liveliness without custom lease duration.
    rclcpp::QoS custom_qos(10);
    custom_qos.reliability(rclcpp::QoS::ReliabilityPolicy::RELIABLE)
              .durability(rclcpp::QoS::DurabilityPolicy::TRANSIENT_LOCAL)
              .deadline(std::chrono::milliseconds(100))
              .lifespan(std::chrono::seconds(1))
              .liveliness(rclcpp::QoS::LivelinessPolicy::AUTOMATIC);
    custom_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "custom_topic", custom_qos);

    // Create a subscription with SensorDataQoS
    sensor_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "sensor_topic", sensor_qos,
      std::bind(&QoSPresetsDemo::sensor_callback, this, std::placeholders::_1));

    // Timer to publish messages
    timer_ = this->create_wall_timer(100ms, std::bind(&QoSPresetsDemo::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "QoS Presets Demo started");
    RCLCPP_INFO(this->get_logger(), "  - sensor_topic: SensorDataQoS (best effort, volatile)");
    RCLCPP_INFO(this->get_logger(), "  - reliable_topic: ReliableQoS");
    RCLCPP_INFO(this->get_logger(), "  - best_effort_topic: BestEffortQoS");
    RCLCPP_INFO(this->get_logger(), "  - custom_topic: Custom QoS with deadline/lifespan");
  }

private:
  void timer_callback()
  {
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message->header().stamp().sec() = count_;
    message->header().stamp().nanosec() = 0;
    message->header().frame_id() = "demo_frame";
    message->height() = 480;
    message->width() = 640;
    message->encoding() = "rgb8";
    message->is_bigendian() = false;
    message->step() = 640 * 3;
    message->data() = std::vector<uint8_t>(640 * 480 * 3, static_cast<uint8_t>(count_ % 256));

    RCLCPP_INFO(this->get_logger(), "Publishing image #%zu", count_);
    
    sensor_publisher_->publish(message);
    reliable_publisher_->publish(message);
    best_effort_publisher_->publish(message);
    custom_publisher_->publish(message);

    count_++;
  }

  void sensor_callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image: %dx%d, frame_id: %s",
                msg->width(), msg->height(), msg->header().frame_id().c_str());
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr sensor_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reliable_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr best_effort_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr custom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sensor_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSPresetsDemo>());
  rclcpp::shutdown();
  return 0;
}
