#include "ROSTypeImagePubSub.hpp"
#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>

ROSTypeImagePubSub::ROSTypeImagePubSub(uint16_t domain_number)
    : Node(domain_number), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;
  init();
}

ROSTypeImagePubSub::ROSTypeImagePubSub(std::string node_name)
    : Node(node_name), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;
  init();
}

ROSTypeImagePubSub::ROSTypeImagePubSub(std::shared_ptr<rclcpp::DomainParticipant> participant)
    : Node(participant), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;
  init();
}

ROSTypeImagePubSub::~ROSTypeImagePubSub()
{
}

void ROSTypeImagePubSub::init()
{
  this->declare_parameter("publish_topic_name", std::string("default_topic_pub"));
  this->declare_parameter("publish_topic_interval_ms", 100);
  this->declare_parameter("subscribe_topic_name", std::string("default_topic_sub"));
  this->declare_parameter("subscribe_topic_interval_ms", 100);

  this->get_parameter("publish_topic_name", publish_topic_name_);
  this->get_parameter("publish_topic_interval_ms", interval_ms_);
  this->get_parameter("subscribe_topic_name", subscribe_topic_name_);

  std::cout << "publish_topic_name: " << publish_topic_name_ << std::endl;
  std::cout << "interval_ms: " << interval_ms_ << std::endl;
  std::cout << "subscribe_topic_name: " << subscribe_topic_name_ << std::endl;

  if (interval_ms_ <= 0)
  {
    std::cerr << "Interval Time Error!" << std::endl;
    return;
  }

  publisher_ptr_ = create_publisher<sensor_msgs::msg::Image>(publish_topic_name_, 10);
  if (!publisher_ptr_)
  {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
    return;
  }

  // Create a subscription with a topic named subscribe_topic_name_ and default QoS
  subscriber_ptr_ = create_subscription<sensor_msgs::msg::Image>(subscribe_topic_name_, 10, std::bind(&ROSTypeImagePubSub::callbackSubscribe, this, std::placeholders::_1));
  if (subscriber_ptr_ == 0)
  {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return;
  }

  int test = 100;

  // Setup timer for periodic callback
  timer_callback_ = [this, test]()
  { this->callbackPublish(test); };
  timer_ptr_ = create_wall_timer(std::chrono::milliseconds(interval_ms_), timer_callback_);
  if (!timer_ptr_)
  {
    std::cerr << "Error: Failed to create a timer." << std::endl;
    return;
  }

  return;
}

void ROSTypeImagePubSub::callbackPublish(int test)
{
  // Update and publish message
  std::shared_ptr<sensor_msgs::msg::Image> publish_msg = std::make_shared<sensor_msgs::msg::Image>();
  publish_msg->header().stamp().sec() = (test + counter_);
  counter_++;

  publisher_ptr_->publish(publish_msg);
}

void ROSTypeImagePubSub::callbackSubscribe(sensor_msgs::msg::Image::SharedPtr message)
{
  if (message == nullptr)
  {
    std::cerr << "Error: Received null message in callback." << std::endl;
    return;
  }

  // Handle the received message
  std::cout << "Received data: " << message->header().frame_id() << std::endl;
}