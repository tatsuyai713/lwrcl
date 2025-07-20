#include "ROSTypeDataPublisher.hpp"
#include <iostream>
#include <chrono>

ROSTypeDataPublisher::ROSTypeDataPublisher(uint16_t domain_number)
    : Node(domain_number), topic_name_("default_topic"), interval_ms_(1000)
{
  init();
}

ROSTypeDataPublisher::ROSTypeDataPublisher(const std::string &node_name)
    : Node(node_name), topic_name_("default_topic"), interval_ms_(1000)
{
  init();
}

ROSTypeDataPublisher::ROSTypeDataPublisher(std::shared_ptr<rclcpp::DomainParticipant> participant)
    : Node(participant), topic_name_("default_topic"), interval_ms_(1000)
{
  init();
}

ROSTypeDataPublisher::ROSTypeDataPublisher(std::shared_ptr<rclcpp::DomainParticipant> participant, const std::string &node_name)
    : Node(participant, node_name), topic_name_("default_topic"), interval_ms_(1000)
{
  init();
}

ROSTypeDataPublisher::~ROSTypeDataPublisher()
{
  init();
}

void ROSTypeDataPublisher::init()
{
  this->declare_parameter("publish_topic_name", std::string("default_topic_pub"));
  this->declare_parameter("publish_topic_interval_ms", 100);

  this->get_parameter("publish_topic_name", topic_name_);
  this->get_parameter("publish_topic_interval_ms", interval_ms_);
  
  std::cout << "publish_topic_name: " << topic_name_ << std::endl;
  std::cout << "interval_ms: " << interval_ms_ << std::endl;

  if (interval_ms_ <= 0)
  {
    std::cerr << "Interval Time Error!" << std::endl;
    return;
  }

  publisher_ptr_ = create_publisher<CustomROSTypeDataPublisher::msg::CustomMessage>(topic_name_, 10);
  if (!publisher_ptr_)
  {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
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

void ROSTypeDataPublisher::callbackPublish(int test)
{
  // Update and publish message
  std::shared_ptr<CustomROSTypeDataPublisher::msg::CustomMessage> publish_msg = std::make_shared<CustomROSTypeDataPublisher::msg::CustomMessage>();
  publish_msg->index(publish_msg->index() + 1);
  std::string s = "BigData" + std::to_string(publish_msg->index() % 10);
  publish_msg->message(s);

  publisher_ptr_->publish(publish_msg);
}
