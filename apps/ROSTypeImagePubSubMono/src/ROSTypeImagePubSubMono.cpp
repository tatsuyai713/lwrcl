#include "ROSTypeImagePubSubMono.hpp"
#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>

ROSTypeImagePubSubMono::ROSTypeImagePubSubMono(uint16_t domain_number)
    : Node(domain_number), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  gray_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubMono::ROSTypeImagePubSubMono(const std::string &node_name)
    : Node(node_name), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  gray_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubMono::ROSTypeImagePubSubMono(std::shared_ptr<rclcpp::DomainParticipant> participant)
    : Node(participant), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  gray_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubMono::ROSTypeImagePubSubMono(std::shared_ptr<rclcpp::DomainParticipant> participant, const std::string &node_name)
    : Node(participant, node_name), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  gray_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubMono::~ROSTypeImagePubSubMono()
{
}

void ROSTypeImagePubSubMono::init()
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

  subscriber_ptr_ = create_subscription<sensor_msgs::msg::Image>(subscribe_topic_name_, 10, std::bind(&ROSTypeImagePubSubMono::callbackSubscribe, this, std::placeholders::_1));
  if (subscriber_ptr_ == 0)
  {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return;
  }

  return;
}

void ROSTypeImagePubSubMono::callbackSubscribe(sensor_msgs::msg::Image::SharedPtr message)
{
  if (message == nullptr)
  {
    std::cerr << "Error: Received null message in callback." << std::endl;
    return;
  }

  int width = message->width();
  int height = message->height();

  cv::Mat cv_image(height, width, CV_8UC3, message->data().data());

  cv::Mat gray_image;
  cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);

  gray_msg_->width(width);
  gray_msg_->height(height);
  gray_msg_->encoding("mono8");
  gray_msg_->step(gray_image.step);
  gray_msg_->data(std::vector<uint8_t>(gray_image.data, gray_image.data + gray_image.total() * gray_image.elemSize()));

  publisher_ptr_->publish(gray_msg_);
}
