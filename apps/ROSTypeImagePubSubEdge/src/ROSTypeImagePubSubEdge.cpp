#include "ROSTypeImagePubSubEdge.hpp"
#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>

ROSTypeImagePubSubEdge::ROSTypeImagePubSubEdge(uint16_t domain_number)
    : Node(domain_number), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  edge_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubEdge::ROSTypeImagePubSubEdge(const std::string &node_name)
    : Node(node_name), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  edge_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubEdge::ROSTypeImagePubSubEdge(std::shared_ptr<rclcpp::DomainParticipant> participant)
    : Node(participant), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  edge_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubEdge::ROSTypeImagePubSubEdge(std::shared_ptr<rclcpp::DomainParticipant> participant, const std::string &node_name)
    : Node(participant, node_name), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000)
{
  counter_ = 0;

  edge_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  init();
}

ROSTypeImagePubSubEdge::~ROSTypeImagePubSubEdge()
{
}

void ROSTypeImagePubSubEdge::init()
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

  subscriber_ptr_ = create_subscription<sensor_msgs::msg::Image>(subscribe_topic_name_, 10, std::bind(&ROSTypeImagePubSubEdge::callbackSubscribe, this, std::placeholders::_1));
  if (subscriber_ptr_ == 0)
  {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return;
  }

  return;
}

void ROSTypeImagePubSubEdge::callbackSubscribe(sensor_msgs::msg::Image::SharedPtr message)
{
  if (message == nullptr)
  {
    std::cerr << "Error: Received null message in callback." << std::endl;
    return;
  }

  int width = message->width();
  int height = message->height();

  cv::Mat gray_image(height, width, CV_8UC1, message->data().data());

  cv::Mat edges;
  cv::Canny(gray_image, edges, 50, 150);

  edge_msg_->width(width);
  edge_msg_->height(height);
  edge_msg_->encoding("mono8");
  edge_msg_->step(edges.step);
  edge_msg_->data(std::vector<uint8_t>(edges.data, edges.data + edges.total() * edges.elemSize()));

  publisher_ptr_->publish(edge_msg_);
}
