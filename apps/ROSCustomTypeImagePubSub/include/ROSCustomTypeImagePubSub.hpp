#ifndef ROSCUSTOMTYPEIMAGEPUBSUB_H_
#define ROSCUSTOMTYPEIMAGEPUBSUB_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;

class ROSCustomTypeImagePubSub : public Node
{
public:
  ROSCustomTypeImagePubSub(uint16_t domain_number);
  ROSCustomTypeImagePubSub(const std::string &node_name);
  ROSCustomTypeImagePubSub(std::shared_ptr<lwrcl::DomainParticipant> participant);
  ROSCustomTypeImagePubSub(std::shared_ptr<lwrcl::DomainParticipant> participant, const std::string &node_name);
  virtual ~ROSCustomTypeImagePubSub();

  // Callback function to subscribe data
  void callbackSubscribe(sensor_msgs::msg::Image::SharedPtr message);

private:
  void init();
  std::string publish_topic_name_;
  std::string subscribe_topic_name_;
  int interval_ms_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ptr_;
  Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  int counter_;
  sensor_msgs::msg::Image::SharedPtr gray_msg_;
};

#endif /* ROSCUSTOMTYPEIMAGEPUBSUB_H_ */
