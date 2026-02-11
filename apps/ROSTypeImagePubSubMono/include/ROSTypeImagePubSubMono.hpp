#ifndef ROSTYPEIMAGEPUBLSUBMONO_H_
#define ROSTYPEIMAGEPUBLSUBMONO_H_

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;

class ROSTypeImagePubSubMono : public Node
{
public:
  ROSTypeImagePubSubMono(uint16_t domain_number);
  ROSTypeImagePubSubMono(const std::string &node_name);
  ROSTypeImagePubSubMono(std::shared_ptr<rclcpp::DomainParticipant> participant);
  ROSTypeImagePubSubMono(std::shared_ptr<rclcpp::DomainParticipant> participant, const std::string &node_name);
  virtual ~ROSTypeImagePubSubMono();

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

#endif /* ROSTYPEIMAGEPUBLSUBMONO_H_ */
