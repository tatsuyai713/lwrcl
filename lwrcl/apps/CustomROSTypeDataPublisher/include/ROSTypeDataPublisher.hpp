#ifndef ROSTYPEDATAPUBLISHER_H_
#define ROSTYPEDATAPUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "CustomROSTypeDataPublisher/msg/custom_message.hpp"
#include <functional>
#include <memory>
#include <string>

using namespace rclcpp;

class ROSTypeDataPublisher : public Node
{
public:
  ROSTypeDataPublisher(uint16_t domain_number);
  ROSTypeDataPublisher(const std::string &node_name);
  ROSTypeDataPublisher(std::shared_ptr<rclcpp::DomainParticipant> participant);
  ROSTypeDataPublisher(std::shared_ptr<rclcpp::DomainParticipant> participant, const std::string &node_name);
  virtual ~ROSTypeDataPublisher();

  void init();

  // Callback function to publish data
  void callbackPublish(int test);

private:
  std::string topic_name_;
  int interval_ms_;
  Publisher<CustomROSTypeDataPublisher::msg::CustomMessage>::SharedPtr publisher_ptr_;
  TimerBase::SharedPtr timer_ptr_;
  std::function<void()> timer_callback_;
};

#endif /* ROSTYPEDATAPUBLISHER_H_ */
