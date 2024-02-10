#ifndef ROSTYPEDATAPUBLISHER_H_
#define ROSTYPEDATAPUBLISHER_H_

#include "rcl_like_wrapper.hpp"
#include "CustomMessagePubSubTypes.h"
#include "CustomMessage.h"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace rcl_like_wrapper;
class ROSTypeDataPublisherExecutor : public RCLWNode {
public:
    ROSTypeDataPublisherExecutor(uint16_t domain_number);
    virtual ~ROSTypeDataPublisherExecutor();

    // Override init and run methods from RCLWNode
    bool init(const std::string& config_file_path) override;
    // void run() override;

    // Callback function to publish data
    void callbackPublish(int test);

private:
    std::string topic_name_;
    uint16_t interval_ms_;
    intptr_t publisher_ptr_;
    intptr_t timer_ptr_;
    std::function<void()> timer_callback_;
    MessageTypes message_types_;
    CustomMessagePubSubType custom_pubsubtype_;
    sensor_msgs::msg::ImagePubSubType image_pubsubtype_;
};

#endif /* ROSTYPEDATAPUBLISHER_H_ */
