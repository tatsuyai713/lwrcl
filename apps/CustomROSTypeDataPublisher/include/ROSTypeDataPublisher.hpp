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

FAST_DDS_CUSTOM_TYPE(msg,CustomMessage)

class ROSTypeDataPublisher : public RCLWNode {
public:
    ROSTypeDataPublisher(uint16_t domain_number);
    virtual ~ROSTypeDataPublisher();

    // Override init and run methods from RCLWNode
    bool init(const std::string& config_file_path) override;
    // void run() override;

    // Callback function to publish data
    void callbackPublish(int test);

private:
    std::string topic_name_;
    uint16_t interval_ms_;
    Publisher<CustomMessage>* publisher_ptr_;
    Timer<std::chrono::milliseconds>* timer_ptr_;
    std::function<void()> timer_callback_;
    // MessageType
    msg::CustomMessageType pub_message_type_;
};

#endif /* ROSTYPEDATAPUBLISHER_H_ */
