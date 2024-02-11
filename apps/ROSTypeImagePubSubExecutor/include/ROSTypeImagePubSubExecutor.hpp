#ifndef ROSTYPEIMAGEPUBLSUBEXECUTOR_H_
#define ROSTYPEIMAGEPUBLSUBEXECUTOR_H_

#include "rcl_like_wrapper.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace rcl_like_wrapper;
class ROSTypeImagePubSubExecutor : public RCLWNode {
public:
    ROSTypeImagePubSubExecutor(uint16_t domain_number);
    virtual ~ROSTypeImagePubSubExecutor();

    // Override init and run methods from RCLWNode
    bool init(const std::string& config_file_path) override;
    // void run() override;

    // Callback function to publish data
    void callbackPublish(int test);

    // Callback function to subscribe data
    void callbackSubscribe(void* message);

private:
    std::string publish_topic_name_;
    std::string subscribe_topic_name_;
    uint16_t interval_ms_;
    intptr_t publisher_ptr_;
    intptr_t subscriber_ptr_;
    intptr_t timer_ptr_;
    std::function<void()> timer_callback_;
    MessageTypes message_types_;
    int counter_;
};

#endif /* ROSTYPEIMAGEPUBLSUBEXECUTOR_H_ */
