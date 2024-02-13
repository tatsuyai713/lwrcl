#ifndef ROSTYPEIMAGEPUBLSUBEDGE_H_
#define ROSTYPEIMAGEPUBLSUBEDGE_H_

#include "rcl_like_wrapper.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"
#include <functional>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace rcl_like_wrapper;
class ROSTypeImagePubSubEdge : public RCLWNode {
public:
    ROSTypeImagePubSubEdge(uint16_t domain_number);
    virtual ~ROSTypeImagePubSubEdge();

    // Override init and run methods from RCLWNode
    bool init(const std::string& config_file_path) override;
    // void run() override;

    // Callback function to subscribe data
    void callbackSubscribe(void* message);

private:
    std::string publish_topic_name_;
    std::string subscribe_topic_name_;
    uint16_t interval_ms_;
    intptr_t publisher_ptr_;
    intptr_t subscriber_ptr_;
    intptr_t timer_ptr_;
    MessageTypes message_types_;
    int counter_;
    std::shared_ptr<sensor_msgs::msg::Image> edge_msg_;
};

#endif /* ROSTYPEIMAGEPUBLSUBEDGE_H_ */
