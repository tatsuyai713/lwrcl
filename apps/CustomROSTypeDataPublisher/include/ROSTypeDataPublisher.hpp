#ifndef ROSTYPEDATAPUBLISHER_H_
#define ROSTYPEDATAPUBLISHER_H_
#include <thread>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <functional>
#include <yaml-cpp/yaml.h>
#include "CustomMessagePubSubTypes.h"
#include "CustomMessage.h"
#include "rcl_like_wrapper.hpp"

using namespace rcl_like_wrapper;

class ROSTypeDataPublisher
{
public:

    ROSTypeDataPublisher();

    virtual ~ROSTypeDataPublisher();

    //!Initialize
    bool init(std::string config_file_path);

    void run();

    //!Publish
    void callbackPublish(int test);

private:

    std::string topic_name_;
    uint16_t interval_ms_;
    uint8_t domain_number_;
    intptr_t node_ptr_;
    intptr_t publisher_ptr_;
    intptr_t timer_ptr_;
    std::function<void()> timer_callback_;
    MessageTypes message_types_;
    std::shared_ptr<CustomMessage> publish_msg_;
    std::unique_ptr<CustomMessagePubSubType> custom_pubsubtype_;

};

#endif /* ROSTYPEDATAPUBLISHER_H_ */
