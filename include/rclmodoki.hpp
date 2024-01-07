#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include "eprosima_namespace.hpp"

namespace rclmodoki {


class MessageType {
public:
    MessageType(eprosima::fastdds::dds::TopicDataType *message_type)
        : type_support(message_type) {}

    MessageType(const MessageType& other)
        : type_support(other.type_support) {}

    MessageType() : type_support(nullptr) {}

    MessageType& operator=(const MessageType& other) {
        if (this != &other) {
            type_support = other.type_support;
        }
        return *this;
    }

    ~MessageType() {}

    eprosima::fastdds::dds::TypeSupport type_support;
};

using MessageTypes = std::unordered_map<std::string, MessageType>;

int64_t node_create_node(int64_t domain_id);
void node_destroy_node(int64_t node_ptr);
void node_spin(int64_t node_ptr);
void node_stop_spin(int64_t node_ptr);
int64_t publisher_create_publisher(int64_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos& qos);
void publisher_publish_impl(int64_t publisher_ptr, void* message);
int32_t publisher_get_subscription_count(int64_t publisher_ptr);
int64_t subscription_create_subscription(int64_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos& qos, std::function<void(void*)> callback);
int32_t subscription_get_publisher_count(int64_t subscription_ptr);
void rclmodoki_init(const MessageTypes &types);

} // namespace rclmodoki
