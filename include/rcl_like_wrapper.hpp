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

namespace rcl_like_wrapper
{

    class Rate
    {
    public:
        Rate(std::chrono::milliseconds period);
        ~Rate();
        void sleep();

    private:
        std::chrono::milliseconds period_;
        std::chrono::time_point<std::chrono::steady_clock> start_time_;
    };

    class MessageType
    {
    public:
        MessageType(eprosima::fastdds::dds::TopicDataType *message_type);
        MessageType(const MessageType &other);
        MessageType &operator=(const MessageType &other);
        MessageType();
        ~MessageType();

        eprosima::fastdds::dds::TypeSupport type_support;
    };

    // Alias for a map that associates message type names (strings) with MessageType instances.
    using MessageTypes = std::unordered_map<std::string, MessageType>;

    intptr_t create_node(uint16_t domain_id);
    void node_destroy_node(intptr_t node_ptr);
    void node_spin(intptr_t node_ptr);
    void node_spin_once(intptr_t node_ptr);
    void node_stop_spin(intptr_t node_ptr);
    intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos);
    void publish(intptr_t publisher_ptr, void *message);
    int32_t publisher_get_subscriber_count(intptr_t publisher_ptr);
    void destroy_publisher(intptr_t publisher_ptr);
    intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos, std::function<void(void *)> callback);
    int32_t subscriber_get_publisher_count(intptr_t subscriber_ptr);
    void destroy_subscriber(intptr_t subscriber_ptr);
    intptr_t create_timer(intptr_t node_ptr, std::chrono::milliseconds period, std::function<void()> callback);
    void rcl_like_wrapper_init(const MessageTypes &types);

} // namespace rcl_like_wrapper
