#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>

#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include "eprosima_namespace.hpp"

namespace rcl_like_wrapper
{

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

    using MessageTypes = std::unordered_map<std::string, MessageType>;

    intptr_t create_node(uint16_t domain_id);
    void destroy_node(intptr_t node_ptr);
    void spin(intptr_t node_ptr);
    void spin_once(intptr_t node_ptr);
    void spin_some(intptr_t node_ptr);
    void stop_spin(intptr_t node_ptr);
    intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos);
    void publish(intptr_t publisher_ptr, void *message);
    int32_t get_subscriber_count(intptr_t publisher_ptr);
    void destroy_publisher(intptr_t publisher_ptr);
    intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos, std::function<void(void *)> callback);
    int32_t get_publisher_count(intptr_t subscriber_ptr);
    void destroy_subscription(intptr_t subscriber_ptr);
    intptr_t create_timer(intptr_t node_ptr, std::chrono::milliseconds period, std::function<void()> callback);
    void destroy_timer(intptr_t timer_ptr);
    void rcl_like_wrapper_init(const MessageTypes &types);

    class RCLWNode
    {
    protected:
        uint8_t domain_number_;
        intptr_t node_ptr_;
        MessageTypes message_types_;
        bool rclw_node_stop_flag_;

    public:
        RCLWNode();
        virtual ~RCLWNode() = default;
        virtual bool init(const std::string &config_file_path) = 0;
        virtual void spin();
        virtual void stop();
        intptr_t get_node_pointer();
    };
    class Executor
    {
    public:
        Executor();
        ~Executor();

        void add_node(intptr_t node_ptr);
        void remove_node(intptr_t node_ptr);
        void stop();
        void spin();

    private:
        std::vector<intptr_t> nodes_;
        std::mutex mutex_;
        bool running_;
    };

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

} // namespace rcl_like_wrapper
