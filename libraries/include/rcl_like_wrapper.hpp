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

#include "eprosima_namespace.hpp" // Placeholder for actual eProsima namespace definitions

namespace rcl_like_wrapper
{
    // Represents a message type, wrapping Fast DDS TypeSupport for serialization/deserialization
    class MessageType
    {
    public:
        // Constructor from a Fast DDS TopicDataType
        MessageType(eprosima::fastdds::dds::TopicDataType *message_type);
        // Copy constructor
        MessageType(const MessageType &other);
        // Assignment operator
        MessageType &operator=(const MessageType &other);
        // Default constructor
        MessageType();
        // Destructor
        ~MessageType();

        // Fast DDS TypeSupport for this message type
        eprosima::fastdds::dds::TypeSupport type_support;
    };

    // Maps message type names to MessageType objects
    using MessageTypes = std::unordered_map<std::string, MessageType>;

    // Functions to manage nodes, publishers, subscriptions, and timers
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

    // Represents a node in the communication graph, managing lifecycle and communication capabilities
    class RCLWNode
    {
    protected:
        uint8_t domain_number_;                       // DDS domain number for network segmentation
        intptr_t node_ptr_;                           // Pointer to the underlying implementation-specific node object
        MessageTypes message_types_;                  // Supported message types for this node
        bool rclw_node_stop_flag_;                    // Flag to indicate the node should stop processing
        std::mutex mutex_;                            // Mutex to protect access to the nodes list

    public:
        RCLWNode();
        ~RCLWNode();
        virtual bool init(const std::string &config_file_path) = 0; // Initialize the node with configuration
        virtual void spin();                                        // Process messages continuously
        virtual void stop();                                        // Stop processing messages
        intptr_t get_node_pointer();                                // Get the underlying node pointer
    };

    // Manages a collection of nodes, coordinating their execution
    class Executor
    {
    public:
        Executor();
        ~Executor();

        void add_node(intptr_t node_ptr);    // Add a node to the executor
        void remove_node(intptr_t node_ptr); // Remove a node from the executor
        void stop();                         // Stop all nodes in the executor
        void spin();                         // Start processing messages for all nodes

    private:
        std::vector<intptr_t> nodes_; // List of nodes managed by the executor
        std::mutex mutex_;            // Mutex to protect access to the nodes list
        bool running_;                // Flag indicating if the executor is running
    };

    // Utility class to control the rate of execution in a loop
    class Rate
    {
    public:
        Rate(std::chrono::milliseconds period); // Constructor with specified period
        ~Rate();

        void sleep(); // Sleep until the next period

    private:
        std::chrono::milliseconds period_;                              // Duration of the period
        std::chrono::time_point<std::chrono::steady_clock> start_time_; // Time point when the period started
    };

} // namespace rcl_like_wrapper
