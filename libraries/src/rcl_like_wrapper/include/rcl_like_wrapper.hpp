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

#include "eprosima_namespace.hpp" // Placeholder for eProsima Fast DDS namespace definitions

namespace rcl_like_wrapper
{
    // Class representing a message type, encapsulating Fast DDS TypeSupport for serialization and deserialization.
    class MessageType
    {
    public:
        // Constructs a MessageType with a given Fast DDS TopicDataType.
        MessageType(eprosima::fastdds::dds::TopicDataType *message_type);
        // Copy constructor.
        MessageType(const MessageType &other);
        // Copy assignment operator.
        MessageType &operator=(const MessageType &other);
        // Default constructor.
        MessageType();
        // Destructor.
        ~MessageType();

        // Holds Fast DDS TypeSupport for this message type, facilitating serialization/deserialization.
        eprosima::fastdds::dds::TypeSupport type_support;
    };

    // Maps message type names to their corresponding MessageType objects for easy lookup.
    using MessageTypes = std::unordered_map<std::string, MessageType>;

    // Functions for node management, publishing, and subscription handling.
    intptr_t create_node(uint16_t domain_id);
    void destroy_node(intptr_t node_ptr);
    void spin(intptr_t node_ptr);
    void spin_once(intptr_t node_ptr);
    void spin_some(intptr_t node_ptr);
    void stop_spin(intptr_t node_ptr);
    intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos);
    void publish(intptr_t publisher_ptr, void *message);
    int32_t get_subscriber_count(intptr_t publisher_ptr);
    intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos, std::function<void(void *)> callback);
    int32_t get_publisher_count(intptr_t subscriber_ptr);

    // Template function for creating a timer, to be defined in the implementation file
    template <typename Duration>
    intptr_t create_timer(intptr_t node_ptr, Duration period, std::function<void()> callback);

    void rcl_like_wrapper_init(const MessageTypes &types);

    // Represents a node within the ROS-like communication graph, managing its lifecycle and communication capabilities.
    class RCLWNode
    {
    protected:
        intptr_t node_ptr_;        // Pointer to the implementation-specific node object.
        bool rclw_node_stop_flag_; // Flag to signal when the node should stop its processing.
        std::mutex mutex_;         // Mutex for thread-safe access to the node.

    public:
        RCLWNode(uint16_t domain_number);
        virtual ~RCLWNode();
        virtual bool init(const std::string &config_file_path) = 0; // Initializes the node with a configuration file.
        virtual void spin();                                        // Continuously processes messages.
        virtual void spin_some();                                   // Processes available messages without blocking.
        virtual void stop();                                        // Stops message processing.
        intptr_t get_node_pointer();                                // Returns the underlying node pointer.
    };

    // Executor that manages and executes nodes in a single thread.
    class SingleThreadedExecutor
    {
    public:
        SingleThreadedExecutor();
        ~SingleThreadedExecutor();

        void add_node(intptr_t node_ptr);
        void remove_node(intptr_t node_ptr);
        void stop();
        void spin();

    private:
        std::vector<intptr_t> nodes_; // List of nodes managed by the executor.
        std::mutex mutex_;            // Mutex for thread-safe access to the nodes list.
        bool running_;                // Indicates whether the executor is currently running.
    };

    // Executor that manages and executes nodes, each in its own thread, allowing for parallel processing.
    class MultiThreadedExecutor
    {
    public:
        MultiThreadedExecutor();
        ~MultiThreadedExecutor();

        void add_node(intptr_t node_ptr);
        void remove_node(intptr_t node_ptr);
        void stop();
        void spin();

    private:
        std::vector<intptr_t> nodes_;      // List of nodes managed by the executor.
        std::vector<std::thread> threads_; // Threads created for each node for parallel execution.
        std::mutex mutex_;                 // Mutex for thread-safe access to the nodes and threads lists.
        std::atomic<bool> running_;        // Atomic flag indicating whether the executor is currently running.
    };

    class Duration;

    class Time
    {
    public:
        Time();
        Time(int64_t nanoseconds);
        Time(int32_t seconds, uint32_t nanoseconds);
        int64_t nanoseconds() const;
        double seconds() const;

        Time operator+(const Duration &rhs) const;
        Time operator-(const Duration &rhs) const;
        Duration operator-(const Time &rhs) const;

        bool operator==(const Time &rhs) const;
        bool operator!=(const Time &rhs) const;
        bool operator<(const Time &rhs) const;
        bool operator<=(const Time &rhs) const;
        bool operator>(const Time &rhs) const;
        bool operator>=(const Time &rhs) const;

    private:
        int64_t nanoseconds_;
    };

    class Duration
    {
    public:
        Duration();
        Duration(int64_t nanoseconds);
        Duration(int32_t seconds, uint32_t nanoseconds);
        int64_t nanoseconds() const;
        double seconds() const;

        Duration operator+(const Duration &rhs) const;
        Duration operator-(const Duration &rhs) const;

        bool operator==(const Duration &rhs) const;
        bool operator!=(const Duration &rhs) const;
        bool operator<(const Duration &rhs) const;
        bool operator<=(const Duration &rhs) const;
        bool operator>(const Duration &rhs) const;
        bool operator>=(const Duration &rhs) const;

    private:
        int64_t nanoseconds_;
    };

    class Clock
    {
    public:
        enum class ClockType
        {
            SYSTEM_TIME
        };

    private:
        ClockType type_;

    public:
        explicit Clock(ClockType type = ClockType::SYSTEM_TIME);
        Time now();
        ClockType get_clock_type() const;
    };

    class Rate
    {
    private:
        Duration period_;
        std::chrono::steady_clock::time_point next_time_;

    public:
        explicit Rate(const Duration &period);
        void sleep();
    };

} // namespace rcl_like_wrapper
