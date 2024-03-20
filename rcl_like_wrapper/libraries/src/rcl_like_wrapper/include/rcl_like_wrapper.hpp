#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <forward_list>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

#include "eprosima_namespace.hpp"
#include "dds_message_type.hpp"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "timer.hpp"

namespace rcl_like_wrapper
{
  
  class Node
  {
  public:
    Node(int domain_id);
    virtual ~Node();

    template <typename T>
    Publisher<T> *create_publisher(MessageType *message_type, const std::string &topic, const dds::TopicQos &qos)
    {
      auto publisher = std::make_unique<Publisher<T>>(participant_, message_type, std::string("rt/") + topic, qos);
      Publisher<T> *raw_ptr = publisher.get();
      publisher_list_.push_front(std::move(publisher));
      return raw_ptr;
    }

    template <typename T>
    Subscriber<T> *create_subscription(MessageType *message_type, const std::string &topic, const dds::TopicQos &qos,
                                       std::function<void(T *)> callback_function)
    {
      auto subscriber = std::make_unique<Subscriber<T>>(participant_, message_type, std::string("rt/") + topic, qos, callback_function, channel_);
      Subscriber<T> *raw_ptr = subscriber.get();
      subscription_list_.push_front(std::move(subscriber));
      return raw_ptr;
    }

    template <typename T>
    Timer<T> *create_timer(T period, std::function<void()> callback_function)
    {
      auto timer = std::make_unique<Timer<T>>(period, callback_function);
      Timer<T> *raw_ptr = timer.get();
      timer_list_.push_front(std::move(timer));
      return raw_ptr;
    }
    virtual void spin();
    virtual void spin_once();
    virtual void spin_some();
    virtual void stop_spin();
    virtual void shutdown();

  private:
    dds::DomainParticipant *participant_;
    std::forward_list<std::unique_ptr<IPublisher>> publisher_list_;
    std::forward_list<std::unique_ptr<ISubscriber>> subscription_list_;
    std::forward_list<std::unique_ptr<ITimer>> timer_list_;
    Channel<ISubscriptionCallback *> channel_;
  };

  // rcl_like_wrapper state
  bool ok(void);

  // Represents a node within the ROS-like communication graph, managing its lifecycle and communication capabilities.
  class RCLWNode : public Node
  {
  protected:
    bool rclw_node_stop_flag_; // Flag to signal when the node should stop its processing.
    std::mutex mutex_;         // Mutex for thread-safe access to the node.

  public:
    RCLWNode(int domain_number);
    virtual ~RCLWNode();
    virtual bool init(const std::string &config_file_path);     // Initializes the node with a configuration file.
    virtual void spin();                                        // Continuously processes messages.
    virtual void spin_some();                                   // Processes available messages without blocking.
    virtual void stop_spin();
    virtual void shutdown();
  };

  // Executor that manages and executes nodes in a single thread.
  class SingleThreadedExecutor
  {
  public:
    SingleThreadedExecutor();
    ~SingleThreadedExecutor();

    void add_node(Node *node_ptr);
    void remove_node(Node *node_ptr);
    void stop_spin();
    void spin();
    void spin_some();
    void shutdown();

  private:
    std::vector<Node *> nodes_; // List of nodes managed by the executor.
    std::mutex mutex_;          // Mutex for thread-safe access to the nodes list.
    bool running_;              // Indicates whether the executor is currently running.
  };

  // Executor that manages and executes nodes, each in its own thread, allowing for parallel processing.
  class MultiThreadedExecutor
  {
  public:
    MultiThreadedExecutor();
    ~MultiThreadedExecutor();

    void add_node(Node *node_ptr);
    void remove_node(Node *node_ptr);
    void stop_spin();
    void spin();
    void spin_some();
    void shutdown();

  private:
    std::vector<Node *> nodes_;        // List of nodes managed by the executor.
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
