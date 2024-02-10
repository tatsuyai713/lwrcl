#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include "node.hpp"
#include "qos.hpp"
#include "rcl_like_wrapper.hpp"

namespace rcl_like_wrapper
{
  std::atomic_bool global_stop_flag{false}; // グローバル停止フラグ

  void signal_handler(int signal)
  {
    if (signal == SIGINT)
    {
      global_stop_flag = true; // Ctrl+Cが押されたらフラグをセット
    }
  }

  void register_signal_handler()
  {
    std::signal(SIGINT, signal_handler); // シグナルハンドラを登録
  }

  RCLWNode::RCLWNode() : domain_number_(0), node_ptr_(0), rclw_node_stop_flag_(0)
  {
    register_signal_handler();
  }

  void RCLWNode::spin()
  {
    if (node_ptr_ != 0)
    {
      std::thread spin_thread([this]()
                              { rcl_like_wrapper::spin(node_ptr_); });

      while (!rclw_node_stop_flag_ && !global_stop_flag.load())
      {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }

      if (node_ptr_ != 0)
      {
        stop_spin(node_ptr_);
      }
      spin_thread.join();

      if (node_ptr_ != 0)
      {
        auto node = reinterpret_cast<Node *>(node_ptr_);
        node->destroy();
        node_ptr_ = 0;
      }
    }
  }

  void RCLWNode::stop()
  {
    rclw_node_stop_flag_ = false;
  }

  intptr_t RCLWNode::get_node_pointer()
  {
    if (node_ptr_ != 0)
    {
      return node_ptr_;
    }
    return 0;
  }

  Executor::Executor() : running_(true)
  {
    register_signal_handler();
  }

  Executor::~Executor()
  {
    stop();
  }

  void Executor::add_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_.push_back(node_ptr);
  }

  void Executor::remove_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node_ptr), nodes_.end());
  }

  void Executor::stop()
  {
    running_ = true;
  }

  void Executor::spin()
  {
    while (running_ && !global_stop_flag.load())
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto node_ptr : nodes_)
        {
          spin_some(node_ptr);
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    running_ = false;

    std::lock_guard<std::mutex> lock(mutex_);
    for (auto node_ptr : nodes_)
    {
      auto node = reinterpret_cast<Node *>(node_ptr);
      node->destroy();
    }
  }

  Rate::Rate(std::chrono::milliseconds period) : period_(period)
  {
    start_time_ = std::chrono::steady_clock::now();
  }

  Rate::~Rate()
  {
  }

  void Rate::sleep()
  {
    auto now = std::chrono::steady_clock::now();
    if (start_time_ + period_ > now)
    {
      std::this_thread::sleep_until(start_time_ + period_);
    }
    start_time_ += period_; // 次の周期の開始時刻を更新
  }

  MessageType::MessageType(eprosima::fastdds::dds::TopicDataType *message_type)
      : type_support(message_type)
  {
  }

  MessageType::MessageType(const MessageType &other) : type_support(other.type_support)
  {
  }

  MessageType::MessageType() : type_support(nullptr)
  {
  }

  MessageType &MessageType::operator=(const MessageType &other)
  {
    if (this != &other)
    {
      type_support = other.type_support;
    }
    return *this;
  }

  MessageType::~MessageType()
  {
  }

  MessageTypes message_types;

  intptr_t create_node(uint16_t domain_id)
  {
    return reinterpret_cast<intptr_t>(new Node(domain_id));
  }

  void destroy_node(intptr_t node_ptr)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->destroy();
  }

  void spin(intptr_t node_ptr)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin();
  }

  void spin_once(intptr_t node_ptr)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin_once();
  }

  void spin_some(intptr_t node_ptr)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin_some();
  }

  void stop_spin(intptr_t node_ptr)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->stop_spin();
  }

  intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos &qos)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);

    if (message_types.find(message_type_name) == message_types.end())
    {
      return 0; // Handle error: Message type not found
    }

    auto publisher = node->create_publisher(message_types.at(message_type_name), std::string("rt/") + topic, qos);

    if (!publisher)
    {
      return 0; // Handle error: Publisher creation failed
    }

    return reinterpret_cast<intptr_t>(publisher);
  }

  void publish(intptr_t publisher_ptr, void *message)
  {
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    publisher->publish(message);
  }

  int32_t get_subscriber_count(intptr_t publisher_ptr)
  {
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    return publisher->get_subscriber_count();
  }

  void destroy_publisher(intptr_t publisher_ptr)
  {
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    if (publisher)
    {
      publisher->destroy();
    }
  }

  intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos &qos, std::function<void(void *)> callback)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);

    if (message_types.find(message_type_name) == message_types.end())
    {
      return 0; // Handle error: Message type not found
    }

    MessageType &message_type = message_types.at(message_type_name);

    auto subscriber = node->create_subscription(message_type, std::string("rt/") + topic, qos, [callback](void *message_data)
                                                { callback(message_data); });

    if (!subscriber)
    {
      return 0; // Handle error: Subscriber creation failed
    }

    return reinterpret_cast<intptr_t>(subscriber);
  }

  int32_t get_publisher_count(intptr_t subscriber_ptr)
  {
    auto subscriber = reinterpret_cast<Subscriber *>(subscriber_ptr);
    return subscriber->get_publisher_count();
  }

  void destroy_subscription(intptr_t subscriber_ptr)
  {
    auto subscriber = reinterpret_cast<Subscriber *>(subscriber_ptr);
    if (subscriber)
    {
      subscriber->destroy();
    }
  }

  intptr_t create_timer(intptr_t node_ptr, std::chrono::milliseconds period, std::function<void()> callback)
  {
    auto node = reinterpret_cast<Node *>(node_ptr);

    auto timer = node->create_timer(period, callback);

    if (!timer)
    {
      return 0; // Handle error: Timer creation failed
    }

    return reinterpret_cast<intptr_t>(timer);
  }

  void destroy_timer(intptr_t timer_ptr)
  {
    auto timer = reinterpret_cast<Timer *>(timer_ptr);
    if (timer)
    {
      timer->destroy();
    }
  }

  void rcl_like_wrapper_init(const MessageTypes &types)
  {
    message_types = types;
  }

} // namespace rcl_like_wrapper
