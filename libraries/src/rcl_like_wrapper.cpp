#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include "node.hpp" 
#include "rcl_like_wrapper.hpp" // The main header file for the rcl_like_wrapper namespace

namespace rcl_like_wrapper
{
  // Global flag to indicate when the application should stop, e.g., on SIGINT
  std::atomic_bool global_stop_flag{false};

  // Mutex for safe access to global variables across threads
  std::mutex global_mutex;

  // Handles SIGINT signals to gracefully stop the application
  void signal_handler(int signal)
  {
    if (signal == SIGINT)
    {
      std::lock_guard<std::mutex> lock(global_mutex);
      global_stop_flag = true; // Set the global flag to true on Ctrl+C
    }
  }

  // Registers the signal handler for SIGINT
  void register_signal_handler()
  {
    std::signal(SIGINT, signal_handler); // Register the handler for SIGINT
    std::signal(SIGTERM, signal_handler); // Register the handler for SIGTERM
  }

  // Constructor for RCLWNode initializes the node and registers the signal handler
  RCLWNode::RCLWNode() : domain_number_(0), node_ptr_(0), rclw_node_stop_flag_(false)
  {
    register_signal_handler(); // Ensure we handle SIGINT for graceful shutdown
  }

  RCLWNode::~RCLWNode()
  {
    rclw_node_stop_flag_ = true;
    if (node_ptr_ != 0)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto node = reinterpret_cast<Node *>(node_ptr_);
      node->destroy();
      node_ptr_ = 0; // Reset pointer after destruction
    }
  }

  // Main spin function for the node
  void RCLWNode::spin()
  {
    if (node_ptr_ != 0)
    {
      std::thread spin_thread([this]()
                              { rcl_like_wrapper::spin(node_ptr_); });

      // Loop until a stop is requested via stop flag or global stop flag
      while (!rclw_node_stop_flag_ && !global_stop_flag.load())
      {
        std::this_thread::sleep_for(std::chrono::seconds(3)); // Prevent busy waiting
      }

      // If stop is requested, ensure the node's spinning is stopped
      if (node_ptr_ != 0)
      {
        stop_spin(node_ptr_);
      }
      spin_thread.join(); // Wait for the spin thread to finish

      // Clean up the node
      if (node_ptr_ != 0)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto node = reinterpret_cast<Node *>(node_ptr_);
        node->destroy();
        node_ptr_ = 0; // Reset pointer after destruction
      }
    }
  }

  // Request stop spinning the node
  void RCLWNode::stop()
  {
    rclw_node_stop_flag_ = true;
  }

  // Get the raw pointer to the node
  intptr_t RCLWNode::get_node_pointer()
  {
    return node_ptr_ ? node_ptr_ : 0;
  }

  // Executor to manage and spin multiple nodes
  Executor::Executor() : running_(false)
  {
    register_signal_handler(); // Also handle SIGINT for the executor
  }

  Executor::~Executor()
  {
    stop(); // Ensure all nodes are stopped on destruction
  }

  // Add a node to the executor for management
  void Executor::add_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (node_ptr != 0)
    {
      nodes_.push_back(node_ptr); // Add the node if it's valid
    }
    else
    {
      std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
    }
  }

  // Remove a node from the executor
  void Executor::remove_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node_ptr), nodes_.end()); // Remove the node
  }

  // Stop all nodes managed by the executor
  void Executor::stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false; // Set the running flag to false
  }

  // Start spinning all nodes managed by the executor
  void Executor::spin()
  {
    running_ = true;
    while (running_ && !global_stop_flag.load())
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto node_ptr : nodes_) // Spin each node
        {
          spin_some(node_ptr);
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(1)); // Prevent busy waiting
    }
  }

  // Rate control mechanism
  Rate::Rate(std::chrono::milliseconds period) : period_(period)
  {
    start_time_ = std::chrono::steady_clock::now(); // Initialize the start time
  }

  Rate::~Rate()
  {
  }

  // Sleeps for the remainder of the rate period
  void Rate::sleep()
  {
    auto now = std::chrono::steady_clock::now();
    if (start_time_ + period_ > now)
    {
      std::this_thread::sleep_until(start_time_ + period_); // Sleep until the next period
    }
    start_time_ += period_; // Update the start time for the next period
  }

  // Constructor for managing message type support
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
      type_support = other.type_support; // Copy the type support
    }
    return *this;
  }

  MessageType::~MessageType()
  {
  }

  // Global storage for message types available in the system
  MessageTypes message_types;

  // Creates a new node within a specified domain
  intptr_t create_node(uint16_t domain_id)
  {
    // Dynamically allocates a new Node object with the given domain ID and returns its pointer as an integer
    return reinterpret_cast<intptr_t>(new Node(domain_id));
  }

  // Destroys a previously created node
  void destroy_node(intptr_t node_ptr)
  {
    // Casts the integer pointer back to a Node pointer and calls its destroy method
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->destroy();
  }

  // Initiates the spin cycle of a node, making it process messages
  void spin(intptr_t node_ptr)
  {
    // Casts the integer pointer back to a Node pointer and spins it to process messages
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin();
  }

  // Spins a node once, processing at least one message if available
  void spin_once(intptr_t node_ptr)
  {
    // Similar to spin, but only processes messages once
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin_once();
  }

  // Processes some available messages without blocking
  void spin_some(intptr_t node_ptr)
  {
    // Processes available messages without blocking the node
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin_some();
  }

  // Stops the spinning of a node
  void stop_spin(intptr_t node_ptr)
  {
    // Instructs a node to stop its processing cycle
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->stop_spin();
  }

  // Creates a publisher for a node
  intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos &qos)
  {
    // Attempts to create a publisher for a specific message type and topic
    auto node = reinterpret_cast<Node *>(node_ptr);

    if (message_types.find(message_type_name) == message_types.end())
    {
      return 0; // Returns 0 if the message type is not found
    }

    auto publisher = node->create_publisher(message_types.at(message_type_name), std::string("rt/") + topic, qos);

    if (!publisher)
    {
      return 0; // Returns 0 if the publisher could not be created
    }

    return reinterpret_cast<intptr_t>(publisher);
  }

  // Publishes a message through a specific publisher
  void publish(intptr_t publisher_ptr, void *message)
  {
    // Casts the integer pointer back to a Publisher pointer and publishes the message
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    publisher->publish(message);
  }

  // Retrieves the count of subscribers for a given publisher
  int32_t get_subscriber_count(intptr_t publisher_ptr)
  {
    // Returns the number of subscribers connected to the publisher
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    return publisher->get_subscriber_count();
  }

  // Destroys a previously created publisher
  void destroy_publisher(intptr_t publisher_ptr)
  {
    // Casts the integer pointer back to a Publisher pointer and destroys it
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    if (publisher)
    {
      publisher->destroy();
    }
  }

  // Creates a subscription for a node
  intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos &qos, std::function<void(void *)> callback)
  {
    // Attempts to create a subscription for a specific message type and topic with a callback
    auto node = reinterpret_cast<Node *>(node_ptr);

    if (message_types.find(message_type_name) == message_types.end())
    {
      return 0; // Returns 0 if the message type is not found
    }

    MessageType &message_type = message_types.at(message_type_name);

    auto subscriber = node->create_subscription(message_type, std::string("rt/") + topic, qos, callback);

    if (!subscriber)
    {
      return 0; // Returns 0 if the subscriber could not be created
    }

    return reinterpret_cast<intptr_t>(subscriber);
  }

  // Retrieves the count of publishers for a given subscriber
  int32_t get_publisher_count(intptr_t subscriber_ptr)
  {
    // Returns the number of publishers to which the subscriber is connected
    auto subscriber = reinterpret_cast<Subscriber *>(subscriber_ptr);
    return subscriber->get_publisher_count();
  }

  // Destroys a previously created subscription
  void destroy_subscription(intptr_t subscriber_ptr)
  {
    // Casts the integer pointer back to a Subscriber pointer and destroys it
    auto subscriber = reinterpret_cast<Subscriber *>(subscriber_ptr);
    if (subscriber)
    {
      subscriber->destroy();
    }
  }

  // Creates a timer for a node
  intptr_t create_timer(intptr_t node_ptr, std::chrono::milliseconds period, std::function<void()> callback)
  {
    // Attempts to create a timer with a specific period and callback function
    auto node = reinterpret_cast<Node *>(node_ptr);

    auto timer = node->create_timer(period, callback);

    if (!timer)
    {
      return 0; // Returns 0 if the timer could not be created
    }

    return reinterpret_cast<intptr_t>(timer);
  }

  // Destroys a previously created timer
  void destroy_timer(intptr_t timer_ptr)
  {
    // Casts the integer pointer back to a Timer pointer and destroys it
    auto timer = reinterpret_cast<Timer *>(timer_ptr);
    if (timer)
    {
      timer->destroy();
    }
  }

  // Initializes the wrapper with a set of message types
  void rcl_like_wrapper_init(const MessageTypes &types)
  {
    // Initializes the global message types with the provided set
    message_types = types;
  }

} // namespace rcl_like_wrapper
