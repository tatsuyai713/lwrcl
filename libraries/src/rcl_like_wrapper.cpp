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

// Begin namespace for the rcl_like_wrapper functionality
namespace rcl_like_wrapper
{
  // Global flag to control the stopping of the application, e.g., in response to SIGINT
  std::atomic_bool global_stop_flag{false};

  // Mutex to ensure thread-safe access to global variables
  std::mutex global_mutex;

  // Function to handle SIGINT signals for graceful application termination
  void signal_handler(int signal)
  {
    if (signal == SIGINT || signal == SIGTERM)
    {
      std::lock_guard<std::mutex> lock(global_mutex);
      global_stop_flag = true;
    }
  }

  // Registers the above signal_handler to respond to SIGINT and SIGTERM signals
  void register_signal_handler()
  {
    if (std::signal(SIGINT, signal_handler) == SIG_ERR ||
        std::signal(SIGTERM, signal_handler) == SIG_ERR)
    {
      throw std::runtime_error("Failed to set signal handler.");
    }
  }

  // Constructs an RCLWNode object, initializing it and setting up signal handling
  RCLWNode::RCLWNode(uint16_t domain_number) : node_ptr_(0), rclw_node_stop_flag_(false)
  {
    register_signal_handler(); // Setup signal handling for graceful shutdown

    // Initializes the node and related entities like publishers and timers
    node_ptr_ = create_node(domain_number);
    if (!node_ptr_)
    {
      std::cerr << "Error: Failed to create a node." << std::endl;
      return;
    }
  }

  // Destructor for RCLWNode, ensuring node resources are cleaned up
  RCLWNode::~RCLWNode()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (node_ptr_ != 0)
    {
      destroy_node(node_ptr_); // Clean up node resources
      node_ptr_ = 0;
    }
  }

  // Main function to start node operations and handle stop requests
  void RCLWNode::spin()
  {
    if (node_ptr_ != 0)
    {
      // Start spinning the node
      while (!rclw_node_stop_flag_ && !global_stop_flag.load())
      {
        // Assuming `spin_some` is a function that spins the node for a short,
        // manageable duration and can be repeatedly called without blocking indefinitely.
        rcl_like_wrapper::spin_some(node_ptr_);

        // Check frequently for stop flags to ensure the loop can exit promptly.
        // This sleep is not strictly necessary but can be used to prevent
        // the loop from consuming too much CPU. Adjust the duration as needed.
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }

      // Once the loop exits, either because `rclw_node_stop_flag_` or `global_stop_flag` was set,
      // ensure we call `stop_spin` to cleanly stop the node's operations.
      stop_spin(node_ptr_);
    }
  }

  // Main function to start node operations
  void RCLWNode::spin_some()
  {
    if (node_ptr_ != 0)
    {
      rcl_like_wrapper::spin_some(node_ptr_);
    }
  }

  // Triggers a stop request for node operations
  void RCLWNode::stop()
  {
    rclw_node_stop_flag_ = true;
  }

  // Retrieves the raw pointer to the underlying node object
  intptr_t RCLWNode::get_node_pointer()
  {
    return node_ptr_ ? node_ptr_ : 0;
  }

  // Constructor for SingleThreadedExecutor to manage node execution
  SingleThreadedExecutor::SingleThreadedExecutor() : running_(false)
  {
    register_signal_handler(); // Setup to handle SIGINT for graceful shutdown
  }

  // Destructor to ensure all nodes are properly stopped and resources are released
  SingleThreadedExecutor::~SingleThreadedExecutor()
  {
    stop(); // Stops all managed nodes
  }

  // Adds a node to the executor for management and execution
  void SingleThreadedExecutor::add_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (node_ptr != 0)
    {
      nodes_.push_back(node_ptr); // Valid node pointers are added to the list
    }
    else
    {
      std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
    }
  }

  // Removes a node from the executor's management
  void SingleThreadedExecutor::remove_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node_ptr), nodes_.end()); // Removes the specified node
  }

  // Stops all nodes managed by the executor
  void SingleThreadedExecutor::stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
    {
      running_ = false; // Signals all nodes to stop their operations
      for (auto &node_ptr : nodes_)
      {
        if (node_ptr)
        {
          stop_spin(node_ptr); // Stops spinning for each node
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
    }
  }

  // Starts execution of all nodes managed by the executor
  void SingleThreadedExecutor::spin()
  {
    running_ = true;
    while (running_ && !global_stop_flag.load()) // Checks for global stop flag
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node_ptr : nodes_) // Executes spin_some for each managed node
      {
        if (node_ptr)
        {
          spin_some(node_ptr);
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(10)); // Reduces CPU usage
    }
  }

  // Constructor: Initializes the executor with running state set to false and registers the signal handler.
  MultiThreadedExecutor::MultiThreadedExecutor() : running_(false)
  {
    register_signal_handler(); // Register to handle SIGINT signals for graceful shutdown.
  }

  // Destructor: Ensures all managed threads are properly joined before destruction to avoid dangling threads.
  MultiThreadedExecutor::~MultiThreadedExecutor()
  {
    stop(); // Stop all threads and ensure they are joined before object destruction.
  }

  // Adds a new node to the executor for management. Validates the node pointer before adding.
  void MultiThreadedExecutor::add_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_); // Lock for thread-safe access to nodes_ vector.
    if (node_ptr != 0)
    {
      nodes_.push_back(node_ptr); // Valid node pointers are added to the list for execution.
    }
    else
    {
      std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl; // Error handling for null pointers.
    }
  }

  // Removes a node from the executor's management, ensuring it no longer receives execution time.
  void MultiThreadedExecutor::remove_node(intptr_t node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);                                        // Lock for thread-safe modification of nodes_ vector.
    nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node_ptr), nodes_.end()); // Erase the specified node from the vector.
  }

  // Signals all threads to stop and waits for them to finish, ensuring a clean shutdown.
  void MultiThreadedExecutor::stop()
  {
    running_ = false; // Set running flag to false to signal threads to stop.
    for (auto &thread : threads_)
    {
      if (thread.joinable())
      {
        thread.join(); // Wait for thread to finish its execution.
      }
    }
    threads_.clear(); // Clear the list of threads once all have been joined.
  }

  // Starts the execution of all nodes in separate threads, allowing for parallel processing.
  void MultiThreadedExecutor::spin()
  {
    running_ = true; // Set running flag to true to start threads.
    for (auto node_ptr : nodes_)
    {
      threads_.emplace_back([this, node_ptr]() { // Create a new thread for each node.
        while (this->running_ && !global_stop_flag.load())
        { // Continue execution until stop is signaled.
          if (!node_ptr)
          {
            std::cerr << "node pointer is invalid!" << std::endl; // Error handling for invalid node pointers.
          }
          else
          {
            spin_some(node_ptr); // Execute node-specific spinning logic.
          }
          std::this_thread::sleep_for(std::chrono::microseconds(1)); // Prevent busy waiting by sleeping briefly.
        }
      });
    }

    for (auto &thread : threads_)
    {
      if (thread.joinable())
      {
        thread.join(); // Ensure all threads are finished before exiting spin method.
      }
    }
  }

  // Time implementation
  Time::Time() : nanoseconds_(0) {}
  Time::Time(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Time::Time(int32_t seconds, uint32_t nanoseconds) : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
  int64_t Time::nanoseconds() const { return nanoseconds_; }
  double Time::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }

  // Duration implementation
  Duration::Duration() : nanoseconds_(0) {}
  Duration::Duration(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Duration::Duration(int32_t seconds, uint32_t nanoseconds) : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
  int64_t Duration::nanoseconds() const { return nanoseconds_; }
  double Duration::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }

  // Clock implementation
  Clock::Clock(ClockType type) : type_(type) {}
  Time Clock::now()
  {
    switch (type_)
    {
    case ClockType::SYSTEM_TIME:
      return Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count());
    default:
      throw std::runtime_error("Unsupported clock type.");
    }
  }
  Clock::ClockType Clock::get_clock_type() const { return type_; }

  // Rate implementation
  Rate::Rate(const Duration &period) : period_(period), next_time_(std::chrono::steady_clock::now() + std::chrono::nanoseconds(period.nanoseconds())) {}
  void Rate::sleep()
  {
    auto now = std::chrono::steady_clock::now();
    if (now >= next_time_)
    {
      auto periods_missed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_time_) / std::chrono::nanoseconds(period_.nanoseconds()) + 1;
      next_time_ += periods_missed * std::chrono::nanoseconds(period_.nanoseconds());
    }
    std::this_thread::sleep_until(next_time_);
    next_time_ += std::chrono::nanoseconds(period_.nanoseconds());
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
    auto node = new Node(domain_id);
    if (!node)
    {
      return 0;
    }
    return reinterpret_cast<intptr_t>(node);
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
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return;
    }
    // Casts the integer pointer back to a Node pointer and spins it to process messages
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin();
  }

  // Spins a node once, processing at least one message if available
  void spin_once(intptr_t node_ptr)
  {
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return;
    }
    // Similar to spin, but only processes messages once
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin_once();
  }

  // Processes some available messages without blocking
  void spin_some(intptr_t node_ptr)
  {
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return;
    }
    // Processes available messages without blocking the node
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->spin_some();
  }

  // Stops the spinning of a node
  void stop_spin(intptr_t node_ptr)
  {
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return;
    }
    // Instructs a node to stop its processing cycle
    auto node = reinterpret_cast<Node *>(node_ptr);
    node->stop_spin();
  }

  // Creates a publisher for a node
  intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos &qos)
  {
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return 0;
    }
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
    if (!publisher_ptr)
    {
      std::cerr << "publisher pointer is invalid!" << std::endl;
      return;
    }
    // Casts the integer pointer back to a Publisher pointer and publishes the message
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    publisher->publish(message);
  }

  // Retrieves the count of subscribers for a given publisher
  int32_t get_subscriber_count(intptr_t publisher_ptr)
  {
    if (!publisher_ptr)
    {
      std::cerr << "publisher pointer is invalid!" << std::endl;
      return 0;
    }
    // Returns the number of subscribers connected to the publisher
    auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
    return publisher->get_subscriber_count();
  }

  // Creates a subscription for a node
  intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos &qos, std::function<void(void *)> callback)
  {
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return 0;
    }
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
    if (!subscriber_ptr)
    {
      std::cerr << "subscriber pointer is invalid!" << std::endl;
      return 0;
    }
    // Returns the number of publishers to which the subscriber is connected
    auto subscriber = reinterpret_cast<Subscriber *>(subscriber_ptr);
    return subscriber->get_publisher_count();
  }

  // Creates a timer for a node
  template <typename Duration>
  intptr_t create_timer(intptr_t node_ptr, Duration period, std::function<void()> callback)
  {
    if (!node_ptr)
    {
      std::cerr << "node pointer is invalid!" << std::endl;
      return 0;
    }

    // Attempts to create a timer with a specific period and callback function
    auto node = reinterpret_cast<Node *>(node_ptr);

    auto timer = node->create_timer(std::chrono::duration_cast<std::chrono::microseconds>(period), callback);

    if (!timer)
    {
      return 0; // Returns 0 if the timer could not be created
    }

    return reinterpret_cast<intptr_t>(timer);
  }
  template intptr_t rcl_like_wrapper::create_timer<std::chrono::nanoseconds>(intptr_t, std::chrono::nanoseconds, std::function<void()>);
  template intptr_t rcl_like_wrapper::create_timer<std::chrono::microseconds>(intptr_t, std::chrono::microseconds, std::function<void()>);
  template intptr_t rcl_like_wrapper::create_timer<std::chrono::milliseconds>(intptr_t, std::chrono::milliseconds, std::function<void()>);
  template intptr_t rcl_like_wrapper::create_timer<std::chrono::seconds>(intptr_t, std::chrono::seconds, std::function<void()>);

  // Initializes the wrapper with a set of message types
  void rcl_like_wrapper_init(const MessageTypes &types)
  {
    // Initializes the global message types with the provided set
    for (const auto &type : types)
    {
      // Check if the type is already registered
      if (message_types.find(type.first) == message_types.end())
      {
        // If not registered, add the new type
        message_types[type.first] = type.second;
      }
      // If the type is already registered, do nothing
    }
  }

} // namespace rcl_like_wrapper
