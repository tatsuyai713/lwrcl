#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
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
  RCLWNode::RCLWNode(int domain_number) : Node(domain_number), rclw_node_stop_flag_(false)
  {
  }

  // Destructor for RCLWNode, ensuring node resources are cleaned up
  RCLWNode::~RCLWNode()
  {
    stop_spin();
  }

  // Main function to start node operations and handle stop requests
  void RCLWNode::spin()
  {
    // Start spinning the node
    while (!rclw_node_stop_flag_ && !global_stop_flag.load())
    {
      // Assuming `spin_some` is a function that spins the node for a short,
      // manageable duration and can be repeatedly called without blocking indefinitely.
      Node::spin_some();

      // Check frequently for stop flags to ensure the loop can exit promptly.
      // This sleep is not strictly necessary but can be used to prevent
      // the loop from consuming too much CPU. Adjust the duration as needed.
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  // Main function to start node operations
  void RCLWNode::spin_some()
  {
    Node::spin_some();
  }

  // Triggers a stop request for node operations
  void RCLWNode::stop_spin()
  {
    rclw_node_stop_flag_ = true;
  }

  void RCLWNode::shutdown()
  {
    stop_spin();
  }

  // Constructor for SingleThreadedExecutor to manage node execution
  SingleThreadedExecutor::SingleThreadedExecutor() : running_(false)
  {
    register_signal_handler(); // Setup signal handling for shutdown
  }

  // Destructor to ensure all nodes are properly stopped and resources are released
  SingleThreadedExecutor::~SingleThreadedExecutor()
  {
    stop_spin();
  }

  // Adds a node to the executor for management and execution
  void SingleThreadedExecutor::add_node(Node *node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (node_ptr != nullptr)
    {
      nodes_.push_back(node_ptr); // Valid node pointers are added to the list
    }
    else
    {
      std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
    }
  }

  // Removes a node from the executor's management
  void SingleThreadedExecutor::remove_node(Node *node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(node_ptr != nullptr){
      nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node_ptr), nodes_.end()); // Removes the specified node
    }
  }

  // Stops all nodes managed by the executor
  void SingleThreadedExecutor::stop_spin()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_)
    {
      running_ = false; // Signals all nodes to stop their operations
      for (auto &node_ptr : nodes_)
      {
        if (node_ptr)
        {
          node_ptr->stop_spin(); // Stops spinning for each node
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
          node_ptr->spin_some();
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(10)); // Reduces CPU usage
    }
  }

  void SingleThreadedExecutor::spin_some()
  {
    running_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto node_ptr : nodes_) // Executes spin_some for each managed node
    {
      if (node_ptr)
      {
        node_ptr->spin_some();
      }
      else
      {
        std::cerr << "node pointer is invalid!" << std::endl;
      }
    }
  }

  void SingleThreadedExecutor::shutdown()
  {
    stop_spin();
  }

  // Constructor: Initializes the executor with running state set to false and registers the signal handler.
  MultiThreadedExecutor::MultiThreadedExecutor() : running_(false)
  {
    register_signal_handler(); // Setup signal handling for shutdown
  }

  // Destructor: Ensures all managed threads are properly joined before destruction to avoid dangling threads.
  MultiThreadedExecutor::~MultiThreadedExecutor()
  {
    stop_spin(); // Stop all threads and ensure they are joined before object destruction.
  }

  // Adds a new node to the executor for management. Validates the node pointer before adding.
  void MultiThreadedExecutor::add_node(Node *node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_); // Lock for thread-safe access to nodes_ vector.
    if (node_ptr != nullptr)
    {
      nodes_.push_back(node_ptr); // Valid node pointers are added to the list for execution.
    }
    else
    {
      std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl; // Error handling for null pointers.
    }
  }

  // Removes a node from the executor's management, ensuring it no longer receives execution time.
  void MultiThreadedExecutor::remove_node(Node *node_ptr)
  {
    std::lock_guard<std::mutex> lock(mutex_);                                        // Lock for thread-safe modification of nodes_ vector.
    if(node_ptr != nullptr){
      nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node_ptr), nodes_.end()); // Erase the specified node from the vector.
    }
  }

  // Signals all threads to stop and waits for them to finish, ensuring a clean shutdown.
  void MultiThreadedExecutor::stop_spin()
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
            node_ptr->spin_some(); // Execute node-specific spinning logic.
          }
          std::this_thread::sleep_for(std::chrono::microseconds(10)); // Prevent busy waiting by sleeping briefly.
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

  void MultiThreadedExecutor::spin_some()
  {
    running_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto node_ptr : nodes_) // Executes spin_some for each managed node
    {
      if (node_ptr)
      {
        node_ptr->spin_some();
      }
      else
      {
        std::cerr << "node pointer is invalid!" << std::endl;
      }
    }
  }

  void MultiThreadedExecutor::shutdown()
  {
    stop_spin();
  }

  Time::Time() : nanoseconds_(0) {}
  Time::Time(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Time::Time(int32_t seconds, uint32_t nanoseconds) : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
  int64_t Time::nanoseconds() const { return nanoseconds_; }
  double Time::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }
  Time Time::operator+(const Duration &rhs) const { return Time(nanoseconds_ + rhs.nanoseconds()); }
  Time Time::operator-(const Duration &rhs) const { return Time(nanoseconds_ - rhs.nanoseconds()); }
  Duration Time::operator-(const Time &rhs) const { return Duration(nanoseconds_ - rhs.nanoseconds_); }
  bool Time::operator==(const Time &rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
  bool Time::operator!=(const Time &rhs) const { return !(*this == rhs); }
  bool Time::operator<(const Time &rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool Time::operator<=(const Time &rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }
  bool Time::operator>(const Time &rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool Time::operator>=(const Time &rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

  Duration::Duration() : nanoseconds_(0) {}
  Duration::Duration(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Duration::Duration(int32_t seconds, uint32_t nanoseconds) : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
  int64_t Duration::nanoseconds() const { return nanoseconds_; }
  double Duration::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }
  Duration Duration::operator+(const Duration &rhs) const { return Duration(nanoseconds_ + rhs.nanoseconds()); }
  Duration Duration::operator-(const Duration &rhs) const { return Duration(nanoseconds_ - rhs.nanoseconds()); }
  bool Duration::operator==(const Duration &rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
  bool Duration::operator!=(const Duration &rhs) const { return !(*this == rhs); }
  bool Duration::operator<(const Duration &rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool Duration::operator<=(const Duration &rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }
  bool Duration::operator>(const Duration &rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool Duration::operator>=(const Duration &rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

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

  Node::Node(int domain_id)
  {
    register_signal_handler(); // Setup signal handling for shutdown
    participant_ = eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT);
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
  }

  Node::~Node()
  {
    publisher_list_.clear();
    subscription_list_.clear();
    timer_list_.clear();
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
  }

  void Node::spin()
  {
    while (!channel_.is_closed() && !global_stop_flag.load())
    {
      ISubscriptionCallback *callback;
      while (channel_.consume_nowait(callback))
      {
        if (callback)
        {
          callback->invoke();
        }
      }

      auto now = std::chrono::steady_clock::now();
      for (auto &timer : timer_list_)
      {
        timer->check_and_call();
      }

      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    channel_.close();
  }

  void Node::spin_once()
  {
    ISubscriptionCallback *callback;
    if (channel_.consume_nowait(callback))
    {
      callback->invoke();
    }

    for (auto &timer : timer_list_)
    {
      timer->check_and_call();
    }
  }

  void Node::spin_some()
  {
    bool event_processed = false;

    do
    {
      event_processed = false;

      ISubscriptionCallback *callback;
      while (channel_.consume_nowait(callback))
      {
        callback->invoke();
        event_processed = true;
      }

      for (auto &timer : timer_list_)
      {
        timer->check_and_call();
      }
    } while (event_processed);
  }

  void Node::stop_spin()
  {
    channel_.close();
  }

  void Node::shutdown()
  {
    channel_.close();
  }

  bool ok()
  {
    return !global_stop_flag.load();
  }

} // namespace rcl_like_wrapper
