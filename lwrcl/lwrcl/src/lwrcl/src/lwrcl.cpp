#include <csignal>
#include <cstring>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>
#include <memory>
#include <chrono>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include "lwrcl.hpp" // The main header file for the lwrcl namespace

namespace lwrcl
{
  class Node;

  std::atomic<bool> global_stop_flag(false);

  // Function to handle SIGINT signals for graceful application termination
  void lwrcl_signal_handler(int signal)
  {
    if ((signal == SIGINT || signal == SIGTERM) && global_stop_flag.load() == false)
    {
      printf("SIGINT/SIGTERM received, shutting down...\n");
      global_stop_flag.store(true);
    }
  }

  // Constructor for bool
  Parameter::Parameter(const std::string &name, bool value) : name_(name), type_(Type::BOOL)
  {
    string_value_ = value ? "true" : "false";
  }

  // Constructor for int
  Parameter::Parameter(const std::string &name, int value) : name_(name), type_(Type::INT)
  {
    string_value_ = int_to_string(value);
  }

  // Constructor for double
  Parameter::Parameter(const std::string &name, double value) : name_(name), type_(Type::DOUBLE)
  {
    string_value_ = double_to_string(value);
  }

  // Constructor for std::string
  Parameter::Parameter(const std::string &name, const std::string &value) : name_(name), type_(Type::STRING)
  {
    string_value_ = value;
  }

  // Constructor for const char*
  Parameter::Parameter(const std::string &name, const char *value) : name_(name), type_(Type::STRING)
  {
    string_value_ = value;
  }

  // Constructor for bool array
  Parameter::Parameter(const std::string &name, const std::vector<bool> &value)
      : name_(name), type_(Type::BOOL_ARRAY)
  {
    string_value_ = vector_to_string(value);
  }

  // Constructor for int array
  Parameter::Parameter(const std::string &name, const std::vector<int> &value)
      : name_(name), type_(Type::INT_ARRAY)
  {
    string_value_ = vector_to_string(value);
  }

  // Constructor for double array
  Parameter::Parameter(const std::string &name, const std::vector<double> &value)
      : name_(name), type_(Type::DOUBLE_ARRAY)
  {
    string_value_ = vector_to_string(value);
  }

  // Constructor for std::string array
  Parameter::Parameter(const std::string &name, const std::vector<std::string> &value)
      : name_(name), type_(Type::STRING_ARRAY)
  {
    string_value_ = vector_to_string(value);
  }

  // Constructor for const char* array
  Parameter::Parameter(const std::string &name, std::vector<const char *> &value)
      : name_(name), type_(Type::STRING_ARRAY)
  {
    string_value_ = vector_to_string(value);
  }

  // Constructor for Byte array
  Parameter::Parameter(const std::string &name, const std::vector<uint8_t> &value)
      : name_(name), type_(Type::BYTE_ARRAY)
  {
    string_value_ = vector_to_string(value);
  }

  Parameter::Parameter() : type_(Type::UNKNOWN) {}

  // Get name
  std::string Parameter::get_name() const { return name_; }

  // Get value (specific to type)
  bool Parameter::as_bool() const
  {
    if (type_ != Type::BOOL)
    {
      throw std::runtime_error("Parameter is not a bool");
    }
    return string_value_ == "true";
  }

  int Parameter::as_int() const
  {
    if (type_ != Type::INT)
    {
      throw std::runtime_error("Parameter is not an int");
    }
    int int_value;
    std::istringstream iss(string_value_);
    iss >> int_value;
    return int_value;
  }
  double Parameter::as_double() const
  {
    if (type_ != Type::DOUBLE)
    {
      throw std::runtime_error("Parameter is not a double");
    }
    double double_value;
    std::istringstream iss(string_value_);
    iss >> double_value;
    return double_value;
  }
  std::string Parameter::as_string() const { return string_value_; }

  std::vector<bool> Parameter::as_bool_array() const
  {
    if (type_ != Type::BOOL_ARRAY)
    {
      throw std::runtime_error("Parameter is not a bool array");
    }
    return string_to_vector<bool>(string_value_);
  }

  std::vector<int> Parameter::as_integer_array() const
  {
    if (type_ != Type::INT_ARRAY)
    {
      throw std::runtime_error("Parameter is not an int array");
    }
    return string_to_vector<int>(string_value_);
  }

  std::vector<double> Parameter::as_double_array() const
  {
    if (type_ != Type::DOUBLE_ARRAY)
    {
      throw std::runtime_error("Parameter is not a double array");
    }
    return string_to_vector<double>(string_value_);
  }

  std::vector<std::string> Parameter::as_string_array() const
  {
    if (type_ != Type::STRING_ARRAY)
    {
      throw std::runtime_error("Parameter is not a string array");
    }
    return string_to_vector<std::string>(string_value_);
  }

  std::vector<uint8_t> Parameter::as_byte_array() const
  {
    if (type_ != Type::BYTE_ARRAY)
    {
      throw std::runtime_error("Parameter is not a string array");
    }
    return string_to_vector<uint8_t>(string_value_);
  }

  // Convert int to string
  std::string Parameter::int_to_string(int value)
  {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }

  // Convert double to string
  std::string Parameter::double_to_string(double value)
  {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }

  void log(LogLevel level, const char *format, ...)
  {
    va_list args;
    va_start(args, format);

    std::ostringstream msg;
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - now_sec);
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    char timestamp_buf[80];
    std::strftime(timestamp_buf, sizeof(timestamp_buf), "%s", std::localtime(&timestamp));

    switch (level)
    {
    case DEBUG:
      msg << "\033[0;36m[DEBUG]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case INFO:
      msg << "\033[0;37m[INFO]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case WARN:
      msg << "\033[0;33m[WARN]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case ERROR:
      msg << "\033[0;31m[ERROR]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    default:
      msg << "[Unknown log level] [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    }

    msg << ": ";
    std::cout << msg.str();
    vprintf(format, args);
    std::cout << std::endl;

    va_end(args);
  }

  Logger::Logger(const std::string &node_name) : node_name_(node_name) {}

  void Logger::log(LogLevel level, const char *format, ...) const
  {
    va_list args;
    va_start(args, format);

    std::ostringstream msg;
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - now_sec);
    auto timestamp = std::chrono::system_clock::to_time_t(now);
    char timestamp_buf[80];
    std::strftime(timestamp_buf, sizeof(timestamp_buf), "%s", std::localtime(&timestamp));

    switch (level)
    {
    case DEBUG:
      msg << "\033[0;36m[DEBUG]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case INFO:
      msg << "\033[0;37m[INFO]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case WARN:
      msg << "\033[0;33m[WARN]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    case ERROR:
      msg << "\033[0;31m[ERROR]\033[0m [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    default:
      msg << "[Unknown log level] [" << timestamp_buf << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
      break;
    }

    msg << "[" << node_name_ << "]: ";
    std::cout << msg.str();
    vprintf(format, args);
    std::cout << std::endl;

    va_end(args);
  }

  namespace executors
  {
    SingleThreadedExecutor::SingleThreadedExecutor() {}

    SingleThreadedExecutor::~SingleThreadedExecutor()
    {
      cancel();
    }

    void SingleThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.emplace_back(node);
      }
      else
      {
        std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
      }
    }

    void SingleThreadedExecutor::remove_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
      }
    }

    void SingleThreadedExecutor::cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto &node : nodes_)
      {
        if (node != nullptr)
        {
          if (node->closed_ == false)
          {
            node->shutdown();
          }
        }
      }
      nodes_.clear();
    }

    void SingleThreadedExecutor::spin()
    {
      bool exit_flag = false;
      while (global_stop_flag.load() == false && exit_flag == false)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto node : nodes_)
        {
          if (node != nullptr)
          {
            if (node->closed_ == false)
            {
              lwrcl::spin_some(node);
            }
            else
            {
              exit_flag = true;
            }
          }
          else
          {
            exit_flag = true;
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }

    void SingleThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node != nullptr)
        {
          if (node->closed_ == false)
          {
            lwrcl::spin_some(node);
          }
        }
      }
    }

    MultiThreadedExecutor::MultiThreadedExecutor() {}

    MultiThreadedExecutor::~MultiThreadedExecutor()
    {
      cancel();
    }

    void MultiThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.emplace_back(node);
      }
      else
      {
        std::cerr << "Error: Node pointer is null, cannot add to executor." << std::endl;
      }
    }

    void MultiThreadedExecutor::remove_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
      }
    }

    void MultiThreadedExecutor::cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node != nullptr)
        {
          if (node->closed_ == false)
          {
            node->shutdown();
          }
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
      nodes_.clear();
    }

    void MultiThreadedExecutor::spin()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        threads_.emplace_back([this, node]()
                              {
            if(node != nullptr)
            {
              if (node->closed_ == false)
              {
                  lwrcl::spin(node);
              }
            }
            else
            {
                std::cerr << "node pointer is invalid!" << std::endl;
            } });
      }

      for (auto &thread : threads_)
      {
        if (thread.joinable())
        {
          thread.join();
        }
      }
      threads_.clear();
    }

    void MultiThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node != nullptr)
        {
          if (node->closed_ == false)
          {
            lwrcl::spin_some(node);
          }
        }
        else
        {
          std::cerr << "node pointer is invalid!" << std::endl;
        }
      }
    }

    int MultiThreadedExecutor::get_number_of_threads() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return threads_.size();
    }
  } // namespace executors

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
    case ClockType::STEADY_TIME:
      return Time(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now().time_since_epoch())
                      .count());
    default:
      throw std::runtime_error("Unsupported clock type.");
    }
  }
  Clock::ClockType Clock::get_clock_type() const { return type_; }

  // Rate implementation
  Rate::Rate(const Duration &period) : period_(period)
  {
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::nanoseconds(period.nanoseconds());
    auto next_time_temp = now + duration;
    next_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(next_time_temp);
  }
  void Rate::sleep()
  {
    auto now = std::chrono::system_clock::now();
    if (now >= next_time_)
    {
      auto periods_missed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_time_) /
                                std::chrono::nanoseconds(period_.nanoseconds()) +
                            1;
      auto duration_to_add =
          std::chrono::nanoseconds(static_cast<long long>(periods_missed) * period_.nanoseconds());
      auto next_time_temp = next_time_ + duration_to_add;
      next_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(next_time_temp);
    }
    std::this_thread::sleep_until(next_time_);

    auto duration_to_add = std::chrono::nanoseconds(period_.nanoseconds());
    auto next_time_temp = next_time_ + duration_to_add;
    next_time_ = std::chrono::time_point_cast<std::chrono::system_clock::duration>(next_time_temp);
  }

  WallRate::WallRate(const Duration &period) : period_(period), next_time_(std::chrono::steady_clock::now() + std::chrono::nanoseconds(period.nanoseconds())) {}
  void WallRate::sleep()
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
      : participant_(nullptr),
        channel_(std::make_shared<Channel<ChannelCallback *>>()),
        clock_(std::make_unique<Clock>()),
        name_("lwrcl_default_node")
  {
    dds::DomainParticipantQos participant_qos = dds::PARTICIPANT_QOS_DEFAULT;

    // Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 4194304;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;

    // Link the Transport Layer to the Participant.
    participant_qos.transport().user_transports.emplace_back(udp_transport);

    // Increase the sending buffer size
    participant_qos.transport().send_socket_buffer_size = 4194304;
    // Increase the receiving buffer size
    participant_qos.transport().listen_socket_buffer_size = 4194304;

    // eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    auto participant_factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    participant_factory->load_XML_profiles_file("/opt/fast-dds/fastdds.xml");

    participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
        participant_factory->create_participant(domain_id, participant_qos),
        DomainParticipantDeleter());
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    closed_ = false;
  }

  Node::Node(int domain_id, const std::string &name)
      : participant_(nullptr),
        channel_(std::make_shared<Channel<ChannelCallback *>>()),
        clock_(std::make_unique<Clock>()),
        name_(name)
  {
    dds::DomainParticipantQos participant_qos = dds::PARTICIPANT_QOS_DEFAULT;

    // Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 4194304;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;

    // Link the Transport Layer to the Participant.
    participant_qos.transport().user_transports.emplace_back(udp_transport);

    // Increase the sending buffer size
    participant_qos.transport().send_socket_buffer_size = 4194304;
    // Increase the receiving buffer size
    participant_qos.transport().listen_socket_buffer_size = 4194304;

    // eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    auto participant_factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    participant_factory->load_XML_profiles_file("/opt/fast-dds/fastdds.xml");

    participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
        participant_factory->create_participant(domain_id, participant_qos),
        DomainParticipantDeleter());
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    closed_ = false;
  }

  Node::Node(const std::string &name)
      : participant_(nullptr),
        channel_(std::make_shared<Channel<ChannelCallback *>>()),
        clock_(std::make_unique<Clock>()),
        name_(name)
  {
    int domain_id = 0; // Default domain ID
    dds::DomainParticipantQos participant_qos = dds::PARTICIPANT_QOS_DEFAULT;

    // Create a descriptor for the new transport.
    auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
    udp_transport->sendBufferSize = 4194304;
    udp_transport->receiveBufferSize = 4194304;
    udp_transport->non_blocking_send = true;

    // Link the Transport Layer to the Participant.
    participant_qos.transport().user_transports.emplace_back(udp_transport);

    // Increase the sending buffer size
    participant_qos.transport().send_socket_buffer_size = 4194304;
    // Increase the receiving buffer size
    participant_qos.transport().listen_socket_buffer_size = 4194304;

    // eprosima::fastdds::dds::Log::SetVerbosity(eprosima::fastdds::dds::Log::Info);

    auto participant_factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
    participant_factory->load_XML_profiles_file("/opt/fast-dds/fastdds.xml");

    participant_ = std::shared_ptr<eprosima::fastdds::dds::DomainParticipant>(
        participant_factory->create_participant(domain_id, participant_qos),
        DomainParticipantDeleter());
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    closed_ = false;
  }

  Node::Node(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant)
      : participant_(participant),
        channel_(std::make_shared<Channel<ChannelCallback *>>()),
        clock_(std::make_unique<Clock>()),
        name_("lwrcl_default_node")
  {
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    closed_ = false;
  }

  Node::Node(
      std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant, const std::string &name)
      : participant_(participant),
        channel_(std::make_shared<Channel<ChannelCallback *>>()),
        clock_(std::make_unique<Clock>()),
        name_(name)
  {
    if (!participant_)
    {
      throw std::runtime_error("Failed to create domain participant");
    }
    closed_ = false;
  }

  Node::~Node()
  {
    if (closed_ == false)
    {
      this->shutdown();
    }
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id)
  {
    auto node = std::shared_ptr<Node>(new Node(domain_id), [](Node *node)
                                      { delete node; });
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name)
  {
    auto node = std::shared_ptr<Node>(new Node(domain_id, name), [](Node *node)
                                      { delete node; });
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(const std::string &name)
  {
    auto node = std::shared_ptr<Node>(new Node(name), [](Node *node)
                                      { delete node; });
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant)
  {
    auto node = std::shared_ptr<Node>(new Node(participant), [](Node *node)
                                      { delete node; });
    return node;
  }
  std::shared_ptr<Node> Node::make_shared(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant, const std::string &name)
  {
    auto node = std::shared_ptr<Node>(new Node(participant, name), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> Node::get_participant() const
  {
    return participant_;
  }

  std::string Node::get_name() const
  {
    return name_;
  }

  Logger Node::get_logger() const
  {
    return Logger(name_);
  }

  void Node::spin()
  {
    stop_flag_ = false;
    while (closed_ == false && global_stop_flag.load() == false && stop_flag_ == false)
    {
      ChannelCallback *callback;
      while (channel_->consume(callback) && global_stop_flag.load() == false && stop_flag_ == false)
      {
        if (callback)
        {
          callback->invoke();
        }
        else
        {
          break;
        }
      }
    }
  }

  void Node::stop_spin()
  {
    stop_flag_ = true;
  }

  void Node::spin_some()
  {
    bool event_processed = false;

    do
    {
      event_processed = false;

      ChannelCallback *callback;
      while (channel_->consume_nowait(callback))
      {
        callback->invoke();
        event_processed = true;
      }
    } while (event_processed);
  }

  void Node::shutdown()
  {
    printf("Shutting down %s node...\n", name_.c_str());
    publisher_list_.clear();
    for (auto &subscriber : subscription_list_)
    {
      std::static_pointer_cast<ISubscription>(subscriber)->stop();
    }
    subscription_list_.clear();
    for (auto &timer : timer_list_)
    {
      std::static_pointer_cast<TimerBase>(timer)->stop();
    }
    timer_list_.clear();

    for (auto &service : service_list_)
    {
      std::static_pointer_cast<IService>(service)->stop();
    }
    service_list_.clear();
    for (auto &client : client_list_)
    {
      std::static_pointer_cast<IClient>(client)->stop();
    }
    closed_ = true;
  }

  void Node::set_parameters(const std::vector<std::shared_ptr<ParameterBase>> &parameters)
  {
    for (const auto &param : parameters)
    {
      std::string node_name = this->get_name();
      std::string param_name = param->get_name();

      // Check if the node exists in node_parameters
      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        Parameters &params = node_it->second;

        // Check if the parameter exists in the node_parameters for this node
        if (params.find(param_name) != params.end())
        {
          // Update the existing parameter
          params[param_name] = *std::dynamic_pointer_cast<Parameter>(param);
          parameters_[param_name] = *std::dynamic_pointer_cast<Parameter>(param);

          std::cout << "Parameter updated: " << param_name << std::endl;
        }
        else
        {
          std::cerr << "Parameter not found: " << param_name << std::endl;
        }
      }
      else
      {
        std::cerr << "Node not found: " << node_name << std::endl;
      }
    }
  }

  void Node::set_parameters(const std::vector<Parameter> &parameters)
  {
    std::vector<std::shared_ptr<ParameterBase>> base_params;
    for (const auto &param : parameters)
    {
      base_params.push_back(std::make_shared<Parameter>(param));
    }
    set_parameters(base_params);
  }

  void Node::declare_parameter(const std::string &name, const bool &default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const int &default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const double &default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const std::string &default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const char *default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const std::vector<bool> default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const std::vector<int> default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const std::vector<double> default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const std::vector<std::string> default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  void Node::declare_parameter(const std::string &name, const std::vector<uint8_t> default_value)
  {
    std::string node_name = this->get_name();

    auto node_it = node_parameters.find(node_name);
    if (node_it != node_parameters.end())
    {
      const Parameters &params = node_it->second;
      auto param_it = params.find(name);
      if (param_it != params.end())
      {
        Parameter param_value = param_it->second;
        parameters_[name] = param_value;
        node_parameters[node_name][name] = param_value;
        return;
      }
    }

    parameters_[name] = Parameter(name, default_value);
    node_parameters[node_name][name] = Parameter(name, default_value);
  }

  Parameter Node::get_parameter(const std::string &name) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
    {
      return it->second;
    }
    else
    {
      throw std::runtime_error("Parameter not found");
    }
  }

  void Node::get_parameter(const std::string &name, bool &bool_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
    {
      Parameter param = it->second;
      bool_data = param.as_bool();
    }
    else
    {
      throw std::runtime_error("Parameter not found");
    }
  }

  void Node::get_parameter(const std::string &name, int &int_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
    {
      Parameter param = it->second;
      int_data = param.as_int();
    }
    else
    {
      throw std::runtime_error("Parameter not found");
    }
  }

  void Node::get_parameter(const std::string &name, double &double_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
    {
      Parameter param = it->second;
      double_data = param.as_double();
    }
    else
    {
      throw std::runtime_error("Parameter not found");
    }
  }

  void Node::get_parameter(const std::string &name, std::string &string_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
    {
      Parameter param = it->second;
      string_data = param.as_string();
    }
    else
    {
      throw std::runtime_error("Parameter not found");
    }
  }

  Clock::SharedPtr Node::get_clock() const
  {
    return clock_;
  }

  bool ok()
  {
    if (!global_stop_flag.load())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void spin(std::shared_ptr<Node> node)
  {
    if (node != nullptr)
    {
      if (node->closed_ == false)
      {
        node->spin();
      }
    }
    else
    {
      std::cerr << "node pointer is invalid!" << std::endl;
    }
  }

  void spin_some(std::shared_ptr<Node> node)
  {
    if (node != nullptr)
    {
      if (node->closed_ == false)
      {
        node->spin_some();
      }
    }
    else
    {
      std::cerr << "node pointer is invalid!" << std::endl;
    }
  }

  std::string get_params_file_path(int argc, char *argv[])
  {
    if (argc < 2)
    {
      return "";
    }

    bool found_ros_args = false;

    for (int i = 1; i < argc; ++i)
    {
      if (std::string(argv[i]) == "--ros-args")
      {
        found_ros_args = true;
      }
      else if (found_ros_args && std::string(argv[i]) == "--param-file")
      {
        if (i + 1 < argc)
        {
          return std::string(argv[i + 1]);
        }
        else
        {
          return "";
        }
      }
    }

    return "";
  }

  NodeParameters node_parameters;

  void load_parameters(const std::string &file_path)
  {
    YAML::Node config = YAML::LoadFile(file_path);

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      std::string node_name = it->first.as<std::string>();
      YAML::Node parameters = it->second["ros__parameters"];

      Parameters params;
      for (YAML::const_iterator param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
      {
        std::string param_name = param_it->first.as<std::string>();
        auto param_value = param_it->second;
        try
        {
          param_value.as<int>();
          int value = param_value.as<int>();
          params[param_name] = Parameter(param_name, value);
        }
        catch (const YAML::BadConversion &)
        {
          try
          {
            param_value.as<double>();
            double value = param_value.as<double>();
            params[param_name] = Parameter(param_name, value);
          }
          catch (const YAML::BadConversion &)
          {
            try
            {
              param_value.as<bool>();
              bool value = param_value.as<bool>();
              params[param_name] = Parameter(param_name, value);
            }
            catch (const YAML::BadConversion &)
            {
              std::string value = param_value.as<std::string>();
              params[param_name] = Parameter(param_name, value);
            }
          }
        }
      }
      node_parameters[node_name] = params;
    }
  }

  void init(int argc, char *argv[])
  {
    if (std::signal(SIGINT, lwrcl_signal_handler) == SIG_ERR || std::signal(SIGTERM, lwrcl_signal_handler) == SIG_ERR)
    {
      throw std::runtime_error("Failed to set signal handler.");
    }

    try
    {
      std::string params_file_path = get_params_file_path(argc, argv);

      if (!params_file_path.empty())
      {
        std::cout << "Loading parameters from file: " << params_file_path << std::endl;
        load_parameters(params_file_path);
      }
    }
    catch (const YAML::Exception &e)
    {
      std::cerr << "Error parsing YAML file: " << e.what() << std::endl;
      return;
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << std::endl;
      return;
    }
  }

  void shutdown()
  {
    global_stop_flag.store(true);
  }

  void sleep_for(const lwrcl::Duration &duration)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(duration.nanoseconds()));
  }

  // Default QoS profile
  const RMWQoSProfile rmw_qos_profile_default = {
      10, // depth
      RMWQoSHistoryPolicy::KEEP_LAST,
      RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_sensor_data = {
      5, // depth
      RMWQoSHistoryPolicy::KEEP_LAST,
      RMWQoSReliabilityPolicy::BEST_EFFORT,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_parameters = {
      1000, // depth
      RMWQoSHistoryPolicy::KEEP_LAST,
      RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_services_default = {
      10, // depth
      RMWQoSHistoryPolicy::KEEP_LAST,
      RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_parameter_events = {
      1000, // depth
      RMWQoSHistoryPolicy::KEEP_LAST,
      RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  QoSInitialization KeepLast(size_t depth)
  {
    return QoSInitialization(RMWQoSHistoryPolicy::KEEP_LAST, depth);
  }

  QoSInitialization KeepAll()
  {
    return QoSInitialization(RMWQoSHistoryPolicy::KEEP_ALL, 1000);
  }
} // namespace lwrcl
