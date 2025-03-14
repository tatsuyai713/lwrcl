#ifndef LWRCL_HPP_
#define LWRCL_HPP_

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstring>
#include <forward_list>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

#include "clock_time_duration.hpp"
#include "fast_dds_header.hpp"
#include "publisher.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace lwrcl
{

// Signal handler
extern void lwrcl_signal_handler(int signal);
class Logger;
class Node;
class IService;
template <typename T>
class Service;
class IClient;
template <typename T>
class Client;
class Parameter;

// Define a parameter class
typedef std::unordered_map<std::string, Parameter> Parameters;
typedef std::unordered_map<std::string, Parameters> NodeParameters;
// Global variable for parameters
extern NodeParameters node_parameters;
enum LogLevel { DEBUG, INFO, WARN, ERROR };
void log(LogLevel level, const char * format, ...);

// lwrcl functions
bool ok(void);
void spin(std::shared_ptr<lwrcl::Node> node);

void init(int argc, char * argv[]);
void shutdown(void);
void sleep_for(const lwrcl::Duration & duration);
void spin_some(std::shared_ptr<lwrcl::Node> node);
// Load parameters from file
std::string get_params_file_path(int argc, char * argv[]);
void load_parameters(const std::string & file_path);

// Base class for different parameter types

class ParameterBase
{
public:
  virtual ~ParameterBase() = default;
  virtual std::string get_name() const = 0;
  virtual std::string as_string() const = 0;
};

// Parameter class for different types
class Parameter : public ParameterBase
{
public:
  // Constructor for bool
  Parameter(const std::string & name, bool value);
  // Constructor for int
  Parameter(const std::string & name, int value);
  // Constructor for double
  Parameter(const std::string & name, double value);
  // Constructor for std::string
  Parameter(const std::string & name, const std::string & value);
  // Constructor for const char*
  Parameter(const std::string & name, const char * value);
  // Constructor for bool array
  Parameter(const std::string & name, const std::vector<bool> & value);
  // Constructor for int array
  Parameter(const std::string & name, const std::vector<int> & value);
  // Constructor for double array
  Parameter(const std::string & name, const std::vector<double> & value);
  // Constructor for std::string array
  Parameter(const std::string & name, const std::vector<std::string> & value);
  // Constructor for const char* array
  Parameter(const std::string & name, std::vector<const char *> & value);
  // Constructor for Byte array
  Parameter(const std::string & name, const std::vector<uint8_t> & value);
  Parameter();
  ~Parameter() = default;

  std::string get_name() const override;

  bool as_bool() const;
  int as_int() const;
  double as_double() const;
  std::string as_string() const;
  std::vector<bool> as_bool_array() const;
  std::vector<int> as_integer_array() const;
  std::vector<double> as_double_array() const;
  std::vector<std::string> as_string_array() const;
  std::vector<uint8_t> as_byte_array() const;

private:
  enum class Type {
    BOOL,
    INT,
    DOUBLE,
    STRING,
    BOOL_ARRAY,
    INT_ARRAY,
    DOUBLE_ARRAY,
    STRING_ARRAY,
    BYTE_ARRAY,
    UNKNOWN
  };

  std::string name_;
  std::string string_value_;
  Type type_;

  // Convert int to string
  std::string int_to_string(int value);
  // Convert double to string
  std::string double_to_string(double value);

  // Convert vector to string
  template <typename T>
  static std::string vector_to_string(const std::vector<T> & vec)
  {
    std::ostringstream oss;
    for (size_t i = 0; i < vec.size(); ++i) {
      if (i > 0) {
        oss << ",";
      }
      oss << vec[i];
    }
    return oss.str();
  }

  // Convert string to vector
  template <typename T>
  static std::vector<T> string_to_vector(const std::string & str)
  {
    std::vector<T> vec;
    std::istringstream iss(str);
    std::string item;
    while (std::getline(iss, item, ',')) {
      std::istringstream converter(item);
      T value;
      converter >> value;
      vec.push_back(value);
    }
    return vec;
  }
};

class Node : public std::enable_shared_from_this<Node>
{
public:
  using SharedPtr = std::shared_ptr<Node>;

  // Getters
  std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> get_participant() const;
  std::string get_name() const;
  Logger get_logger() const;

  // Create publisher, subscription, service, client, timer
  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(const std::string & topic, const uint16_t & depth)
  {
    QoS qos(depth);
    auto publisher =
      std::make_shared<Publisher<T>>(participant_.get(), std::string("rt/") + topic, qos);
    publisher_list_.push_front(publisher);
    return publisher;
  }

  template <typename T>
  std::shared_ptr<Publisher<T>> create_publisher(const std::string & topic, const QoS & qos)
  {
    auto publisher =
      std::make_shared<Publisher<T>>(participant_.get(), std::string("rt/") + topic, qos);
    publisher_list_.push_front(publisher);
    return publisher;
  }

  template <typename T>
  std::shared_ptr<Subscription<T>> create_subscription(
    const std::string & topic, const uint16_t & depth,
    std::function<void(std::shared_ptr<T>)> callback_function)
  {
    QoS qos(depth);
    auto subscription = std::make_shared<Subscription<T>>(
      participant_.get(), std::string("rt/") + topic, qos, callback_function, channel_);
    subscription_list_.push_front(subscription);
    return subscription;
  }

  template <typename T>
  std::shared_ptr<Subscription<T>> create_subscription(
    const std::string & topic, const QoS & qos,
    std::function<void(std::shared_ptr<T>)> callback_function)
  {
    auto subscription = std::make_shared<Subscription<T>>(
      participant_.get(), std::string("rt/") + topic, qos, callback_function, channel_);
    subscription_list_.push_front(subscription);
    return subscription;
  }

  template <typename T>
  std::shared_ptr<Service<T>> create_service(
    const std::string & service_name,
    std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
      callback_function)
  {
    std::shared_ptr<Service<T>> service =
      std::make_shared<Service<T>>(participant_.get(), service_name, callback_function, channel_);
    service_list_.push_front(service);

    return service;
  }

  template <typename T>
  std::shared_ptr<Client<T>> create_client(const std::string & service_name)
  {
    std::shared_ptr<Client<T>> client =
      std::make_shared<Client<T>>(participant_.get(), service_name, channel_);
    client_list_.push_front(client);

    return client;
  }

  template <typename Rep, typename Period>
  std::shared_ptr<TimerBase> create_timer(
    std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
  {
    lwrcl::Clock::ClockType clock_type = Clock::ClockType::SYSTEM_TIME;
    auto duration = Duration(period);
    auto timer = std::make_shared<TimerBase>(duration, callback_function, channel_, clock_type);
    timer_list_.push_front(timer);
    return timer;
  }

  template <typename Rep, typename Period>
  std::shared_ptr<TimerBase> create_wall_timer(
    std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
  {
    lwrcl::Clock::ClockType clock_type = Clock::ClockType::STEADY_TIME;
    auto duration = Duration(period);
    auto timer = std::make_shared<TimerBase>(duration, callback_function, channel_, clock_type);
    timer_list_.push_front(timer);
    return timer;
  }

  // Create node
  static std::shared_ptr<Node> make_shared(int domain_id);
  static std::shared_ptr<Node> make_shared(int domain_id, const std::string & name);
  static std::shared_ptr<Node> make_shared(const std::string & name);
  static std::shared_ptr<Node> make_shared(
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
  static std::shared_ptr<Node> make_shared(
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant,
    const std::string & name);

  // Friend functions
  friend void lwrcl::spin(std::shared_ptr<Node> node);
  friend void lwrcl::spin_some(std::shared_ptr<Node> node);

  // Node functions
  virtual void shutdown();
  virtual Clock::SharedPtr get_clock();

  // Constructor
  Node(std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant);
  Node(
    std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant,
    const std::string & name);
  // Destructor
  virtual ~Node();

  // Parameter functions
  void set_parameters(const std::vector<std::shared_ptr<ParameterBase>> & parameters);
  void set_parameters(const std::vector<Parameter> & parameters);
  void declare_parameter(const std::string & name, const bool & default_value);
  void declare_parameter(const std::string & name, const int & default_value);
  void declare_parameter(const std::string & name, const double & default_value);
  void declare_parameter(const std::string & name, const std::string & default_value);
  void declare_parameter(const std::string & name, const char * default_value);
  void declare_parameter(const std::string & name, const std::vector<bool> default_value);
  void declare_parameter(const std::string & name, const std::vector<int> default_value);
  void declare_parameter(const std::string & name, const std::vector<double> default_value);
  void declare_parameter(const std::string & name, const std::vector<std::string> default_value);
  void declare_parameter(const std::string & name, const std::vector<uint8_t> default_value);
  Parameter get_parameter(const std::string & name) const;
  void get_parameter(const std::string & name, bool & bool_data) const;
  void get_parameter(const std::string & name, int & int_data) const;
  void get_parameter(const std::string & name, double & double_data) const;
  void get_parameter(const std::string & name, std::string & string_data) const;

  bool closed_;
  void stop_spin();

private:
  virtual void spin();
  virtual void spin_some();

protected:
  // Protected constructor
  Node(int domain_id);
  Node(int domain_id, const std::string & name);
  Node(const std::string & name);

private:
  // Deleter for DomainParticipant
  struct DomainParticipantDeleter
  {
    void operator()(eprosima::fastdds::dds::DomainParticipant * participant) const
    {
      if (participant != nullptr) {
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(
          participant);
      }
    }
  };

  // Member variables
  eprosima::fastdds::dds::DomainParticipantFactory * factory_;
  std::shared_ptr<eprosima::fastdds::dds::DomainParticipant> participant_;
  Channel<ChannelCallback *>::SharedPtr channel_;
  Clock::SharedPtr clock_;
  std::string name_;
  bool stop_flag_;
  bool participant_owned_;

  std::forward_list<std::shared_ptr<IPublisher>> publisher_list_;
  std::forward_list<std::shared_ptr<ISubscription>> subscription_list_;
  std::forward_list<std::shared_ptr<ITimerBase>> timer_list_;
  std::forward_list<std::shared_ptr<IService>> service_list_;
  std::forward_list<std::shared_ptr<IClient>> client_list_;
  Parameters parameters_;
};

// Executor classes
namespace executors
{
// Executor that manages and executes nodes in a single thread.
class SingleThreadedExecutor
{
public:
  SingleThreadedExecutor();
  ~SingleThreadedExecutor();

  void add_node(Node::SharedPtr node);
  void remove_node(Node::SharedPtr node);
  void cancel();
  void clear();
  void spin();
  void spin_some();

private:
  std::vector<Node::SharedPtr> nodes_;  // List of nodes managed by the executor.
  mutable std::mutex mutex_;            // Mutex for thread-safe access to the nodes list.
  bool stop_flag_;                      // Flag to stop the executor.
};

// Executor that manages and executes nodes, each in its own thread, allowing for parallel processing.
class MultiThreadedExecutor
{
public:
  MultiThreadedExecutor();
  ~MultiThreadedExecutor();

  void add_node(Node::SharedPtr node);
  void remove_node(Node::SharedPtr node);
  void cancel();
  void clear();
  void spin();
  void spin_some();
  int get_number_of_threads() const;

private:
  std::vector<Node::SharedPtr> nodes_;  // List of nodes managed by the executor.
  std::vector<std::thread> threads_;    // Threads created for each node for parallel execution.
  mutable std::mutex mutex_;            // Mutex for thread-safe access to the nodes list.
  bool stop_flag_;                      // Flag to stop the executor.
};
}  // namespace executors

// Rate class
class Rate
{
private:
  Duration period_;
  std::chrono::system_clock::time_point next_time_;

public:
  explicit Rate(const Duration & period);
  void sleep();
};

class WallRate
{
private:
  Duration period_;
  std::chrono::steady_clock::time_point next_time_;

public:
  explicit WallRate(const Duration & period);
  void sleep();
};

// Logger Class
class Logger
{
public:
  Logger(const std::string & node_name);

  void log(LogLevel level, const char * format, ...) const;

private:
  std::string node_name_;
};

class IService
{
public:
  virtual ~IService() = default;
  virtual void stop() = 0;
};

template <typename T>
class Service : public IService, public std::enable_shared_from_this<Service<T>>
{
public:
  using SharedPtr = std::shared_ptr<Service>;

  Service(
    eprosima::fastdds::dds::DomainParticipant * participant, const std::string & service_name,
    std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
      callback_function,
    std::shared_ptr<Channel<ChannelCallback *>> channel)
  : participant_(participant),
    service_name_(service_name),
    callback_function_(callback_function),
    channel_(channel)
  {
    // Create topic
    request_topic_name_ = service_name_ + "_Request";
    response_topic_name_ = service_name_ + "_Response";

    RMWQoSProfile rmw_qos_profile_services = rmw_qos_profile_services_default;
    QoS service_qos(KeepLast(10), rmw_qos_profile_services);

    publisher_ = std::make_shared<Publisher<typename T::Response>>(
      participant_, std::string("rp/") + response_topic_name_, service_qos);

    request_callback_function_ = [this](std::shared_ptr<typename T::Request> request) {
      std::shared_ptr<typename T::Response> response = std::make_shared<typename T::Response>();
      callback_function_(request, response);
      publisher_->publish(response);
    };

    subscription_ = std::make_shared<Subscription<typename T::Request>>(
      participant_, std::string("rp/") + request_topic_name_, service_qos,
      request_callback_function_, channel_);
  }

  ~Service() {}  // Destructor

  void stop() override { subscription_->stop(); }

private:
  eprosima::fastdds::dds::DomainParticipant * participant_;
  std::string service_name_;
  std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
    callback_function_;
  std::function<void(std::shared_ptr<typename T::Request>)> request_callback_function_;
  std::shared_ptr<Publisher<typename T::Response>> publisher_;
  std::shared_ptr<Subscription<typename T::Request>> subscription_;
  std::string request_topic_name_;
  std::string response_topic_name_;
  Channel<ChannelCallback *>::SharedPtr channel_;
};

class IClient
{
public:
  virtual ~IClient() = default;
  virtual void stop() = 0;
};

enum FutureReturnCode { SUCCESS, INTERRUPTED, TIMEOUT };

class FutureBase
{
public:
  virtual ~FutureBase() = default;
  virtual std::future_status wait_for(std::chrono::milliseconds timeout) = 0;
};

template <typename T>
class TypedFuture : public FutureBase
{
public:
  TypedFuture(std::shared_future<std::shared_ptr<T>> future) : future_(future) {}

  std::future_status wait_for(std::chrono::milliseconds timeout) override
  {
    return future_.wait_for(timeout);
  }

private:
  std::shared_future<std::shared_ptr<T>> future_;
};

template <typename T>
class Client : public IClient, public std::enable_shared_from_this<Client<T>>
{
public:
  using SharedPtr = std::shared_ptr<Client>;

  Client(
    eprosima::fastdds::dds::DomainParticipant * participant, const std::string & service_name,
    std::shared_ptr<Channel<ChannelCallback *>> channel)
  : participant_(participant),
    service_name_(service_name),
    channel_(channel),
    response_(nullptr),
    response_callback_function_(nullptr)
  {
    // Create topic
    request_topic_name_ = service_name_ + "_Request";
    response_topic_name_ = service_name_ + "_Response";

    RMWQoSProfile rmw_qos_profile_services = rmw_qos_profile_services_default;
    QoS client_qos(KeepLast(10), rmw_qos_profile_services);

    publisher_ = std::make_shared<Publisher<typename T::Request>>(
      participant_, std::string("rp/") + request_topic_name_, client_qos);

    subscription_ = std::make_shared<Subscription<typename T::Response>>(
      participant_, std::string("rp/") + response_topic_name_, client_qos,
      std::function<void(std::shared_ptr<typename T::Response>)>(
        std::bind(&Client::handle_response, this, std::placeholders::_1)),
      channel_);
  }
  ~Client() {}  // Destructor

  void handle_response(std::shared_ptr<typename T::Response> response)
  {
    if (response_callback_function_) {
      response_callback_function_(response);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    response_ = response;
    response_received_ = true;

    cv_.notify_one();
  }

  void stop() override { subscription_->stop(); }

  std::shared_ptr<FutureBase> async_send_request(std::shared_ptr<typename T::Request> request)
  {
    auto promise = std::make_shared<std::promise<std::shared_ptr<typename T::Response>>>();
    auto future = promise->get_future().share();

    response_callback_function_ = [promise](std::shared_ptr<void> response) mutable {
      promise->set_value(std::static_pointer_cast<typename T::Response>(response));
    };

    publisher_->publish(request);

    return std::make_shared<TypedFuture<typename T::Response>>(future);
  }

  template <typename Duration>
  bool wait_for_service(const Duration & timeout)
  {
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point end_time = start_time + timeout;

    while (current_time < end_time) {
      if (publisher_->get_subscriber_count() > 0) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      current_time = std::chrono::system_clock::now();
    }

    return false;
  }

private:
  eprosima::fastdds::dds::DomainParticipant * participant_;
  std::string service_name_;
  std::shared_ptr<Publisher<typename T::Request>> publisher_;
  std::shared_ptr<Subscription<typename T::Response>> subscription_;
  std::string request_topic_name_;
  std::string response_topic_name_;
  std::function<void(std::shared_ptr<typename T::Response>)> response_callback_function_;
  std::shared_ptr<typename T::Response> response_;
  Channel<ChannelCallback *>::SharedPtr channel_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool response_received_ = false;
};  //

template <typename Duration>
FutureReturnCode spin_until_future_complete(
  std::shared_ptr<lwrcl::Node> node, std::shared_ptr<FutureBase> future, const Duration & timeout)
{
  std::thread spin_thread([node]() { lwrcl::spin(node); });

  if (
    future->wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout)) ==
    std::future_status::ready) {
    node->stop_spin();
    spin_thread.join();
    return SUCCESS;
  } else {
    node->stop_spin();
    spin_thread.join();
    return TIMEOUT;
  }
}

struct rsl_serialized_message_t
{
  char * buffer;
  size_t length;
};

class SerializedMessage
{
public:
  SerializedMessage()
  {
    data_.buffer = nullptr;
    data_.length = 0;
  }

  SerializedMessage(size_t initial_capacity)
  {
    data_.buffer = new char[initial_capacity];
    data_.length = initial_capacity;
  }

  SerializedMessage(const SerializedMessage & other)
  {
    data_.buffer = new char[other.data_.length];
    memcpy(data_.buffer, other.data_.buffer, other.data_.length);
    data_.length = other.data_.length;
  }

  SerializedMessage(const rsl_serialized_message_t & other)
  {
    data_.buffer = new char[other.length];
    memcpy(data_.buffer, other.buffer, other.length);
    data_.length = other.length;
  }

  SerializedMessage(SerializedMessage && other) noexcept
  {
    data_ = other.data_;
    other.data_.buffer = nullptr;
    other.data_.length = 0;
  }

  SerializedMessage(rsl_serialized_message_t && other) noexcept
  {
    data_ = other;
    other.buffer = nullptr;
    other.length = 0;
  }

  SerializedMessage & operator=(const SerializedMessage & other)
  {
    if (this != &other) {
      if (data_.buffer != nullptr) {
        delete[] data_.buffer;
      }
      data_.buffer = new char[other.data_.length];
      memcpy(data_.buffer, other.data_.buffer, other.data_.length);
      data_.length = other.data_.length;
    }
    return *this;
  }

  SerializedMessage & operator=(const rsl_serialized_message_t & other)
  {
    if (data_.buffer != other.buffer) {
      if (data_.buffer != nullptr) {
        delete[] data_.buffer;
      }
      data_.buffer = new char[other.length];
      memcpy(data_.buffer, other.buffer, other.length);
      data_.length = other.length;
    }
    return *this;
  }

  SerializedMessage & operator=(SerializedMessage && other) noexcept
  {
    if (this != &other) {
      if (data_.buffer != nullptr) {
        delete[] data_.buffer;
      }
      data_ = other.data_;
      other.data_.buffer = nullptr;
      other.data_.length = 0;
    }
    return *this;
  }

  SerializedMessage & operator=(rsl_serialized_message_t && other) noexcept
  {
    if (data_.buffer != other.buffer) {
      if (data_.buffer != nullptr) {
        delete[] data_.buffer;
      }
      data_ = other;
      other.buffer = nullptr;
      other.length = 0;
    }
    return *this;
  }

  virtual ~SerializedMessage()
  {
    if (data_.buffer != nullptr) {
      delete[] data_.buffer;
    }
  }

  rsl_serialized_message_t & get_rcl_serialized_message() { return data_; }

  const rsl_serialized_message_t & get_rcl_serialized_message() const { return data_; }

  size_t size() const { return data_.length; }

  size_t capacity() const { return data_.length; }

  void reserve(size_t new_capacity)
  {
    if (new_capacity > data_.length) {
      char * new_buffer = new char[new_capacity];
      if (data_.buffer != nullptr) {
        memcpy(new_buffer, data_.buffer, data_.length);
        delete[] data_.buffer;
      }
      data_.buffer = new_buffer;
    }
  }

  rsl_serialized_message_t release_rsl_serialized_message()
  {
    rsl_serialized_message_t temp = data_;
    data_.buffer = nullptr;
    data_.length = 0;
    return temp;
  }

private:
  rsl_serialized_message_t data_;
};

template <typename T>
class Serializarion
{
public:
  static void serialize_message(T * message, SerializedMessage * serialized_message)
  {
    eprosima::fastcdr::FastBuffer fastbuffer;
    eprosima::fastcdr::Cdr cdr(fastbuffer);
    uint8_t header[4] = {0x00, 0x01, 0x00, 0x00};
    message->serialize(cdr);
    const char * buffer = cdr.getBufferPointer();
    size_t length = cdr.getSerializedDataLength();
    serialized_message->reserve(length + 4);
    char * combined_buffer = serialized_message->get_rcl_serialized_message().buffer;
    memcpy(combined_buffer, header, 4);
    memcpy(combined_buffer + 4, buffer, length);
    serialized_message->get_rcl_serialized_message().length = length + 4;
  }

  static void deserialize_message(SerializedMessage * serialized_message, T * message)
  {
    eprosima::fastcdr::FastBuffer fastbuffer(
      reinterpret_cast<char *>(serialized_message->get_rcl_serialized_message().buffer + 4),
      serialized_message->get_rcl_serialized_message().length - 4);
    eprosima::fastcdr::Cdr cdr(fastbuffer);
    message->deserialize(cdr);
  }
};
}  // namespace lwrcl

#define LWRCL_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define LWRCL_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define LWRCL_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define LWRCL_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

#endif  // LWRCL_HPP_
