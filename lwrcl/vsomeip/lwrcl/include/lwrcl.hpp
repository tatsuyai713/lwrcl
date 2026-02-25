#ifndef LWRCL_HPP_
#define LWRCL_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <forward_list>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <cstring>

#include <vsomeip/vsomeip.hpp>
#include "org/eclipse/cyclonedds/core/cdr/basic_cdr_ser.hpp"
#include "qos.hpp"
#include "clock_time_duration.hpp"
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

  // lwrcl functions
  bool ok(void);
  void spin(std::shared_ptr<lwrcl::Node> node);
  void init(int argc, char *argv[]);
  void shutdown(void);
  void sleep_for(const lwrcl::Duration &duration);
  void spin_some(std::shared_ptr<lwrcl::Node> node);

  // For vsomeip backend, DomainParticipant maps to vsomeip::application
  using DomainParticipant = vsomeip::application;

  class ParameterBase
  {
  public:
    virtual ~ParameterBase() = default;
    virtual std::string get_name() const = 0;
    virtual std::string as_string() const = 0;

  protected:
    ParameterBase() = default;
  };

  class Parameter : public ParameterBase
  {
  public:
    Parameter(const std::string &name, bool value);
    Parameter(const std::string &name, int value);
    Parameter(const std::string &name, double value);
    Parameter(const std::string &name, const std::string &value);
    Parameter(const std::string &name, const char *value);
    Parameter(const std::string &name, const std::vector<bool> &value);
    Parameter(const std::string &name, const std::vector<int> &value);
    Parameter(const std::string &name, const std::vector<double> &value);
    Parameter(const std::string &name, const std::vector<std::string> &value);
    Parameter(const std::string &name, std::vector<const char *> &value);
    Parameter(const std::string &name, const std::vector<uint8_t> &value);
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
    enum class Type
    {
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

    std::string int_to_string(int value);
    std::string double_to_string(double value);

    template <typename T>
    static std::string vector_to_string(const std::vector<T> &vec)
    {
      std::ostringstream oss;
      for (size_t i = 0; i < vec.size(); ++i)
      {
        if (i > 0)
        {
          oss << ",";
        }
        oss << vec[i];
      }
      return oss.str();
    }

    template <typename T>
    static std::vector<T> string_to_vector(const std::string &str)
    {
      std::vector<T> vec;
      std::istringstream iss(str);
      std::string item;
      while (std::getline(iss, item, ','))
      {
        std::istringstream converter(item);
        T value{};
        converter >> value;
        vec.push_back(value);
      }
      return vec;
    }
  };

  typedef std::unordered_map<std::string, Parameter> Parameters;
  typedef std::unordered_map<std::string, Parameters> NodeParameters;
  extern NodeParameters node_parameters;

  std::string get_params_file_path(int argc, char *argv[]);
  void load_parameters(const std::string &file_path);

  class Node : public std::enable_shared_from_this<Node>
  {
  public:
    using SharedPtr = std::shared_ptr<Node>;

    struct NodeOptions
    {
      bool use_intra_process_comms = false;
      bool start_parameter_services = true;
      bool start_parameter_event_publisher = true;
      bool allow_undeclared_parameters = false;
      bool automatically_declare_parameters_from_overrides = false;

      NodeOptions &set_use_intra_process_comms(bool value)
      {
        use_intra_process_comms = value;
        return *this;
      }

      NodeOptions &set_allow_undeclared_parameters(bool value)
      {
        allow_undeclared_parameters = value;
        return *this;
      }
    };

    // Getters
    std::shared_ptr<vsomeip::application> get_participant() const;
    std::string get_name() const;
    std::string get_namespace() const;
    std::string get_fully_qualified_name() const;
    const NodeOptions &get_node_options() const;
    Logger get_logger() const;

    // Create publisher
    template <typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic, const uint16_t &depth)
    {
      QoS qos(depth);
      auto publisher =
          std::make_shared<Publisher<T>>(app_, resolve_topic_name(topic), qos);
      publisher_list_.push_front(publisher);
      return publisher;
    }

    template <typename T>
    std::shared_ptr<Publisher<T>> create_publisher(const std::string &topic, const QoS &qos)
    {
      auto publisher =
          std::make_shared<Publisher<T>>(app_, resolve_topic_name(topic), qos);
      publisher_list_.push_front(publisher);
      return publisher;
    }

    // Create subscription
    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(
        const std::string &topic, const uint16_t &depth,
        std::function<void(std::shared_ptr<T>)> callback_function)
    {
      QoS qos(depth);
      auto subscription = std::make_shared<Subscription<T>>(
          app_, resolve_topic_name(topic), qos, std::move(callback_function), callback_mutex_);
      subscription_list_.push_front(subscription);
      return subscription;
    }

    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(
        const std::string &topic, const QoS &qos,
        std::function<void(std::shared_ptr<T>)> callback_function)
    {
      auto subscription = std::make_shared<Subscription<T>>(
          app_, resolve_topic_name(topic), qos, std::move(callback_function), callback_mutex_);
      subscription_list_.push_front(subscription);
      return subscription;
    }

    // Const-ref callback overloads
    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(
        const std::string &topic, const uint16_t &depth,
        std::function<void(const T &)> callback_function)
    {
      auto wrapper = [cb = std::move(callback_function)](std::shared_ptr<T> msg)
      {
        cb(*msg);
      };
      return create_subscription<T>(topic, depth, std::function<void(std::shared_ptr<T>)>(std::move(wrapper)));
    }

    template <typename T>
    std::shared_ptr<Subscription<T>> create_subscription(
        const std::string &topic, const QoS &qos,
        std::function<void(const T &)> callback_function)
    {
      auto wrapper = [cb = std::move(callback_function)](std::shared_ptr<T> msg)
      {
        cb(*msg);
      };
      return create_subscription<T>(topic, qos, std::function<void(std::shared_ptr<T>)>(std::move(wrapper)));
    }

    // Create service
    template <typename T>
    std::shared_ptr<Service<T>> create_service(
        const std::string &service_name,
        std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
            callback_function)
    {
      std::shared_ptr<Service<T>> service =
          std::make_shared<Service<T>>(app_, service_name, std::move(callback_function), callback_mutex_);
      service_list_.push_front(service);
      return service;
    }

    // Create client
    template <typename T>
    std::shared_ptr<Client<T>> create_client(const std::string &service_name)
    {
      std::shared_ptr<Client<T>> client =
          std::make_shared<Client<T>>(app_, service_name, callback_mutex_);
      client_list_.push_front(client);
      return client;
    }

    // Create timer
    template <typename Rep, typename Period>
    std::shared_ptr<TimerBase> create_timer(
        std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
    {
      lwrcl::Clock::ClockType clock_type = Clock::ClockType::SYSTEM_TIME;
      auto duration = Duration(period);
      auto timer = std::make_shared<TimerBase>(duration, std::move(callback_function), callback_mutex_, clock_type);
      timer_list_.push_front(timer);
      return timer;
    }

    template <typename Rep, typename Period>
    std::shared_ptr<TimerBase> create_wall_timer(
        std::chrono::duration<Rep, Period> period, std::function<void()> callback_function)
    {
      lwrcl::Clock::ClockType clock_type = Clock::ClockType::STEADY_TIME;
      auto duration = Duration(period);
      auto timer = std::make_shared<TimerBase>(duration, std::move(callback_function), callback_mutex_, clock_type);
      timer_list_.push_front(timer);
      return timer;
    }

    // Create node
    static std::shared_ptr<Node> make_shared(int domain_id);
    static std::shared_ptr<Node> make_shared(int domain_id, const std::string &name);
    static std::shared_ptr<Node> make_shared(int domain_id, const std::string &name, const std::string &ns);
    static std::shared_ptr<Node> make_shared(int domain_id, const std::string &name, const std::string &ns, const NodeOptions &options);
    static std::shared_ptr<Node> make_shared(const std::string &name);
    static std::shared_ptr<Node> make_shared(const std::string &name, const std::string &ns);
    static std::shared_ptr<Node> make_shared(const std::string &name, const std::string &ns, const NodeOptions &options);
    static std::shared_ptr<Node> make_shared(
        std::shared_ptr<vsomeip::application> app);
    static std::shared_ptr<Node> make_shared(
        std::shared_ptr<vsomeip::application> app,
        const std::string &name);
    static std::shared_ptr<Node> make_shared(
        std::shared_ptr<vsomeip::application> app,
        const std::string &name, const std::string &ns);

    // Friend functions
    friend void lwrcl::spin(std::shared_ptr<Node> node);
    friend void lwrcl::spin_some(std::shared_ptr<Node> node);

    // Node functions
    virtual void shutdown();
    virtual Clock::SharedPtr get_clock();
    Time now() { return get_clock()->now(); }

    // Constructors
    Node(std::shared_ptr<vsomeip::application> app);
    Node(std::shared_ptr<vsomeip::application> app, const std::string &name);
    Node(std::shared_ptr<vsomeip::application> app, const std::string &name, const std::string &ns);
    virtual ~Node();
    Node(const Node &) = delete;
    Node &operator=(const Node &) = delete;
    Node(Node &&) = default;
    Node &operator=(Node &&) = default;

    void set_parameters(const std::vector<std::shared_ptr<ParameterBase>> &parameters);
    void set_parameters(const std::vector<Parameter> &parameters);
    void declare_parameter(const std::string &name, const bool &default_value);
    void declare_parameter(const std::string &name, const int &default_value);
    void declare_parameter(const std::string &name, const double &default_value);
    void declare_parameter(const std::string &name, const std::string &default_value);
    void declare_parameter(const std::string &name, const char *default_value);
    void declare_parameter(const std::string &name, const std::vector<bool> default_value);
    void declare_parameter(const std::string &name, const std::vector<int> default_value);
    void declare_parameter(const std::string &name, const std::vector<double> default_value);
    void declare_parameter(const std::string &name, const std::vector<std::string> default_value);
    void declare_parameter(const std::string &name, const std::vector<uint8_t> default_value);
    Parameter get_parameter(const std::string &name) const;
    void get_parameter(const std::string &name, bool &bool_data) const;
    void get_parameter(const std::string &name, int &int_data) const;
    void get_parameter(const std::string &name, double &double_data) const;
    void get_parameter(const std::string &name, std::string &string_data) const;

    bool has_parameter(const std::string &name) const {
      return parameters_.find(name) != parameters_.end();
    }
    void undeclare_parameter(const std::string &name) {
      parameters_.erase(name);
    }
    void set_parameter(const Parameter &param) {
      parameters_[param.get_name()] = param;
    }
    std::vector<Parameter> get_parameters(const std::vector<std::string> &names) const {
      std::vector<Parameter> result;
      result.reserve(names.size());
      for (const auto &n : names) {
        result.push_back(get_parameter(n));
      }
      return result;
    }
    std::vector<std::string> list_parameters(
        const std::vector<std::string> &prefixes, uint64_t /*depth*/) const {
      std::vector<std::string> result;
      for (const auto &kv : parameters_) {
        if (prefixes.empty()) {
          result.push_back(kv.first);
        } else {
          for (const auto &prefix : prefixes) {
            if (kv.first.size() >= prefix.size() &&
                kv.first.substr(0, prefix.size()) == prefix) {
              result.push_back(kv.first);
              break;
            }
          }
        }
      }
      return result;
    }

    std::atomic<bool> closed_{false};
    void stop_spin();

  private:
    virtual void spin();
    virtual void spin_some();

    // vsomeip application initialization helper
    void init_vsomeip_app(const std::string &app_name);

  protected:
    // Protected constructors
    Node(int domain_id);
    Node(int domain_id, const std::string &name);
    Node(int domain_id, const std::string &name, const std::string &ns);
    Node(int domain_id, const std::string &name, const std::string &ns, const NodeOptions &options);
    Node(const std::string &name);
    Node(const std::string &name, const std::string &ns);
    Node(const std::string &name, const std::string &ns, const NodeOptions &options);

  private:
    // Member variables
    std::shared_ptr<vsomeip::application> app_;
    CallbackChannel::SharedPtr channel_;
    std::shared_ptr<std::mutex> callback_mutex_;
    std::shared_ptr<std::condition_variable> node_cv_;
    std::shared_ptr<std::mutex> node_cv_mutex_;
    std::shared_ptr<std::atomic<bool>> node_data_pending_;
    Clock::SharedPtr clock_;
    std::string name_;
    std::string namespace_;
    NodeOptions node_options_;
    std::atomic<bool> stop_flag_{false};
    bool app_owned_;

    // vsomeip event loop thread
    std::thread vsomeip_thread_;
    std::atomic<bool> registered_{false};
    std::mutex registered_mutex_;
    std::condition_variable registered_cv_;

    std::forward_list<std::shared_ptr<IPublisher>> publisher_list_;
    std::forward_list<std::shared_ptr<ISubscription>> subscription_list_;
    std::forward_list<std::shared_ptr<ITimerBase>> timer_list_;
    std::forward_list<std::shared_ptr<IService>> service_list_;
    std::forward_list<std::shared_ptr<IClient>> client_list_;
    Parameters parameters_;

    std::string get_topic_prefix() const
    {
      if (namespace_.empty() || namespace_ == "/")
        return "rt/";
      std::string ns = namespace_;
      if (ns.front() == '/')
        ns = ns.substr(1);
      if (!ns.empty() && ns.back() != '/')
        ns += '/';
      return "rt/" + ns;
    }

    std::string resolve_topic_name(const std::string &topic) const
    {
      if (!topic.empty() && topic[0] == '/')
        return "rt" + topic;
      return get_topic_prefix() + topic;
    }
  };

  // Executor classes
  namespace executors
  {
    class SingleThreadedExecutor
    {
    public:
      SingleThreadedExecutor();
      ~SingleThreadedExecutor();

      SingleThreadedExecutor(const SingleThreadedExecutor &) = delete;
      SingleThreadedExecutor &operator=(const SingleThreadedExecutor &) = delete;
      SingleThreadedExecutor(SingleThreadedExecutor &&) = default;
      SingleThreadedExecutor &operator=(SingleThreadedExecutor &&) = default;

      void add_node(Node::SharedPtr node);
      void remove_node(Node::SharedPtr node);
      void cancel();
      void clear();
      void spin();
      void spin_some();

    private:
      std::vector<Node::SharedPtr> nodes_;
      mutable std::mutex mutex_;
      std::atomic<bool> stop_flag_{true};
    };

    class MultiThreadedExecutor
    {
    public:
      MultiThreadedExecutor();
      ~MultiThreadedExecutor();

      MultiThreadedExecutor(const MultiThreadedExecutor &) = delete;
      MultiThreadedExecutor &operator=(const MultiThreadedExecutor &) = delete;
      MultiThreadedExecutor(MultiThreadedExecutor &&) = default;
      MultiThreadedExecutor &operator=(MultiThreadedExecutor &&) = default;

      void add_node(Node::SharedPtr node);
      void remove_node(Node::SharedPtr node);
      void cancel();
      void clear();
      void spin();
      void spin_some();
      int get_number_of_threads() const;

    private:
      std::vector<Node::SharedPtr> nodes_;
      std::vector<std::thread> threads_;
      mutable std::mutex mutex_;
      std::atomic<bool> stop_flag_{true};
    };
  } // namespace executors

  // Rate class
  class Rate
  {
  public:
    explicit Rate(const Duration &period);

    Rate(const Rate &) = delete;
    Rate &operator=(const Rate &) = delete;
    Rate(Rate &&) = default;
    Rate &operator=(Rate &&) = default;

    void sleep();

  private:
    Duration period_;
    std::chrono::system_clock::time_point next_time_;
  };

  class WallRate
  {
  public:
    explicit WallRate(const Duration &period);

    WallRate(const WallRate &) = delete;
    WallRate &operator=(const WallRate &) = delete;
    WallRate(WallRate &&) = default;
    WallRate &operator=(WallRate &&) = default;

    void sleep();

  private:
    Duration period_;
    std::chrono::steady_clock::time_point next_time_;
  };

  // Logger Class
  enum LogLevel
  {
    DEBUG,
    INFO,
    WARN,
    ERROR
  };

  void log(LogLevel level, const char *format, ...);

  class Logger
  {
  public:
    Logger(const std::string &node_name);

    Logger(const Logger &) = default;
    Logger &operator=(const Logger &) = default;
    Logger(Logger &&) = default;
    Logger &operator=(Logger &&) = default;

    void log(LogLevel level, const char *format, ...) const;

  private:
    std::string node_name_;
  };

  class IService
  {
  public:
    virtual ~IService() = default;
    virtual void stop() = 0;

    IService(const IService &) = delete;
    IService &operator=(const IService &) = delete;
    IService(IService &&) = default;
    IService &operator=(IService &&) = default;

  protected:
    IService() = default;
  };

  // Service uses pub/sub internally (same pattern as CycloneDDS backend)
  template <typename T>
  class Service : public IService, public std::enable_shared_from_this<Service<T>>
  {
  public:
    using SharedPtr = std::shared_ptr<Service>;

    Service(
        std::shared_ptr<vsomeip::application> app, const std::string &service_name,
        std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
            callback_function,
        std::shared_ptr<std::mutex> /*node_mutex*/)
        : IService(),
          std::enable_shared_from_this<Service<T>>(),
          app_(app),
          service_name_(service_name),
          callback_function_(std::move(callback_function)),
          request_callback_function_(),
          publisher_(nullptr),
          subscription_(nullptr),
          request_topic_name_(service_name_ + "_Request"),
          response_topic_name_(service_name_ + "_Response")
    {
      RMWQoSProfile rmw_qos_profile_services = rmw_qos_profile_services_default;
      QoS service_qos(KeepLast(10), rmw_qos_profile_services);

      publisher_ = std::make_shared<Publisher<typename T::Response>>(
          app_, std::string("rp/") + response_topic_name_, service_qos);

      request_callback_function_ = [this](std::shared_ptr<typename T::Request> request)
      {
        std::shared_ptr<typename T::Response> response = std::make_shared<typename T::Response>();
        callback_function_(request, response);
        publisher_->publish(response);
      };

      subscription_ = std::make_shared<Subscription<typename T::Request>>(
          app_, std::string("rp/") + request_topic_name_, service_qos,
          request_callback_function_, std::make_shared<std::mutex>());
    }

    ~Service() = default;

    Service(const Service &) = delete;
    Service &operator=(const Service &) = delete;
    Service(Service &&) = delete;
    Service &operator=(Service &&) = delete;

    void stop() override
    {
      if (subscription_)
      {
        subscription_->stop();
      }
    }

  private:
    std::shared_ptr<vsomeip::application> app_;
    std::string service_name_;
    std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
        callback_function_;
    std::function<void(std::shared_ptr<typename T::Request>)> request_callback_function_;
    std::shared_ptr<Publisher<typename T::Response>> publisher_;
    std::shared_ptr<Subscription<typename T::Request>> subscription_;
    std::string request_topic_name_;
    std::string response_topic_name_;
  };

  class IClient
  {
  public:
    virtual ~IClient() = default;
    virtual void stop() = 0;

    IClient(const IClient &) = delete;
    IClient &operator=(const IClient &) = delete;
    IClient(IClient &&) = default;
    IClient &operator=(IClient &&) = default;

  protected:
    IClient() = default;
  };

  enum FutureReturnCode
  {
    SUCCESS,
    INTERRUPTED,
    TIMEOUT
  };

  class FutureBase
  {
  public:
    virtual ~FutureBase() = default;
    virtual std::future_status wait_for(std::chrono::milliseconds timeout) = 0;

    FutureBase(const FutureBase &) = delete;
    FutureBase &operator=(const FutureBase &) = delete;
    FutureBase(FutureBase &&) = default;
    FutureBase &operator=(FutureBase &&) = default;

  protected:
    FutureBase() = default;
  };

  template <typename T>
  class TypedFuture : public FutureBase
  {
  public:
    explicit TypedFuture(std::shared_future<std::shared_ptr<T>> future)
        : FutureBase(), future_(std::move(future))
    {
    }

    TypedFuture(const TypedFuture &) = delete;
    TypedFuture &operator=(const TypedFuture &) = delete;
    TypedFuture(TypedFuture &&) = default;
    TypedFuture &operator=(TypedFuture &&) = default;

    std::future_status wait_for(std::chrono::milliseconds timeout) override
    {
      return future_.wait_for(timeout);
    }

    std::shared_ptr<T> get()
    {
      return future_.get();
    }

  private:
    std::shared_future<std::shared_ptr<T>> future_;
  };

  template <typename T>
  class Client : public IClient, public std::enable_shared_from_this<Client<T>>
  {
  public:
    using SharedPtr = std::shared_ptr<Client>;
    using SharedRequest = std::shared_ptr<typename T::Request>;
    using SharedResponse = std::shared_ptr<typename T::Response>;
    using SharedFuture = std::shared_future<SharedResponse>;

    Client(
        std::shared_ptr<vsomeip::application> app, const std::string &service_name,
        std::shared_ptr<std::mutex> /*node_mutex*/)
        : IClient(),
          std::enable_shared_from_this<Client<T>>(),
          app_(app),
          service_name_(service_name),
          response_(nullptr),
          publisher_(nullptr),
          subscription_(nullptr),
          request_topic_name_(service_name_ + "_Request"),
          response_topic_name_(service_name_ + "_Response"),
          mutex_(),
          cv_(),
          response_received_(false)
    {
      RMWQoSProfile rmw_qos_profile_services = rmw_qos_profile_services_default;
      QoS client_qos(KeepLast(10), rmw_qos_profile_services);

      publisher_ = std::make_shared<Publisher<typename T::Request>>(
          app_, std::string("rp/") + request_topic_name_, client_qos);

      subscription_ = std::make_shared<Subscription<typename T::Response>>(
          app_, std::string("rp/") + response_topic_name_, client_qos,
          std::function<void(std::shared_ptr<typename T::Response>)>(
              [this](std::shared_ptr<typename T::Response> resp) { handle_response(std::move(resp)); }),
          std::make_shared<std::mutex>());
    }

    ~Client() = default;

    Client(const Client &) = delete;
    Client &operator=(const Client &) = delete;
    Client(Client &&) = delete;
    Client &operator=(Client &&) = delete;

    void handle_response(std::shared_ptr<typename T::Response> response)
    {
      std::shared_ptr<std::promise<std::shared_ptr<typename T::Response>>> promise;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!response_promises_.empty())
        {
          promise = response_promises_.front();
          response_promises_.pop();
        }
        response_ = response;
        response_received_ = true;
      }
      cv_.notify_one();

      if (promise)
      {
        promise->set_value(response);
      }
    }

    void stop() override
    {
      if (subscription_)
      {
        subscription_->stop();
      }
    }

    SharedFuture async_send_request(SharedRequest request)
    {
      auto promise = std::make_shared<std::promise<SharedResponse>>();
      auto future = promise->get_future().share();

      {
        std::lock_guard<std::mutex> lock(mutex_);
        response_promises_.push(promise);
      }

      publisher_->publish(request);

      return future;
    }

    template <typename Duration>
    bool wait_for_service(const Duration &timeout)
    {
      std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
      std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
      std::chrono::system_clock::time_point end_time = start_time + timeout;

      while (current_time < end_time)
      {
        if (publisher_->get_subscriber_count() > 0)
        {
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        current_time = std::chrono::system_clock::now();
      }

      return false;
    }

    bool service_is_ready() const {
      return publisher_->get_subscriber_count() > 0;
    }

  private:
    std::shared_ptr<vsomeip::application> app_;
    std::string service_name_;
    std::shared_ptr<typename T::Response> response_;
    std::queue<std::shared_ptr<std::promise<std::shared_ptr<typename T::Response>>>> response_promises_;
    std::shared_ptr<Publisher<typename T::Request>> publisher_;
    std::shared_ptr<Subscription<typename T::Response>> subscription_;
    std::string request_topic_name_;
    std::string response_topic_name_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> response_received_{false};
  };

  template <typename Duration>
  FutureReturnCode spin_until_future_complete(
      std::shared_ptr<lwrcl::Node> node, std::shared_ptr<FutureBase> future, const Duration &timeout)
  {
    std::thread spin_thread([node]()
                            { lwrcl::spin(node); });

    if (
        future->wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout)) ==
        std::future_status::ready)
    {
      node->stop_spin();
      spin_thread.join();
      return SUCCESS;
    }
    else
    {
      node->stop_spin();
      spin_thread.join();
      return TIMEOUT;
    }
  }

  template <typename ResponseT, typename Duration>
  FutureReturnCode spin_until_future_complete(
      std::shared_ptr<lwrcl::Node> node, std::shared_future<ResponseT> &future, const Duration &timeout)
  {
    std::thread spin_thread([node]()
                            { lwrcl::spin(node); });

    if (
        future.wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout)) ==
        std::future_status::ready)
    {
      node->stop_spin();
      spin_thread.join();
      return SUCCESS;
    }
    else
    {
      node->stop_spin();
      spin_thread.join();
      return TIMEOUT;
    }
  }

  // SerializedMessage and lwrcl_serialized_message_t are defined in serialized_message.hpp
  // (included via publisher.hpp / subscription.hpp)

  template <typename T>
  class Serialization
  {
  public:
    static void serialize_message(T *message, SerializedMessage *serialized_message)
    {
      using namespace org::eclipse::cyclonedds::core::cdr;

      // Determine serialized size via move (dry-run)
      basic_cdr_stream sizer;
      move(sizer, *message, false);
      size_t payload_size = sizer.position();

      // 4-byte CDR header + payload
      serialized_message->reserve(payload_size + 4);
      char *buf = serialized_message->get_rcl_serialized_message().buffer;

      uint8_t header[4] = {0x00, 0x01, 0x00, 0x00};
      memcpy(buf, header, 4);

      // Serialize into buffer after header
      basic_cdr_stream writer;
      writer.set_buffer(buf + 4, payload_size);
      write(writer, *message, false);

      serialized_message->get_rcl_serialized_message().length = payload_size + 4;
    }

    static void deserialize_message(SerializedMessage *serialized_message, T *message)
    {
      using namespace org::eclipse::cyclonedds::core::cdr;

      char *buf = serialized_message->get_rcl_serialized_message().buffer;
      size_t length = serialized_message->get_rcl_serialized_message().length;

      if (length <= 4)
      {
        return;
      }

      // Skip 4-byte CDR header and deserialize payload
      basic_cdr_stream reader;
      reader.set_buffer(buf + 4, length - 4);
      read(reader, *message, false);
    }
  };
} // namespace lwrcl

#define LWRCL_DEBUG(logger, ...) (logger).log(lwrcl::DEBUG, __VA_ARGS__)
#define LWRCL_INFO(logger, ...) (logger).log(lwrcl::INFO, __VA_ARGS__)
#define LWRCL_WARN(logger, ...) (logger).log(lwrcl::WARN, __VA_ARGS__)
#define LWRCL_ERROR(logger, ...) (logger).log(lwrcl::ERROR, __VA_ARGS__)

#endif // LWRCL_HPP_
