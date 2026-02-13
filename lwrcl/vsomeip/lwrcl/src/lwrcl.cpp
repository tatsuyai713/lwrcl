#include "lwrcl.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cassert>
#include <condition_variable>
#include <csignal>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace lwrcl
{
  std::condition_variable s_waiting_cv;
  std::mutex s_waiting_mutex;
  volatile std::sig_atomic_t s_signal_status = 0;

  std::atomic_bool global_stop_flag{false};

  void lwrcl_signal_handler(int signal)
  {
    if ((signal == SIGINT || signal == SIGTERM))
    {
      printf("SIGINT/SIGTERM received, shutting down...\n");
      global_stop_flag.store(true);
      s_signal_status = signal;
      s_waiting_cv.notify_all();
    }
  }

  class Node;

  NodeParameters node_parameters;

  // Parameter implementations (identical to CycloneDDS backend)
  Parameter::Parameter(const std::string &name, bool value)
      : name_(name), string_value_(value ? "true" : "false"), type_(Type::BOOL) {}

  Parameter::Parameter(const std::string &name, int value)
      : name_(name), string_value_(int_to_string(value)), type_(Type::INT) {}

  Parameter::Parameter(const std::string &name, double value)
      : name_(name), string_value_(double_to_string(value)), type_(Type::DOUBLE) {}

  Parameter::Parameter(const std::string &name, const std::string &value)
      : name_(name), string_value_(value), type_(Type::STRING) {}

  Parameter::Parameter(const std::string &name, const char *value)
      : name_(name), string_value_(value), type_(Type::STRING) {}

  Parameter::Parameter(const std::string &name, const std::vector<bool> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::BOOL_ARRAY) {}

  Parameter::Parameter(const std::string &name, const std::vector<int> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::INT_ARRAY) {}

  Parameter::Parameter(const std::string &name, const std::vector<double> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::DOUBLE_ARRAY) {}

  Parameter::Parameter(const std::string &name, const std::vector<std::string> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::STRING_ARRAY) {}

  Parameter::Parameter(const std::string &name, std::vector<const char *> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::STRING_ARRAY) {}

  Parameter::Parameter(const std::string &name, const std::vector<uint8_t> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::BYTE_ARRAY) {}

  Parameter::Parameter() : name_(), string_value_(), type_(Type::UNKNOWN) {}

  std::string Parameter::get_name() const { return name_; }

  bool Parameter::as_bool() const
  {
    if (type_ != Type::BOOL)
      throw std::runtime_error("Parameter is not a bool");
    return string_value_ == "true";
  }

  int Parameter::as_int() const
  {
    if (type_ != Type::INT)
      throw std::runtime_error("Parameter is not an int");
    int int_value{};
    std::istringstream iss(string_value_);
    iss >> int_value;
    return int_value;
  }

  double Parameter::as_double() const
  {
    if (type_ != Type::DOUBLE)
      throw std::runtime_error("Parameter is not a double");
    double double_value{};
    std::istringstream iss(string_value_);
    iss >> double_value;
    return double_value;
  }

  std::string Parameter::as_string() const { return string_value_; }

  std::vector<bool> Parameter::as_bool_array() const
  {
    if (type_ != Type::BOOL_ARRAY)
      throw std::runtime_error("Parameter is not a bool array");
    return string_to_vector<bool>(string_value_);
  }

  std::vector<int> Parameter::as_integer_array() const
  {
    if (type_ != Type::INT_ARRAY)
      throw std::runtime_error("Parameter is not an int array");
    return string_to_vector<int>(string_value_);
  }

  std::vector<double> Parameter::as_double_array() const
  {
    if (type_ != Type::DOUBLE_ARRAY)
      throw std::runtime_error("Parameter is not a double array");
    return string_to_vector<double>(string_value_);
  }

  std::vector<std::string> Parameter::as_string_array() const
  {
    if (type_ != Type::STRING_ARRAY)
      throw std::runtime_error("Parameter is not a string array");
    return string_to_vector<std::string>(string_value_);
  }

  std::vector<uint8_t> Parameter::as_byte_array() const
  {
    if (type_ != Type::BYTE_ARRAY)
      throw std::runtime_error("Parameter is not a byte array");
    return string_to_vector<uint8_t>(string_value_);
  }

  std::string Parameter::int_to_string(int value)
  {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }

  std::string Parameter::double_to_string(double value)
  {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }

  // Logger implementations
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
    struct tm local_time_buf;
    auto local_time = localtime_r(&timestamp, &local_time_buf);
    if (local_time != nullptr)
    {
      std::strftime(timestamp_buf, sizeof(timestamp_buf), "%Y-%m-%d %H:%M:%S", local_time);
    }
    else
    {
      std::snprintf(timestamp_buf, sizeof(timestamp_buf), "0000-00-00 00:00:00");
      ms = std::chrono::milliseconds(0);
    }

    switch (level)
    {
    case DEBUG:
      msg << "\033[0;36m[DEBUG]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    case INFO:
      msg << "\033[0;37m[INFO]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    case WARN:
      msg << "\033[0;33m[WARN]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    case ERROR:
      msg << "\033[0;31m[ERROR]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    default:
      msg << "[Unknown log level] [" << timestamp_buf << "." << std::setfill('0') << std::setw(3)
          << ms.count() << "] ";
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
    struct tm local_time_buf;
    auto local_time = localtime_r(&timestamp, &local_time_buf);
    std::strftime(timestamp_buf, sizeof(timestamp_buf), "%Y-%m-%d %H:%M:%S", local_time);

    switch (level)
    {
    case DEBUG:
      msg << "\033[0;36m[DEBUG]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    case INFO:
      msg << "\033[0;37m[INFO]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    case WARN:
      msg << "\033[0;33m[WARN]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    case ERROR:
      msg << "\033[0;31m[ERROR]\033[0m [" << timestamp_buf << "." << std::setfill('0')
          << std::setw(3) << ms.count() << "] ";
      break;
    default:
      msg << "[Unknown log level] [" << timestamp_buf << "." << std::setfill('0') << std::setw(3)
          << ms.count() << "] ";
      break;
    }

    msg << "[" << node_name_ << "]: ";
    std::cout << msg.str();
    vprintf(format, args);
    std::cout << std::endl;

    va_end(args);
  }

  // Executor implementations
  namespace executors
  {
    SingleThreadedExecutor::SingleThreadedExecutor() : stop_flag_(true) {}
    SingleThreadedExecutor::~SingleThreadedExecutor() { clear(); }

    void SingleThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
        nodes_.emplace_back(node);
      else
        throw std::runtime_error("Error: Node pointer is null, cannot add to executor.");
    }

    void SingleThreadedExecutor::remove_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
        nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
    }

    void SingleThreadedExecutor::clear()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto &node : nodes_)
      {
        if (node != nullptr && node->closed_ == false)
          node->shutdown();
      }
      nodes_.clear();
      stop_flag_ = true;
    }

    void SingleThreadedExecutor::cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_flag_ = true;
    }

    void SingleThreadedExecutor::spin()
    {
      stop_flag_ = false;
      while (global_stop_flag.load() == false && stop_flag_ == false)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto node : nodes_)
        {
          if (node != nullptr && node->closed_ == false)
            lwrcl::spin_some(node);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }

      if (global_stop_flag.load() == true)
        clear();
    }

    void SingleThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node != nullptr && node->closed_ == false)
          lwrcl::spin_some(node);
      }
    }

    MultiThreadedExecutor::MultiThreadedExecutor() : stop_flag_(true) {}
    MultiThreadedExecutor::~MultiThreadedExecutor() { clear(); }

    void MultiThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
        nodes_.emplace_back(node);
      else
        throw std::runtime_error("Error: Node pointer is null, cannot add to executor.");
    }

    void MultiThreadedExecutor::remove_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
        nodes_.erase(std::remove(nodes_.begin(), nodes_.end(), node), nodes_.end());
    }

    void MultiThreadedExecutor::cancel()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node != nullptr && node->closed_ == false)
          node->stop_spin();
      }
      stop_flag_ = true;
    }

    void MultiThreadedExecutor::clear()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto &node : nodes_)
      {
        if (node != nullptr)
        {
          if (node->closed_ == false)
            node->shutdown();
        }
        else
        {
          std::cerr << "[WARN] Node pointer is null in executor clear." << std::endl;
        }
      }
      nodes_.clear();
      stop_flag_ = true;
    }

    void MultiThreadedExecutor::spin()
    {
      stop_flag_ = false;
      for (auto node : nodes_)
      {
        threads_.emplace_back([this, node]()
                              {
          if (node != nullptr) {
            if (node->closed_ == false)
              lwrcl::spin(node);
          } else {
            std::cerr << "[WARN] Node pointer is null in executor spin." << std::endl;
          } });
      }

      for (auto &thread : threads_)
      {
        if (thread.joinable())
          thread.join();
      }
      threads_.clear();

      if (global_stop_flag.load() == true)
        clear();
    }

    void MultiThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto node : nodes_)
      {
        if (node != nullptr)
        {
          if (node->closed_ == false)
            lwrcl::spin_some(node);
        }
        else
        {
          throw std::runtime_error("Error: Node pointer is null, cannot add to executor.");
        }
      }
    }

    int MultiThreadedExecutor::get_number_of_threads() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return static_cast<int>(threads_.size());
    }
  } // namespace executors

  // Time/Duration/Clock implementations
  Time::Time() : nanoseconds_(0) {}
  Time::Time(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Time::Time(int32_t seconds, uint32_t nanoseconds)
      : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
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
  Duration::Duration(int32_t seconds, uint32_t nanoseconds)
      : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds) {}
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
  Rate::Rate(const Duration &period)
      : period_(period),
        next_time_(std::chrono::system_clock::now() +
                   std::chrono::duration_cast<std::chrono::system_clock::duration>(
                       std::chrono::nanoseconds(period.nanoseconds())))
  {
  }

  void Rate::sleep()
  {
    using sys_clock = std::chrono::system_clock;
    using sys_dur = sys_clock::duration;

    const sys_dur period_d =
        std::chrono::duration_cast<sys_dur>(std::chrono::nanoseconds(period_.nanoseconds()));

    auto now = sys_clock::now();
    if (now >= next_time_)
    {
      const auto behind = now - next_time_;
      const auto periods_missed = behind / period_d + sys_dur::rep(1);
      next_time_ += periods_missed * period_d;
    }

    std::this_thread::sleep_until(next_time_);
    next_time_ += period_d;
  }

  WallRate::WallRate(const Duration &period)
      : period_(period),
        next_time_(std::chrono::steady_clock::now() + std::chrono::nanoseconds(period.nanoseconds()))
  {
  }

  void WallRate::sleep()
  {
    auto now = std::chrono::steady_clock::now();
    if (now >= next_time_)
    {
      auto periods_missed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - next_time_) /
                                std::chrono::nanoseconds(period_.nanoseconds()) +
                            1;
      next_time_ += periods_missed * std::chrono::nanoseconds(period_.nanoseconds());
    }

    std::this_thread::sleep_until(next_time_);
    next_time_ += std::chrono::nanoseconds(period_.nanoseconds());
  }

  // ======================================================================
  // vsomeip Application initialization helper
  // ======================================================================
  void Node::init_vsomeip_app(const std::string &app_name)
  {
    // If VSOMEIP_ROUTING is not explicitly set, make this application its
    // own routing manager.  This avoids the need for an external vsomeipd
    // daemon in single-process / test scenarios.  Users who set VSOMEIP_ROUTING
    // in the environment or in a vsomeip JSON config keep full control.
    if (!std::getenv("VSOMEIP_ROUTING"))
    {
      setenv("VSOMEIP_ROUTING", app_name.c_str(), 0);
    }

    app_ = vsomeip::runtime::get()->create_application(app_name);
    if (!app_)
    {
      throw std::runtime_error("Failed to create vsomeip application: " + app_name);
    }

    if (!app_->init())
    {
      throw std::runtime_error("Failed to initialize vsomeip application: " + app_name);
    }

    // Register state handler to know when the app is registered with the routing manager
    app_->register_state_handler(
        [this](vsomeip::state_type_e state)
        {
          if (state == vsomeip::state_type_e::ST_REGISTERED)
          {
            std::lock_guard<std::mutex> lock(registered_mutex_);
            registered_.store(true);
            registered_cv_.notify_all();
          }
        });

    // Start vsomeip event loop in a background thread
    vsomeip_thread_ = std::thread([this]()
                                  { app_->start(); });

    // Wait for the application to be registered (with timeout)
    {
      std::unique_lock<std::mutex> lock(registered_mutex_);
      if (!registered_cv_.wait_for(lock, std::chrono::seconds(10),
                                   [this]()
                                   { return registered_.load(); }))
      {
        std::cerr << "[WARN] vsomeip application registration timed out" << std::endl;
      }
    }
  }

  // ======================================================================
  // Node constructors
  // ======================================================================
  Node::Node(int domain_id)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_("lwrcl_default_node"),
        namespace_(""),
        node_options_(),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    (void)domain_id; // vsomeip doesn't use domain IDs
    init_vsomeip_app(name_);
  }

  Node::Node(int domain_id, const std::string &name)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(""),
        node_options_(),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    (void)domain_id;
    init_vsomeip_app(name_);
  }

  Node::Node(int domain_id, const std::string &name, const std::string &ns)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(ns),
        node_options_(),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    (void)domain_id;
    init_vsomeip_app(name_);
  }

  Node::Node(int domain_id, const std::string &name, const std::string &ns, const NodeOptions &options)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(ns),
        node_options_(options),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    (void)domain_id;
    init_vsomeip_app(name_);
  }

  Node::Node(const std::string &name)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(""),
        node_options_(),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    init_vsomeip_app(name_);
  }

  Node::Node(const std::string &name, const std::string &ns)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(ns),
        node_options_(),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    init_vsomeip_app(name_);
  }

  Node::Node(const std::string &name, const std::string &ns, const NodeOptions &options)
      : closed_(false),
        app_(nullptr),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(ns),
        node_options_(options),
        stop_flag_(false),
        app_owned_(true),
        registered_(false),
        parameters_()
  {
    init_vsomeip_app(name_);
  }

  Node::Node(std::shared_ptr<vsomeip::application> app)
      : closed_(false),
        app_(app),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_("lwrcl_default_node"),
        namespace_(""),
        node_options_(),
        stop_flag_(false),
        app_owned_(false),
        registered_(true), // Assume external app is already started
        parameters_()
  {
    if (!app_)
      throw std::runtime_error("vsomeip application pointer is null");
  }

  Node::Node(std::shared_ptr<vsomeip::application> app, const std::string &name)
      : closed_(false),
        app_(app),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(""),
        node_options_(),
        stop_flag_(false),
        app_owned_(false),
        registered_(true),
        parameters_()
  {
    if (!app_)
      throw std::runtime_error("vsomeip application pointer is null");
  }

  Node::Node(std::shared_ptr<vsomeip::application> app, const std::string &name, const std::string &ns)
      : closed_(false),
        app_(app),
        channel_(std::make_shared<CallbackChannel>()),
        clock_(std::make_unique<Clock>()),
        name_(name),
        namespace_(ns),
        node_options_(),
        stop_flag_(false),
        app_owned_(false),
        registered_(true),
        parameters_()
  {
    if (!app_)
      throw std::runtime_error("vsomeip application pointer is null");
  }

  Node::~Node() { shutdown(); }

  // ======================================================================
  // Node::make_shared factories
  // ======================================================================
  std::shared_ptr<Node> Node::make_shared(int domain_id)
  {
    return std::shared_ptr<Node>(new Node(domain_id), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name)
  {
    return std::shared_ptr<Node>(new Node(domain_id, name), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(const std::string &name)
  {
    return std::shared_ptr<Node>(new Node(name), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(std::shared_ptr<vsomeip::application> app)
  {
    return std::shared_ptr<Node>(new Node(app), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(std::shared_ptr<vsomeip::application> app, const std::string &name)
  {
    return std::shared_ptr<Node>(new Node(app, name), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name, const std::string &ns)
  {
    return std::shared_ptr<Node>(new Node(domain_id, name, ns), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name, const std::string &ns, const NodeOptions &options)
  {
    return std::shared_ptr<Node>(new Node(domain_id, name, ns, options), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(const std::string &name, const std::string &ns)
  {
    return std::shared_ptr<Node>(new Node(name, ns), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(const std::string &name, const std::string &ns, const NodeOptions &options)
  {
    return std::shared_ptr<Node>(new Node(name, ns, options), [](Node *node)
                                 { delete node; });
  }

  std::shared_ptr<Node> Node::make_shared(std::shared_ptr<vsomeip::application> app, const std::string &name, const std::string &ns)
  {
    return std::shared_ptr<Node>(new Node(app, name, ns), [](Node *node)
                                 { delete node; });
  }

  // ======================================================================
  // Node accessors
  // ======================================================================
  std::shared_ptr<vsomeip::application> Node::get_participant() const
  {
    return app_;
  }

  std::string Node::get_name() const { return name_; }
  std::string Node::get_namespace() const { return namespace_; }

  std::string Node::get_fully_qualified_name() const
  {
    if (namespace_.empty())
      return "/" + name_;
    std::string ns = namespace_;
    if (ns.front() != '/')
      ns = "/" + ns;
    if (ns.back() == '/')
      ns.pop_back();
    return ns + "/" + name_;
  }

  const Node::NodeOptions &Node::get_node_options() const { return node_options_; }
  Logger Node::get_logger() const { return Logger(name_); }

  // ======================================================================
  // Node spin
  // ======================================================================
  void Node::spin()
  {
    stop_flag_ = false;
    while (closed_ == false && global_stop_flag.load() == false && stop_flag_ == false)
    {
      CallbackPtr callback;
      while (channel_->consume(callback) && global_stop_flag.load() == false && stop_flag_ == false)
      {
        if (callback)
          callback->invoke();
        else
          break;
      }
    }

    if (global_stop_flag.load() == true)
      shutdown();
  }

  void Node::stop_spin() { stop_flag_ = true; }

  void Node::spin_some()
  {
    bool event_processed = false;
    do
    {
      event_processed = false;
      CallbackPtr callback;
      while (channel_->consume_nowait(callback))
      {
        callback->invoke();
        event_processed = true;
      }
    } while (event_processed);
  }

  // ======================================================================
  // Node shutdown
  // ======================================================================
  void Node::shutdown()
  {
    if (closed_.load())
      return;

    publisher_list_.clear();
    for (auto &subscriber : subscription_list_)
      std::static_pointer_cast<ISubscription>(subscriber)->stop();
    subscription_list_.clear();
    for (auto &timer : timer_list_)
      std::static_pointer_cast<TimerBase>(timer)->stop();
    timer_list_.clear();
    for (auto &service : service_list_)
      std::static_pointer_cast<IService>(service)->stop();
    service_list_.clear();
    for (auto &client : client_list_)
      std::static_pointer_cast<IClient>(client)->stop();
    client_list_.clear();

    // Stop the vsomeip application if we own it
    if (app_owned_ && app_)
    {
      app_->stop();
      if (vsomeip_thread_.joinable())
        vsomeip_thread_.join();
    }

    closed_ = true;
  }

  Clock::SharedPtr Node::get_clock() { return clock_; }

  // ======================================================================
  // Parameter methods (identical to CycloneDDS backend)
  // ======================================================================
  void Node::set_parameters(const std::vector<std::shared_ptr<ParameterBase>> &parameters)
  {
    for (const auto &param : parameters)
    {
      std::string node_name = this->get_name();
      std::string param_name = param->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        Parameters &params = node_it->second;
        if (params.find(param_name) != params.end())
        {
          params[param_name] = *std::dynamic_pointer_cast<Parameter>(param);
          parameters_[param_name] = *std::dynamic_pointer_cast<Parameter>(param);
        }
        else
        {
          throw std::runtime_error("Parameter not found");
        }
      }
      else
      {
        throw std::runtime_error("Node not found");
      }
    }
  }

  void Node::set_parameters(const std::vector<Parameter> &parameters)
  {
    std::vector<std::shared_ptr<ParameterBase>> base_params;
    base_params.reserve(parameters.size());
    for (const auto &param : parameters)
      base_params.push_back(std::make_shared<Parameter>(param));
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      auto param_it = node_it->second.find(name);
      if (param_it != node_it->second.end())
      {
        parameters_[name] = param_it->second;
        node_parameters[node_name][name] = param_it->second;
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
      return it->second;
    else
      throw std::runtime_error("Parameter not found");
  }

  void Node::get_parameter(const std::string &name, bool &bool_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
      bool_data = it->second.as_bool();
    else
      throw std::runtime_error("Parameter not found");
  }

  void Node::get_parameter(const std::string &name, int &int_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
      int_data = it->second.as_int();
    else
      throw std::runtime_error("Parameter not found");
  }

  void Node::get_parameter(const std::string &name, double &double_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
      double_data = it->second.as_double();
    else
      throw std::runtime_error("Parameter not found");
  }

  void Node::get_parameter(const std::string &name, std::string &string_data) const
  {
    auto it = parameters_.find(name);
    if (it != parameters_.end())
      string_data = it->second.as_string();
    else
      throw std::runtime_error("Parameter not found");
  }

  // ======================================================================
  // Free functions
  // ======================================================================
  void init(int argc, char *argv[])
  {
    global_stop_flag.store(false);

    if (std::signal(SIGINT, lwrcl_signal_handler) == SIG_ERR)
      throw std::runtime_error("Failed to set signal handler.");
    if (std::signal(SIGTERM, lwrcl_signal_handler) == SIG_ERR)
      throw std::runtime_error("Failed to set signal handler.");

    try
    {
      std::string params_file_path = get_params_file_path(argc, argv);
      if (!params_file_path.empty())
      {
        std::cout << "Loading parameters from file: " << params_file_path << std::endl;
        load_parameters(params_file_path);
      }
    }
    catch (const YAML::Exception &)
    {
      throw std::runtime_error("Error parsing YAML file");
    }
    catch (const std::exception &e)
    {
      throw std::runtime_error(e.what());
    }
  }

  bool ok() { return !global_stop_flag.load(); }

  void spin(std::shared_ptr<Node> node)
  {
    if (node != nullptr)
    {
      if (node->closed_ == false)
        node->spin();
    }
    else
    {
      throw std::runtime_error("Node pointer is invalid!");
    }
  }

  void stop_spin() { global_stop_flag.store(true); }

  void spin_some(std::shared_ptr<Node> node)
  {
    if (node != nullptr)
    {
      if (node->closed_ == false)
        node->spin_some();
    }
    else
    {
      throw std::runtime_error("Node pointer is invalid!");
    }
  }

  void shutdown() { global_stop_flag.store(true); }

  std::string get_params_file_path(int argc, char *argv[])
  {
    if (argc < 2)
      return "";

    bool found_ros_args = false;
    for (int i = 1; i < argc; ++i)
    {
      if (std::string(argv[i]) == "--ros-args")
        found_ros_args = true;
      else if (found_ros_args && std::string(argv[i]) == "--param-file")
      {
        if (i + 1 < argc)
          return std::string(argv[i + 1]);
        else
          return "";
      }
    }
    return "";
  }

  void load_parameters(const std::string &file_path)
  {
    YAML::Node config = YAML::LoadFile(file_path);

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      std::string node_name = it->first.as<std::string>();
      YAML::Node parameters = it->second["ros__parameters"];

      Parameters params;
      for (YAML::const_iterator param_it = parameters.begin(); param_it != parameters.end();
           ++param_it)
      {
        std::string param_name = param_it->first.as<std::string>();
        auto param_value = param_it->second;
        try
        {
          int value = param_value.as<int>();
          params[param_name] = Parameter(param_name, value);
        }
        catch (const YAML::BadConversion &)
        {
          try
          {
            double value = param_value.as<double>();
            params[param_name] = Parameter(param_name, value);
          }
          catch (const YAML::BadConversion &)
          {
            try
            {
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

  void sleep_for(const lwrcl::Duration &duration)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(duration.nanoseconds()));
  }

  // Default QoS profiles
  const RMWQoSProfile rmw_qos_profile_default = {
      10, RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_sensor_data = {
      5, RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::BEST_EFFORT,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_parameters = {
      1000, RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_services_default = {
      10, RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_parameter_events = {
      1000, RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

} // namespace lwrcl
