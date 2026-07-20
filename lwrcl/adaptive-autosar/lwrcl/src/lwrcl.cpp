#include "lwrcl.hpp" // The main header file for the lwrcl namespace

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cassert>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <unistd.h>

namespace lwrcl
{
  namespace
  {
    Duration period_from_rate_hz(double rate_hz)
    {
      if (!std::isfinite(rate_hz) || rate_hz <= 0.0)
      {
        throw std::invalid_argument("rate_hz must be finite and greater than zero");
      }
      return Duration(std::chrono::duration<double>(1.0 / rate_hz));
    }
  }

  std::condition_variable s_waiting_cv;
  std::mutex s_waiting_mutex;
  volatile std::sig_atomic_t s_signal_status = 0;

  // Global flag to control the stopping of the application, e.g., in response to SIGINT
  std::atomic_bool global_stop_flag{false};

  // Function to handle SIGINT signals for graceful application termination
  void lwrcl_signal_handler(int signal)
  {
    if ((signal == SIGINT || signal == SIGTERM))
    {
      static const char message[] = "SIGINT/SIGTERM received, shutting down...\n";
      (void)::write(STDERR_FILENO, message, sizeof(message) - 1);
      global_stop_flag.store(true);
      s_signal_status = signal;
    }
  }

  class Node;

  NodeParameters node_parameters;
  std::mutex node_parameters_mutex;

  namespace
  {
    std::chrono::nanoseconds timer_wait_timeout(const std::vector<std::shared_ptr<ITimerBase>> &timers)
    {
      const auto max_wait = std::chrono::milliseconds(100);
      if (timers.empty()) return max_wait;

      auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(max_wait);
      for (const auto &timer : timers)
      {
        const auto until_next = timer->time_until_next_call();
        if (until_next <= std::chrono::nanoseconds::zero()) return std::chrono::nanoseconds::zero();
        if (until_next == std::chrono::nanoseconds::max()) continue;
        if (until_next < timeout) timeout = until_next;
      }
      return timeout;
    }

    bool parse_parameter_int(const std::string &text, int &value)
    {
      if (text.empty()) return false;
      char *end = nullptr;
      errno = 0;
      long parsed = std::strtol(text.c_str(), &end, 10);
      if (end == text.c_str() || *end != '\0' || errno == ERANGE ||
          parsed < INT_MIN || parsed > INT_MAX)
      {
        return false;
      }
      value = static_cast<int>(parsed);
      return true;
    }

    bool parse_parameter_double(const std::string &text, double &value)
    {
      if (text.empty()) return false;
      char *end = nullptr;
      errno = 0;
      double parsed = std::strtod(text.c_str(), &end);
      if (end == text.c_str() || *end != '\0' || errno == ERANGE)
      {
        return false;
      }
      value = parsed;
      return true;
    }

    bool parse_parameter_bool(const std::string &text, bool &value)
    {
      std::string lower = text;
      std::transform(lower.begin(), lower.end(), lower.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (lower == "true" || lower == "yes" || lower == "on")
      {
        value = true;
        return true;
      }
      if (lower == "false" || lower == "no" || lower == "off")
      {
        value = false;
        return true;
      }
      return false;
    }

    Parameter make_parameter_from_yaml_scalar(const std::string &name, const YAML::Node &node)
    {
      const std::string text = node.as<std::string>();
      int int_value{};
      if (parse_parameter_int(text, int_value)) return Parameter(name, int_value);
      double double_value{};
      if (parse_parameter_double(text, double_value)) return Parameter(name, double_value);
      bool bool_value{};
      if (parse_parameter_bool(text, bool_value)) return Parameter(name, bool_value);
      return Parameter(name, text);
    }
  }

  Parameter::Parameter(const std::string &name, bool value)
      : name_(name), string_value_(value ? "true" : "false"), type_(Type::BOOL)
  {
  }

  Parameter::Parameter(const std::string &name, int value)
      : name_(name), string_value_(int_to_string(value)), type_(Type::INT)
  {
  }

  Parameter::Parameter(const std::string &name, double value)
      : name_(name), string_value_(double_to_string(value)), type_(Type::DOUBLE)
  {
  }

  Parameter::Parameter(const std::string &name, const std::string &value)
      : name_(name), string_value_(value), type_(Type::STRING)
  {
  }

  Parameter::Parameter(const std::string &name, const char *value)
      : name_(name), string_value_(value), type_(Type::STRING)
  {
  }

  Parameter::Parameter(const std::string &name, const std::vector<bool> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::BOOL_ARRAY)
  {
  }

  Parameter::Parameter(const std::string &name, const std::vector<int> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::INT_ARRAY)
  {
  }

  Parameter::Parameter(const std::string &name, const std::vector<double> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::DOUBLE_ARRAY)
  {
  }

  Parameter::Parameter(const std::string &name, const std::vector<std::string> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::STRING_ARRAY)
  {
  }

  Parameter::Parameter(const std::string &name, std::vector<const char *> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::STRING_ARRAY)
  {
  }

  Parameter::Parameter(const std::string &name, const std::vector<uint8_t> &value)
      : name_(name), string_value_(vector_to_string(value)), type_(Type::BYTE_ARRAY)
  {
  }

  Parameter::Parameter() : name_(), string_value_(), type_(Type::UNKNOWN) {}

  const std::string &Parameter::get_name() const { return name_; }

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
    int int_value{};
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
    double double_value{};
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
      throw std::runtime_error("Parameter is not a byte array");
    }
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
    oss << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
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

    msg << "[" << node_name_ << "]: ";
    std::cout << msg.str();
    vprintf(format, args);
    std::cout << std::endl;

    va_end(args);
  }

  namespace executors
  {
    SingleThreadedExecutor::SingleThreadedExecutor() : stop_flag_(true) {}

    SingleThreadedExecutor::~SingleThreadedExecutor() { clear(); }

    void SingleThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.emplace_back(node);
      }
      else
      {
        throw std::runtime_error("Error: Node pointer is null, cannot add to executor.");
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

    void SingleThreadedExecutor::clear()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto &node : nodes_)
      {
        if (node != nullptr)
        {
          if (!node->closed_.load())
          {
            node->shutdown();
          }
        }
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
      while (!global_stop_flag.load() && !stop_flag_.load())
      {
        bool did_work = false;
        // Snapshot the node list so callbacks run without holding mutex_;
        // otherwise cancel()/add_node() from a callback would deadlock.
        std::vector<Node::SharedPtr> nodes_snapshot;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          nodes_snapshot = nodes_;
        }
        for (const auto &node : nodes_snapshot)
        {
          if (node != nullptr && !node->closed_.load())
          {
            if (node->try_spin_some()) did_work = true;
          }
        }
        // Adaptive sleep: re-poll immediately if work was done, otherwise yield briefly.
        if (!did_work)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }

      if (global_stop_flag.load())
      {
        clear();
      }
    }

    void SingleThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& node : nodes_)
      {
        if (node != nullptr)
        {
          if (!node->closed_.load())
          {
            lwrcl::spin_some(node);
          }
        }
      }
    }

    MultiThreadedExecutor::MultiThreadedExecutor() : stop_flag_(true) {}

    MultiThreadedExecutor::~MultiThreadedExecutor() { clear(); }

    void MultiThreadedExecutor::add_node(Node::SharedPtr node)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node != nullptr)
      {
        nodes_.emplace_back(node);
      }
      else
      {
        throw std::runtime_error("Error: Node pointer is null, cannot add to executor.");
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
      for (const auto& node : nodes_)
      {
        if (node != nullptr)
        {
          if (!node->closed_.load())
          {
            node->stop_spin();
          }
        }
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
          if (!node->closed_.load())
          {
            node->shutdown();
          }
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
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_flag_.store(false);
        for (auto node : nodes_)
        {
          threads_.emplace_back([node]()
                                {
        if (node != nullptr) {
          if (!node->closed_.load()) {
            lwrcl::spin(node);
          }
        } else {
          std::cerr << "[WARN] Node pointer is null in executor spin." << std::endl;
        } });
        }
      }

      while (true)
      {
        std::thread thread;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (threads_.empty())
          {
            break;
          }
          thread = std::move(threads_.back());
          threads_.pop_back();
        }

        if (thread.joinable())
        {
          thread.join();
        }
      }

      if (global_stop_flag.load())
      {
        clear();
      }
    }

    void MultiThreadedExecutor::spin_some()
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto& node : nodes_)
      {
        if (node != nullptr && !node->closed_.load())
        {
          lwrcl::spin_some(node);
        }
      }
    }

    int MultiThreadedExecutor::get_number_of_threads() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return static_cast<int>(threads_.size());
    }
  } // namespace executors

  Time::Time() : nanoseconds_(0) {}
  Time::Time(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Time::Time(int32_t seconds, uint32_t nanoseconds)
      : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds)
  {
  }
  int64_t Time::nanoseconds() const { return nanoseconds_; }
  double Time::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }
  Time Time::operator+(const Duration &rhs) const { return Time(nanoseconds_ + rhs.nanoseconds()); }
  Time Time::operator-(const Duration &rhs) const { return Time(nanoseconds_ - rhs.nanoseconds()); }
  Duration Time::operator-(const Time &rhs) const
  {
    return Duration(nanoseconds_ - rhs.nanoseconds_);
  }
  bool Time::operator==(const Time &rhs) const { return nanoseconds_ == rhs.nanoseconds_; }
  bool Time::operator!=(const Time &rhs) const { return !(*this == rhs); }
  bool Time::operator<(const Time &rhs) const { return nanoseconds_ < rhs.nanoseconds_; }
  bool Time::operator<=(const Time &rhs) const { return nanoseconds_ <= rhs.nanoseconds_; }
  bool Time::operator>(const Time &rhs) const { return nanoseconds_ > rhs.nanoseconds_; }
  bool Time::operator>=(const Time &rhs) const { return nanoseconds_ >= rhs.nanoseconds_; }

  Duration::Duration() : nanoseconds_(0) {}
  Duration::Duration(int64_t nanoseconds) : nanoseconds_(nanoseconds) {}
  Duration::Duration(int32_t seconds, uint32_t nanoseconds)
      : nanoseconds_(static_cast<int64_t>(seconds) * 1000000000 + nanoseconds)
  {
  }
  int64_t Duration::nanoseconds() const { return nanoseconds_; }
  double Duration::seconds() const { return static_cast<double>(nanoseconds_) / 1e9; }
  Duration Duration::operator+(const Duration &rhs) const
  {
    return Duration(nanoseconds_ + rhs.nanoseconds());
  }
  Duration Duration::operator-(const Duration &rhs) const
  {
    return Duration(nanoseconds_ - rhs.nanoseconds());
  }
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
  Rate::Rate(double rate_hz, Clock::SharedPtr clock)
      : Rate(period_from_rate_hz(rate_hz), std::move(clock))
  {
  }

  Rate::Rate(const Duration &period, Clock::SharedPtr clock)
      : period_(period),
        clock_(clock ? clock : std::make_shared<Clock>(Clock::ClockType::SYSTEM_TIME)),
        next_time_(clock_->now())
  {
    if (period_.nanoseconds() <= 0)
    {
      throw std::invalid_argument("period must be greater than zero");
    }
  }

  bool Rate::sleep()
  {
    auto now = clock_->now();
    auto next_interval = next_time_ + period_;
    if (now < next_time_)
    {
      next_interval = now + period_;
    }
    next_time_ = next_time_ + period_;

    if (next_interval <= now)
    {
      if (now > next_interval + period_)
      {
        next_time_ = now + period_;
      }
      return false;
    }

    const int64_t sleep_ns = (next_interval - now).nanoseconds();
    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
    return true;
  }

  Clock::ClockType Rate::get_type() const { return clock_->get_clock_type(); }

  void Rate::reset() { next_time_ = clock_->now(); }

  std::chrono::nanoseconds Rate::period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

  // WallRate implementation
  WallRate::WallRate(double rate_hz)
      : WallRate(period_from_rate_hz(rate_hz))
  {
  }

  WallRate::WallRate(const Duration &period)
      : period_(period),
        clock_(std::make_shared<Clock>(Clock::ClockType::STEADY_TIME)),
        next_time_(clock_->now())
  {
    if (period_.nanoseconds() <= 0)
    {
      throw std::invalid_argument("period must be greater than zero");
    }
  }

  bool WallRate::sleep()
  {
    auto now = clock_->now();
    auto next_interval = next_time_ + period_;
    if (now < next_time_)
    {
      next_interval = now + period_;
    }
    next_time_ = next_time_ + period_;

    if (next_interval <= now)
    {
      if (now > next_interval + period_)
      {
        next_time_ = now + period_;
      }
      return false;
    }

    const int64_t sleep_ns = (next_interval - now).nanoseconds();
    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
    return true;
  }

  Clock::ClockType WallRate::get_type() const { return clock_->get_clock_type(); }

  void WallRate::reset() { next_time_ = clock_->now(); }

  std::chrono::nanoseconds WallRate::period() const
  {
    return std::chrono::nanoseconds(period_.nanoseconds());
  }

  // Node constructors - Adaptive AUTOSAR backend uses AutosarDomainParticipant
  Node::Node(int domain_id)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(
            static_cast<uint32_t>(domain_id)))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_("lwrcl_default_node")
      , namespace_("")
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(int domain_id, const std::string &name)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(
            static_cast<uint32_t>(domain_id)))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_("")
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(int domain_id, const std::string &name, const std::string &ns)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(
            static_cast<uint32_t>(domain_id)))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_(ns)
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(int domain_id, const std::string &name, const std::string &ns, const NodeOptions &options)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(
            static_cast<uint32_t>(domain_id)))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_(ns)
      , node_options_(options)
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(const std::string &name)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(0))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_("")
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(const std::string &name, const std::string &ns)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(0))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_(ns)
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(const std::string &name, const std::string &ns, const NodeOptions &options)
      : closed_(false)
      , participant_(std::make_shared<AutosarDomainParticipant>(0))
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_(ns)
      , node_options_(options)
      , stop_flag_(false)
      , participant_owned_(true)
      , parameters_()
  {
  }

  Node::Node(std::shared_ptr<AutosarDomainParticipant> participant)
      : closed_(false)
      , participant_(participant)
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_("lwrcl_default_node")
      , namespace_("")
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(false)
      , parameters_()
  {
    if (!participant_)
    {
      throw std::runtime_error("Domain participant pointer is null");
    }
  }

  Node::Node(
      std::shared_ptr<AutosarDomainParticipant> participant, const std::string &name)
      : closed_(false)
      , participant_(participant)
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_("")
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(false)
      , parameters_()
  {
    if (!participant_)
    {
      throw std::runtime_error("Domain participant pointer is null");
    }
  }

  Node::Node(
      std::shared_ptr<AutosarDomainParticipant> participant, const std::string &name, const std::string &ns)
      : closed_(false)
      , participant_(participant)
      , callback_mutex_(std::make_shared<std::mutex>())
      , node_cv_(std::make_shared<std::condition_variable>())
      , node_cv_mutex_(std::make_shared<std::mutex>())
      , node_data_pending_(std::make_shared<std::atomic<bool>>(false))
      , clock_(std::make_unique<Clock>())
      , name_(name)
      , namespace_(ns)
      , node_options_()
      , stop_flag_(false)
      , participant_owned_(false)
      , parameters_()
  {
    if (!participant_)
    {
      throw std::runtime_error("Domain participant pointer is null");
    }
  }

  Node::~Node() { shutdown(); }

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

  std::shared_ptr<Node> Node::make_shared(
      std::shared_ptr<AutosarDomainParticipant> participant)
  {
    auto node = std::shared_ptr<Node>(new Node(participant), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<Node> Node::make_shared(
      std::shared_ptr<AutosarDomainParticipant> participant, const std::string &name)
  {
    auto node = std::shared_ptr<Node>(new Node(participant, name), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name, const std::string &ns)
  {
    auto node = std::shared_ptr<Node>(new Node(domain_id, name, ns), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<Node> Node::make_shared(int domain_id, const std::string &name, const std::string &ns, const NodeOptions &options)
  {
    auto node = std::shared_ptr<Node>(new Node(domain_id, name, ns, options), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<Node> Node::make_shared(const std::string &name, const std::string &ns)
  {
    auto node = std::shared_ptr<Node>(new Node(name, ns), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<Node> Node::make_shared(const std::string &name, const std::string &ns, const NodeOptions &options)
  {
    auto node = std::shared_ptr<Node>(new Node(name, ns, options), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<Node> Node::make_shared(
      std::shared_ptr<AutosarDomainParticipant> participant, const std::string &name, const std::string &ns)
  {
    auto node = std::shared_ptr<Node>(new Node(participant, name, ns), [](Node *node)
                                      { delete node; });
    return node;
  }

  std::shared_ptr<AutosarDomainParticipant> Node::get_participant() const
  {
    return participant_;
  }

  const std::string &Node::get_name() const { return name_; }

  const std::string &Node::get_namespace() const { return namespace_; }

  std::string Node::get_fully_qualified_name() const {
    if (namespace_.empty()) {
      return "/" + name_;
    }
    std::string ns = namespace_;
    if (ns.front() != '/') {
      ns = "/" + ns;
    }
    if (ns.back() == '/') {
      ns.pop_back();
    }
    return ns + "/" + name_;
  }

  const Node::NodeOptions &Node::get_node_options() const { return node_options_; }

  Logger Node::get_logger() const { return Logger(name_); }

  void Node::spin()
  {
    stop_flag_ = false;

    std::vector<std::shared_ptr<ISubscription>> subs;
    for (auto &sub : subscription_list_) subs.push_back(sub);

    // Register shared wakeup condition variable with all subscriptions.
    // Stops per-subscription threads; callbacks are invoked directly.
    for (auto &sub : subs) sub->add_to_waitset(node_cv_, node_cv_mutex_, node_data_pending_);

    while (!closed_.load() && !global_stop_flag.load() && !stop_flag_.load())
    {
      std::vector<std::shared_ptr<ITimerBase>> timers;
      {
        std::lock_guard<std::mutex> lock(timer_list_mutex_);
        for (auto &timer : timer_list_) timers.push_back(timer);
      }
      {
        std::unique_lock<std::mutex> lk(*node_cv_mutex_);
        node_cv_->wait_for(lk, timer_wait_timeout(timers),
            [this]() { return node_data_pending_->load() || closed_.load()
                        || global_stop_flag.load() || stop_flag_.load(); });
      }
      if (stop_flag_.load() || global_stop_flag.load()) break;
      if (node_data_pending_->exchange(false))
      {
        for (auto &sub : subs)
        {
          try
          {
            sub->invoke_if_data();
          }
          catch (const std::exception &e)
          {
            std::cerr << "Exception in Node::spin: " << e.what() << std::endl;
          }
          catch (...)
          {
            std::cerr << "Unknown exception in Node::spin." << std::endl;
          }
        }
      }
      for (auto &timer : timers)
      {
        try
        {
          timer->execute_if_ready();
        }
        catch (const std::exception &e)
        {
          std::cerr << "Exception in timer execution: " << e.what() << std::endl;
        }
        catch (...)
        {
          std::cerr << "Unknown exception in timer execution." << std::endl;
        }
      }
    }

    if (global_stop_flag.load())
    {
      shutdown();
    }
  }

  void Node::stop_spin()
  {
    stop_flag_ = true;
    // Wake the spin() cv so it exits immediately.
    if (node_cv_) node_cv_->notify_all();
  }

  bool Node::try_spin_some()
  {
    bool did_work = false;
    for (auto &sub : subscription_list_)
    {
      if (sub->invoke_if_data())
      {
        did_work = true;
      }
    }
    std::vector<std::shared_ptr<ITimerBase>> timers;
    {
      std::lock_guard<std::mutex> lock(timer_list_mutex_);
      for (auto &timer : timer_list_) timers.push_back(timer);
    }
    for (auto &timer : timers)
    {
      if (timer->execute_if_ready())
      {
        did_work = true;
      }
    }
    return did_work;
  }

  void Node::spin_some()
  {
    try_spin_some();
  }

  void Node::shutdown()
  {
    if (closed_.exchange(true)) return;
    stop_spin();
    publisher_list_.clear();
    for (auto &subscriber : subscription_list_)
    {
      subscriber->stop();
    }
    subscription_list_.clear();
    {
      std::lock_guard<std::mutex> lock(timer_list_mutex_);
      for (auto &timer : timer_list_)
      {
        std::static_pointer_cast<TimerBase>(timer)->stop();
      }
      timer_list_.clear();
    }

    for (auto &service : service_list_)
    {
      service->stop();
    }
    service_list_.clear();
    for (auto &client : client_list_)
    {
      client->stop();
    }
    client_list_.clear();
  }

  Clock::SharedPtr Node::get_clock() { return clock_; }

  void Node::set_parameters(const std::vector<std::shared_ptr<ParameterBase>> &parameters)
  {
    for (const auto &param : parameters)
    {
      std::string node_name = this->get_name();
      std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
      std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);
      if (!param)
      {
        throw std::runtime_error("Parameter pointer is null");
      }
      auto parameter = std::dynamic_pointer_cast<Parameter>(param);
      if (!parameter)
      {
        throw std::runtime_error("Parameter type is invalid");
      }
      std::string param_name = parameter->get_name();

      auto node_it = node_parameters.find(node_name);
      if (node_it != node_parameters.end())
      {
        Parameters &params = node_it->second;

        if (params.find(param_name) != params.end())
        {
          params[param_name] = *parameter;
          parameters_[param_name] = *parameter;

          std::cout << "Parameter updated: " << param_name << std::endl;
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
    {
      base_params.push_back(std::make_shared<Parameter>(param));
    }
    set_parameters(base_params);
  }

  void Node::declare_parameter(const std::string &name, const bool &default_value)
  {
    std::string node_name = this->get_name();
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);

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
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);
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
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);
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
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);
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
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);
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
    std::lock_guard<std::mutex> parameters_lock(parameters_mutex_);
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

  namespace
  {
    std::atomic<bool> ara_runtime_initialized{false};

    // ara::core::Deinitialize() must run only after every ara::com object
    // (proxies/skeletons held by nodes, publishers, subscriptions) has been
    // destroyed. lwrcl::shutdown() is called while nodes are still alive
    // (rclcpp idiom: shutdown() stops spinning, objects are destroyed later),
    // so deinitialization is deferred to process exit and runs exactly once.
    void deinitialize_ara_runtime()
    {
      if (ara_runtime_initialized.exchange(false))
      {
        try
        {
          ara::core::Deinitialize();
        }
        catch (const std::exception &e)
        {
          std::cerr << "[WARN] ara::core::Deinitialize() failed: " << e.what() << std::endl;
        }
        catch (...)
        {
          std::cerr << "[WARN] ara::core::Deinitialize() failed with unknown exception." << std::endl;
        }
      }
    }
  }

  void init(int argc, char *argv[])
  {
    global_stop_flag.store(false);

    const char *skip_ara_core_init = std::getenv("LWRCL_SKIP_ARA_CORE_INIT");
    const bool should_skip_ara_core_init =
        skip_ara_core_init != nullptr && std::string(skip_ara_core_init) == "1";

    // Initialize Adaptive AUTOSAR runtime (once per process). Unit tests can
    // skip ara::core lifecycle setup to avoid platform-runtime shutdown hooks
    // while still exercising the lwrcl/ara::com communication path.
    if (!should_skip_ara_core_init && !ara_runtime_initialized.load())
    {
      auto ara_result = ara::core::Initialize();
      if (!ara_result.HasValue()) {
        std::cerr << "[WARN] ara::core::Initialize() failed, continuing without AUTOSAR runtime." << std::endl;
      }
      else
      {
        ara_runtime_initialized.store(true);
        std::atexit(deinitialize_ara_runtime);
      }
    }

    if (std::signal(SIGINT, lwrcl_signal_handler) == SIG_ERR)
    {
      throw std::runtime_error("Failed to set signal handler.");
    }
    if (std::signal(SIGTERM, lwrcl_signal_handler) == SIG_ERR)
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
      if (!node->closed_.load())
      {
        node->spin();
      }
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
      if (!node->closed_.load())
      {
        node->spin_some();
      }
    }
    else
    {
      throw std::runtime_error("Node pointer is invalid!");
    }
  }

  void shutdown()
  {
    global_stop_flag.store(true);
    // Note: ara::core::Deinitialize() is intentionally NOT called here.
    // Nodes (and their ara::com proxies/skeletons) are typically still alive
    // when shutdown() is invoked; deinitialization happens at process exit
    // (registered via std::atexit in init()).
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
      else if (
          found_ros_args &&
          (std::string(argv[i]) == "--params-file" || std::string(argv[i]) == "--param-file"))
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

  void load_parameters(const std::string &file_path)
  {
    YAML::Node config = YAML::LoadFile(file_path);
    std::lock_guard<std::mutex> node_parameters_lock(node_parameters_mutex);

    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
    {
      std::string node_name = it->first.as<std::string>();
      YAML::Node parameters = it->second["ros__parameters"];

      Parameters params;
      for (YAML::const_iterator param_it = parameters.begin(); param_it != parameters.end();
           ++param_it)
      {
        std::string param_name = param_it->first.as<std::string>();
        params[param_name] = make_parameter_from_yaml_scalar(param_name, param_it->second);
      }
      node_parameters[node_name] = params;
    }
  }

  void sleep_for(const lwrcl::Duration &duration)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(duration.nanoseconds()));
  }

  // Default QoS profile
  const RMWQoSProfile rmw_qos_profile_default = {
      10, // depth
      RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_sensor_data = {
      5, // depth
      RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::BEST_EFFORT,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_parameters = {
      1000, // depth
      RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_services_default = {
      10, // depth
      RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

  const RMWQoSProfile rmw_qos_profile_parameter_events = {
      1000, // depth
      RMWQoSHistoryPolicy::KEEP_LAST, RMWQoSReliabilityPolicy::RELIABLE,
      RMWQoSDurabilityPolicy::VOLATILE};

} // namespace lwrcl
