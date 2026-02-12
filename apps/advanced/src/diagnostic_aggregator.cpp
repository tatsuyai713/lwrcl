/**
 * @file diagnostic_aggregator.cpp
 * @brief Advanced example: Multi-node diagnostic monitoring system
 *
 * Demonstrates:
 *  - Custom IDL message types (DiagnosticEntry with nested KeyValue)
 *  - Service server and client (request diagnostics on demand)
 *  - spin_until_future_complete for synchronous service calls
 *  - Multiple timers with different periods
 *  - YAML parameter loading (--params-file)
 *  - Namespace isolation for subsystems
 *  - MultiThreadedExecutor for parallel node processing
 *  - TransientLocal durability for late-joining subscribers
 */

#include <chrono>
#include <memory>
#include <vector>
#include <random>
#include <mutex>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

// Custom message generated from DiagnosticEntry.idl
#include "advanced_msgs/msg/diagnostic_entry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// ==========================================================================
// Node 1: HardwareMonitor — Simulates hardware sensor readings and publishes
//         diagnostic entries periodically.
// ==========================================================================
class HardwareMonitor : public rclcpp::Node
{
public:
  HardwareMonitor(const std::string &name, const std::string &ns)
    : Node(name, ns), tick_count_(0), rng_(42)
  {
    // Parameters
    declare_parameter("hardware_id", std::string("sensor_board_01"));
    declare_parameter("report_interval_ms", 500);
    declare_parameter("warn_threshold_temp", 75.0);
    declare_parameter("error_threshold_temp", 90.0);

    get_parameter("hardware_id", hardware_id_);
    get_parameter("report_interval_ms", report_interval_ms_);
    get_parameter("warn_threshold_temp", warn_temp_);
    get_parameter("error_threshold_temp", error_temp_);

    RCLCPP_INFO(get_logger(), "HW Monitor [%s]: interval=%dms, warn=%.0f°C, error=%.0f°C",
                hardware_id_.c_str(), report_interval_ms_, warn_temp_, error_temp_);

    // TransientLocal QoS — late joiners get the last diagnostic status
    rclcpp::QoS diag_qos(1);
    diag_qos.durability(rclcpp::QoS::DurabilityPolicy::TRANSIENT_LOCAL)
            .reliability(rclcpp::QoS::ReliabilityPolicy::RELIABLE);

    diag_pub_ = create_publisher<advanced_msgs::msg::DiagnosticEntry>(
      "diagnostics", diag_qos);

    // Heartbeat timer
    auto period = std::chrono::milliseconds(report_interval_ms_);
    monitor_timer_ = create_wall_timer(period, std::bind(&HardwareMonitor::report, this));
  }

private:
  void report()
  {
    tick_count_++;
    auto entry = std::make_shared<advanced_msgs::msg::DiagnosticEntry>();

    // Timestamp
    auto now = get_clock()->now();
    entry->header().stamp().sec() = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    entry->header().stamp().nanosec() = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    entry->header().frame_id() = "hw_monitor";
    entry->node_name() = get_fully_qualified_name();
    entry->hardware_id() = hardware_id_;

    // Simulate sensor readings
    std::uniform_real_distribution<double> temp_dist(40.0, 95.0);
    std::uniform_real_distribution<double> voltage_dist(4.8, 5.2);
    std::uniform_int_distribution<int>     mem_dist(40, 95);

    double temperature = temp_dist(rng_);
    double voltage = voltage_dist(rng_);
    int mem_usage = mem_dist(rng_);

    // Determine level
    if (temperature >= error_temp_) {
      entry->level() = 2; // ERROR
      entry->message() = "CRITICAL: Temperature exceeded error threshold!";
    } else if (temperature >= warn_temp_) {
      entry->level() = 1; // WARN
      entry->message() = "WARNING: Temperature approaching limit";
    } else {
      entry->level() = 0; // OK
      entry->message() = "All systems nominal";
    }

    // Key-value telemetry data
    auto &values = entry->values();
    {
      advanced_msgs::msg::KeyValue kv;
      kv.key() = "temperature_c";
      kv.value() = std::to_string(temperature);
      values.push_back(kv);
    }
    {
      advanced_msgs::msg::KeyValue kv;
      kv.key() = "voltage_v";
      kv.value() = std::to_string(voltage);
      values.push_back(kv);
    }
    {
      advanced_msgs::msg::KeyValue kv;
      kv.key() = "memory_pct";
      kv.value() = std::to_string(mem_usage);
      values.push_back(kv);
    }
    {
      advanced_msgs::msg::KeyValue kv;
      kv.key() = "uptime_ticks";
      kv.value() = std::to_string(tick_count_);
      values.push_back(kv);
    }

    diag_pub_->publish(entry);

    const char *level_str[] = {"OK", "WARN", "ERROR", "STALE"};
    RCLCPP_INFO(get_logger(), "[%s] %s — temp=%.1f°C, volt=%.2fV, mem=%d%%",
                hardware_id_.c_str(), level_str[entry->level()],
                temperature, voltage, mem_usage);
  }

  // Params
  std::string hardware_id_;
  int report_interval_ms_;
  double warn_temp_, error_temp_;

  // State
  uint64_t tick_count_;
  std::mt19937 rng_;

  // ROS
  rclcpp::Publisher<advanced_msgs::msg::DiagnosticEntry>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

// ==========================================================================
// Node 2: DiagnosticAggregator — Collects diagnostics from multiple hardware
//         monitors, provides a service to query current system health.
// ==========================================================================
class DiagnosticAggregator : public rclcpp::Node
{
public:
  DiagnosticAggregator()
    : Node("aggregator", "diagnostics")
  {
    // Subscribe to all hardware monitors' diagnostics (absolute topic with
    // different namespace prefixes would require separate subscriptions;
    // for simplicity we subscribe to the same namespace topics)
    rclcpp::QoS diag_qos(10);
    diag_qos.durability(rclcpp::QoS::DurabilityPolicy::TRANSIENT_LOCAL)
            .reliability(rclcpp::QoS::ReliabilityPolicy::RELIABLE);

    sub_hw1_ = create_subscription<advanced_msgs::msg::DiagnosticEntry>(
      "/hw_subsystem/diagnostics", diag_qos,
      std::bind(&DiagnosticAggregator::on_diag, this, _1));

    sub_hw2_ = create_subscription<advanced_msgs::msg::DiagnosticEntry>(
      "/sensors/diagnostics", diag_qos,
      std::bind(&DiagnosticAggregator::on_diag, this, _1));

    // Service: request current aggregated status
    service_ = create_service<sensor_msgs::srv::SetCameraInfo>(
      "get_system_health",
      std::bind(&DiagnosticAggregator::handle_health_request, this, _1, _2));

    // Summary publisher (5s interval)
    summary_pub_ = create_publisher<std_msgs::msg::String>("summary", 10);
    summary_timer_ = create_wall_timer(5s, std::bind(&DiagnosticAggregator::publish_summary, this));

    RCLCPP_INFO(get_logger(), "DiagnosticAggregator ready — service: get_system_health");
  }

private:
  void on_diag(advanced_msgs::msg::DiagnosticEntry::SharedPtr entry)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Store latest entry per hardware_id
    latest_[entry->hardware_id()] = entry;

    if (entry->level() >= 2) {
      RCLCPP_WARN(get_logger(), "*** ALERT from [%s]: %s",
                  entry->hardware_id().c_str(), entry->message().c_str());
    }
  }

  void handle_health_request(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> /*request*/,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // Use the response's success field to indicate overall health
    bool all_ok = true;
    for (auto &pair : latest_) {
      if (pair.second->level() >= 2) {
        all_ok = false;
        break;
      }
    }
    response->success() = all_ok;
    // Pack summary into status_message
    response->status_message() = build_summary_text();

    RCLCPP_INFO(get_logger(), "Health query result: %s", all_ok ? "HEALTHY" : "DEGRADED");
  }

  void publish_summary()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data() = build_summary_text();
    summary_pub_->publish(msg);
  }

  std::string build_summary_text()
  {
    const char *level_str[] = {"OK", "WARN", "ERROR", "STALE"};
    std::ostringstream oss;
    oss << "=== System Health Summary (" << latest_.size() << " sources) ===\n";
    for (auto &pair : latest_) {
      auto &e = pair.second;
      oss << "  [" << level_str[e->level()] << "] " << e->hardware_id()
          << " — " << e->message() << "\n";
      for (auto &kv : e->values()) {
        oss << "       " << kv.key() << " = " << kv.value() << "\n";
      }
    }
    return oss.str();
  }

  std::mutex mtx_;
  std::map<std::string, advanced_msgs::msg::DiagnosticEntry::SharedPtr> latest_;

  rclcpp::Subscription<advanced_msgs::msg::DiagnosticEntry>::SharedPtr sub_hw1_;
  rclcpp::Subscription<advanced_msgs::msg::DiagnosticEntry>::SharedPtr sub_hw2_;
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

// ==========================================================================
// Node 3: HealthChecker — Periodically queries the aggregator service to
//         demonstrate service client + spin_until_future_complete.
// ==========================================================================
class HealthChecker : public rclcpp::Node
{
public:
  HealthChecker()
    : Node("health_checker", "diagnostics")
  {
    client_ = create_client<sensor_msgs::srv::SetCameraInfo>("get_system_health");

    // Check health every 10 seconds
    check_timer_ = create_wall_timer(10s, std::bind(&HealthChecker::check_health, this));

    RCLCPP_INFO(get_logger(), "HealthChecker ready — will query service every 10s");
  }

private:
  void check_health()
  {
    if (!client_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "Health service not available");
      return;
    }

    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
    auto future = client_->async_send_request(request);

    auto rc = rclcpp::spin_until_future_complete(this->shared_from_this(), future, 5s);
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      RCLCPP_INFO(get_logger(), "System health: %s",
                  response->success() ? "HEALTHY" : "DEGRADED");
      if (!response->status_message().empty()) {
        RCLCPP_INFO(get_logger(), "\n%s", response->status_message().c_str());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Health check timed out");
    }
  }

  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

// ==========================================================================
// Main — Multi-threaded executor with namespace-isolated nodes
// ==========================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Two hardware monitors in different namespaces
  auto hw1 = std::make_shared<HardwareMonitor>("cpu_monitor", "hw_subsystem");
  auto hw2 = std::make_shared<HardwareMonitor>("gpu_monitor", "sensors");

  // Aggregator and health checker in diagnostics namespace
  auto aggregator = std::make_shared<DiagnosticAggregator>();
  auto checker    = std::make_shared<HealthChecker>();

  // Multi-threaded executor — each node gets its own thread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(hw1);
  executor.add_node(hw2);
  executor.add_node(aggregator);
  executor.add_node(checker);

  RCLCPP_INFO(hw1->get_logger(), "Diagnostic system started with %d threads",
              executor.get_number_of_threads());
  executor.spin();
  executor.cancel();

  rclcpp::shutdown();
  return 0;
}
