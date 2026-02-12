/**
 * @file state_machine_controller.cpp
 * @brief Advanced example: Timer-driven state machine with service control
 *
 * Demonstrates:
 *  - Timer control: cancel(), reset(), is_canceled(), is_ready()
 *  - Service-based state transitions (external commands)
 *  - State machine pattern with multiple states
 *  - Multiple publishers for different data depending on state
 *  - Dynamic timer period changes via reset
 *  - Serialization API for message introspection
 *  - Clock time usage for duration tracking
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

// ==========================================================================
// RobotController — A state machine that transitions between states:
//   IDLE → SCANNING → TRACKING → IDLE
// Each state uses different timer cadences and publishes different data.
// Transitions are triggered via a service call.
// ==========================================================================
class RobotController : public rclcpp::Node
{
public:
  enum class State { IDLE, SCANNING, TRACKING };

  RobotController()
    : Node("robot_controller"), state_(State::IDLE), scan_count_(0), track_count_(0)
  {
    // Publishers for each state
    status_pub_ = create_publisher<std_msgs::msg::String>("robot/status", 10);
    scan_pub_   = create_publisher<sensor_msgs::msg::Image>("robot/scan_data", 10);
    track_pub_  = create_publisher<std_msgs::msg::String>("robot/track_target", 10);

    // Timers — start in canceled state; activated per state
    idle_timer_  = create_wall_timer(2s, std::bind(&RobotController::idle_tick, this));
    scan_timer_  = create_wall_timer(100ms, std::bind(&RobotController::scan_tick, this));
    track_timer_ = create_wall_timer(50ms, std::bind(&RobotController::track_tick, this));

    // Start only idle timer
    scan_timer_->cancel();
    track_timer_->cancel();

    // Service to control state transitions
    control_service_ = create_service<sensor_msgs::srv::SetCameraInfo>(
      "robot/control",
      std::bind(&RobotController::handle_command, this, _1, _2));

    // Heartbeat timer (always active)
    heartbeat_timer_ = create_wall_timer(5s, std::bind(&RobotController::heartbeat, this));

    state_start_ = get_clock()->now();

    RCLCPP_INFO(get_logger(), "RobotController initialized — state: IDLE");
    RCLCPP_INFO(get_logger(), "Send commands via 'robot/control' service");
    RCLCPP_INFO(get_logger(), "  Commands: scan, track, idle, status");
  }

private:
  // --- State transition ---
  void transition_to(State new_state)
  {
    State old_state = state_;
    state_ = new_state;
    state_start_ = get_clock()->now();

    // Cancel all work timers
    idle_timer_->cancel();
    scan_timer_->cancel();
    track_timer_->cancel();

    // Activate timer for new state
    switch (new_state) {
      case State::IDLE:
        idle_timer_->reset();
        RCLCPP_INFO(get_logger(), "TRANSITION: %s → IDLE", state_name(old_state));
        break;
      case State::SCANNING:
        scan_count_ = 0;
        scan_timer_->reset();
        RCLCPP_INFO(get_logger(), "TRANSITION: %s → SCANNING", state_name(old_state));
        break;
      case State::TRACKING:
        track_count_ = 0;
        track_timer_->reset();
        RCLCPP_INFO(get_logger(), "TRANSITION: %s → TRACKING", state_name(old_state));
        break;
    }

    publish_status("state_change");
  }

  // --- Timer callbacks ---
  void idle_tick()
  {
    RCLCPP_DEBUG(get_logger(), "IDLE tick — waiting for commands...");
    publish_status("idle_heartbeat");
  }

  void scan_tick()
  {
    scan_count_++;

    // Simulate LIDAR scan data
    auto scan = std::make_shared<sensor_msgs::msg::Image>();
    auto now = get_clock()->now();
    scan->header().stamp().sec() = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    scan->header().stamp().nanosec() = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    scan->header().frame_id() = "lidar_link";
    scan->width() = 360;
    scan->height() = 1;
    scan->encoding() = "32FC1"; // float distances
    scan->step() = 360 * 4;
    scan->data().resize(360 * 4, 0);

    // Fill with simulated range data
    for (int i = 0; i < 360; i++) {
      float range = 1.0f + static_cast<float>(i % 10) * 0.5f;
      std::memcpy(&scan->data()[i * 4], &range, sizeof(float));
    }

    scan_pub_->publish(scan);

    if (scan_count_ % 50 == 0) {
      RCLCPP_INFO(get_logger(), "SCANNING: %lu scans completed", scan_count_);
    }

    // Auto-transition to tracking after 200 scans (20 seconds at 10Hz)
    if (scan_count_ >= 200) {
      RCLCPP_INFO(get_logger(), "Scan complete — target found, transitioning to TRACKING");
      transition_to(State::TRACKING);
    }
  }

  void track_tick()
  {
    track_count_++;

    // Simulate target tracking data
    double angle = track_count_ * 0.1;
    double x = 5.0 + std::sin(angle) * 2.0;
    double y = 3.0 + std::cos(angle) * 1.5;

    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data() = "target: x=" + std::to_string(x)
                + " y=" + std::to_string(y)
                + " confidence=" + std::to_string(0.95 - (track_count_ % 100) * 0.005);
    track_pub_->publish(msg);

    if (track_count_ % 100 == 0) {
      RCLCPP_INFO(get_logger(), "TRACKING: %lu updates, target at (%.2f, %.2f)",
                  track_count_, x, y);
    }

    // Auto-transition to idle after 500 tracking updates (25 seconds at 20Hz)
    if (track_count_ >= 500) {
      RCLCPP_INFO(get_logger(), "Target lost — returning to IDLE");
      transition_to(State::IDLE);
    }
  }

  void heartbeat()
  {
    auto elapsed = get_clock()->now() - state_start_;
    RCLCPP_INFO(get_logger(), "Heartbeat — state: %s for %.1fs | timers: idle=%s scan=%s track=%s",
                state_name(state_),
                elapsed.seconds(),
                idle_timer_->is_ready() ? "ON" : "OFF",
                scan_timer_->is_ready() ? "ON" : "OFF",
                track_timer_->is_ready() ? "ON" : "OFF");
  }

  // --- Service handler ---
  void handle_command(
    const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
    std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response)
  {
    // Use camera_info.distortion_model as a command string
    std::string cmd = request->camera_info().distortion_model();
    RCLCPP_INFO(get_logger(), "Received command: '%s'", cmd.c_str());

    if (cmd == "scan") {
      transition_to(State::SCANNING);
      response->success() = true;
      response->status_message() = "Transitioned to SCANNING";
    } else if (cmd == "track") {
      transition_to(State::TRACKING);
      response->success() = true;
      response->status_message() = "Transitioned to TRACKING";
    } else if (cmd == "idle") {
      transition_to(State::IDLE);
      response->success() = true;
      response->status_message() = "Transitioned to IDLE";
    } else if (cmd == "status") {
      response->success() = true;
      response->status_message() = std::string("Current state: ") + state_name(state_);
    } else {
      response->success() = false;
      response->status_message() = "Unknown command: " + cmd;
    }
  }

  // --- Helpers ---
  void publish_status(const std::string &event)
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data() = "state=" + std::string(state_name(state_))
                + " event=" + event
                + " scan_count=" + std::to_string(scan_count_)
                + " track_count=" + std::to_string(track_count_);
    status_pub_->publish(msg);
  }

  static const char *state_name(State s)
  {
    switch (s) {
      case State::IDLE:     return "IDLE";
      case State::SCANNING: return "SCANNING";
      case State::TRACKING: return "TRACKING";
      default:              return "UNKNOWN";
    }
  }

  // State
  State state_;
  uint64_t scan_count_, track_count_;
  rclcpp::Time state_start_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr track_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr idle_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr track_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Service
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr control_service_;
};

// ==========================================================================
// CommandSender — Sends state commands to the robot controller periodically
//                 to demonstrate service client interaction.
// ==========================================================================
class CommandSender : public rclcpp::Node
{
public:
  CommandSender()
    : Node("command_sender"), cmd_index_(0)
  {
    client_ = create_client<sensor_msgs::srv::SetCameraInfo>("robot/control");

    // Send commands on a schedule
    cmd_timer_ = create_wall_timer(8s, std::bind(&CommandSender::send_next_command, this));

    RCLCPP_INFO(get_logger(), "CommandSender ready — will cycle through commands");
  }

private:
  void send_next_command()
  {
    const std::string commands[] = {"scan", "track", "idle", "status"};
    std::string cmd = commands[cmd_index_ % 4];
    cmd_index_++;

    if (!client_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "robot/control service not available");
      return;
    }

    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
    request->camera_info().distortion_model() = cmd;

    auto future = client_->async_send_request(request);
    auto rc = rclcpp::spin_until_future_complete(shared_from_this(), future, 5s);

    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      auto resp = future.get();
      RCLCPP_INFO(get_logger(), "Command '%s': %s — %s",
                  cmd.c_str(),
                  resp->success() ? "OK" : "FAIL",
                  resp->status_message().c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Command '%s' timed out", cmd.c_str());
    }
  }

  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  size_t cmd_index_;
};

// ==========================================================================
// Main
// ==========================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<RobotController>();
  auto sender = std::make_shared<CommandSender>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(sender);

  RCLCPP_INFO(controller->get_logger(), "State machine system started");
  executor.spin();
  executor.cancel();

  rclcpp::shutdown();
  return 0;
}
