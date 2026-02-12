/**
 * @file video_stream_pipeline.cpp
 * @brief Advanced example: H265 video stream simulation pipeline
 *
 * Demonstrates:
 *  - Custom IDL message types (VideoStreamStatus)
 *  - Parameter-driven configuration (--params-file)
 *  - Shared DomainParticipant across multiple nodes
 *  - SingleThreadedExecutor with multiple nodes
 *  - Multiple QoS profiles (SensorDataQoS for video, ReliableQoS for status)
 *  - Wall timers for periodic encoding simulation
 *  - Namespace-aware topic routing
 *  - Logger usage with structured output
 */

#include <chrono>
#include <memory>
#include <random>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

// Custom message generated from VideoStreamStatus.idl
#include "advanced_msgs/msg/video_stream_status.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// ==========================================================================
// Node 1: VideoEncoder – Captures synthetic frames and publishes compressed
//         H265-like packets along with stream status reports.
// ==========================================================================
class VideoEncoder : public rclcpp::Node
{
public:
  VideoEncoder(const std::string &name, const std::string &ns)
    : Node(name, ns), frame_count_(0), bytes_total_(0)
  {
    // --- Parameters ---
    declare_parameter("width", 1920);
    declare_parameter("height", 1080);
    declare_parameter("target_fps", 30.0);
    declare_parameter("keyframe_interval", 30);
    declare_parameter("stream_name", std::string("main_camera"));

    get_parameter("width", width_);
    get_parameter("height", height_);
    get_parameter("target_fps", target_fps_);
    get_parameter("keyframe_interval", keyframe_interval_);
    get_parameter("stream_name", stream_name_);

    RCLCPP_INFO(get_logger(), "Encoder config: %dx%d @ %.1f fps, stream=%s",
                width_, height_, target_fps_, stream_name_.c_str());

    // --- Publishers ---
    // Video frames use best-effort QoS (sensor-data pattern)
    rclcpp::SensorDataQoS video_qos;
    video_pub_ = create_publisher<sensor_msgs::msg::Image>("video_raw", video_qos);

    // Status uses reliable QoS for guaranteed delivery
    rclcpp::ReliableQoS status_qos;
    status_pub_ = create_publisher<advanced_msgs::msg::VideoStreamStatus>(
      "/diagnostics/video_status", status_qos);

    // --- Encode timer ---
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / target_fps_));
    encode_timer_ = create_wall_timer(period, std::bind(&VideoEncoder::encode_frame, this));

    // --- Status report timer (1 Hz) ---
    status_timer_ = create_wall_timer(1s, std::bind(&VideoEncoder::publish_status, this));

    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "VideoEncoder ready — publishing to '%s'",
                video_pub_->get_topic_name().c_str());
  }

private:
  void encode_frame()
  {
    frame_count_++;
    bool is_keyframe = (frame_count_ % keyframe_interval_) == 1;

    // Simulate compressed frame data — keyframes are larger
    size_t frame_size = is_keyframe ? (width_ * height_ / 4) : (width_ * height_ / 40);

    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header().stamp().sec() = static_cast<int32_t>(frame_count_ / static_cast<uint64_t>(target_fps_));
    msg->header().stamp().nanosec() = 0;
    msg->header().frame_id() = stream_name_;
    msg->width() = static_cast<uint32_t>(width_);
    msg->height() = static_cast<uint32_t>(height_);
    msg->encoding() = is_keyframe ? "h265_keyframe" : "h265_inter";
    msg->is_bigendian() = false;
    msg->step() = static_cast<uint32_t>(frame_size);

    // Fill with simulated compressed data
    msg->data().resize(frame_size, static_cast<uint8_t>(frame_count_ % 256));
    bytes_total_ += frame_size;

    video_pub_->publish(msg);

    if (is_keyframe) {
      RCLCPP_INFO(get_logger(), "[frame %lu] KEYFRAME, size=%zu bytes",
                  frame_count_, frame_size);
    }
  }

  void publish_status()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time_).count();
    double actual_fps = elapsed > 0 ? frame_count_ / elapsed : 0.0;
    double bitrate = elapsed > 0 ? (bytes_total_ * 8.0 / 1000.0) / elapsed : 0.0;

    auto status = std::make_shared<advanced_msgs::msg::VideoStreamStatus>();
    status->header().stamp().sec() = static_cast<int32_t>(elapsed);
    status->header().frame_id() = "encoder";
    status->stream_name() = stream_name_;
    status->width() = width_;
    status->height() = height_;
    status->encoding() = "h265";
    status->frame_count() = frame_count_;
    status->bytes_total() = bytes_total_;
    status->fps_actual() = actual_fps;
    status->fps_target() = target_fps_;
    status->bitrate_kbps() = bitrate;
    status->is_keyframe() = (frame_count_ % keyframe_interval_) == 1;
    status->quality_level() = 85;

    status_pub_->publish(status);

    RCLCPP_INFO(get_logger(), "Status: %.1f fps (target %.1f), %.1f kbps, %lu frames",
                actual_fps, target_fps_, bitrate, frame_count_);
  }

  // Parameters
  int width_, height_, keyframe_interval_;
  double target_fps_;
  std::string stream_name_;

  // State
  uint64_t frame_count_;
  uint64_t bytes_total_;
  std::chrono::steady_clock::time_point start_time_;

  // ROS entities
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr video_pub_;
  rclcpp::Publisher<advanced_msgs::msg::VideoStreamStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr encode_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

// ==========================================================================
// Node 2: StreamMonitor – Subscribes to video + status, tracks statistics,
//         and publishes aggregated reports.
//         Shares DomainParticipant with the encoder for efficiency.
// ==========================================================================
class StreamMonitor : public rclcpp::Node
{
public:
  StreamMonitor(std::shared_ptr<rclcpp::DomainParticipant> participant,
                const std::string &name, const std::string &ns)
    : Node(participant, name, ns), frames_received_(0), total_latency_ms_(0.0)
  {
    // Subscribe to video stream (best effort — matches publisher)
    rclcpp::SensorDataQoS video_qos;
    video_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "video_raw", video_qos,
      std::bind(&StreamMonitor::on_video, this, _1));

    // Subscribe to status reports (reliable — matches publisher, absolute topic)
    rclcpp::ReliableQoS status_qos;
    status_sub_ = create_subscription<advanced_msgs::msg::VideoStreamStatus>(
      "/diagnostics/video_status", status_qos,
      std::bind(&StreamMonitor::on_status, this, _1));

    // Publish aggregated report
    report_pub_ = create_publisher<std_msgs::msg::String>("/diagnostics/report", 10);

    // Report timer (2s interval)
    report_timer_ = create_wall_timer(2s, std::bind(&StreamMonitor::publish_report, this));

    RCLCPP_INFO(get_logger(), "StreamMonitor ready — shared participant, monitoring '%s'",
                video_sub_->get_topic_name().c_str());
  }

private:
  void on_video(sensor_msgs::msg::Image::SharedPtr msg)
  {
    frames_received_++;
    last_encoding_ = msg->encoding();

    // Simulate latency measurement
    auto now = std::chrono::steady_clock::now();
    if (frames_received_ > 1) {
      double dt = std::chrono::duration<double, std::milli>(now - last_frame_time_).count();
      total_latency_ms_ += dt;
    }
    last_frame_time_ = now;
  }

  void on_status(advanced_msgs::msg::VideoStreamStatus::SharedPtr status)
  {
    last_status_ = status;
    RCLCPP_DEBUG(get_logger(), "Status: %s — %.1f fps, %.1f kbps",
                 status->stream_name().c_str(),
                 status->fps_actual(), status->bitrate_kbps());
  }

  void publish_report()
  {
    auto report = std::make_shared<std_msgs::msg::String>();
    std::string text = "=== Stream Monitor Report ===\n";
    text += "  Frames received: " + std::to_string(frames_received_) + "\n";

    if (frames_received_ > 1) {
      double avg_latency = total_latency_ms_ / (frames_received_ - 1);
      text += "  Avg inter-frame: " + std::to_string(avg_latency) + " ms\n";
    }

    if (last_status_) {
      text += "  Stream: " + last_status_->stream_name() + "\n";
      text += "  Resolution: " + std::to_string(last_status_->width())
              + "x" + std::to_string(last_status_->height()) + "\n";
      text += "  FPS: " + std::to_string(last_status_->fps_actual())
              + " / " + std::to_string(last_status_->fps_target()) + "\n";
      text += "  Bitrate: " + std::to_string(last_status_->bitrate_kbps()) + " kbps\n";
      text += "  Total bytes: " + std::to_string(last_status_->bytes_total()) + "\n";
    }

    report->data() = text;
    report_pub_->publish(report);
    RCLCPP_INFO(get_logger(), "Published report (%lu frames received)", frames_received_);
  }

  // State
  uint64_t frames_received_;
  double total_latency_ms_;
  std::string last_encoding_;
  std::chrono::steady_clock::time_point last_frame_time_;
  advanced_msgs::msg::VideoStreamStatus::SharedPtr last_status_;

  // ROS entities
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_sub_;
  rclcpp::Subscription<advanced_msgs::msg::VideoStreamStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

// ==========================================================================
// Main — Wire up encoder + monitor with shared participant and executor
// ==========================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create encoder in "camera" namespace
  auto encoder = std::make_shared<VideoEncoder>("encoder", "camera");

  // Create monitor sharing the same DDS participant (efficient, zero-copy capable)
  auto monitor = std::make_shared<StreamMonitor>(
    encoder->get_participant(), "monitor", "camera");

  // Run both nodes in a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(encoder);
  executor.add_node(monitor);

  RCLCPP_INFO(encoder->get_logger(), "Pipeline started. Press Ctrl+C to stop.");
  executor.spin();
  executor.cancel();

  rclcpp::shutdown();
  return 0;
}
