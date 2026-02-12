/**
 * @file adaptive_image_pipeline.cpp
 * @brief Advanced example: Adaptive QoS image pipeline with WaitSet polling
 *
 * Demonstrates:
 *  - WaitSet synchronous polling model (non-callback pattern)
 *  - Multiple QoS tiers for image quality levels
 *  - Rate-controlled publishing loop with WallRate
 *  - Dynamic quality adaptation based on subscriber count
 *  - Clock / Time API for timestamping
 *  - Separate high-quality vs. thumbnail publishers
 *  - MessageInfo metadata from take()
 */

#include <chrono>
#include <memory>
#include <vector>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// ==========================================================================
// Node: AdaptivePublisher — Publishes two image streams at different
//       quality tiers. Adapts frame rate based on subscriber count.
// ==========================================================================
class AdaptivePublisher : public rclcpp::Node
{
public:
  AdaptivePublisher()
    : Node("adaptive_pub", "imaging"), frame_count_(0)
  {
    // Declare parameters
    declare_parameter("base_width", 1280);
    declare_parameter("base_height", 720);
    declare_parameter("max_fps", 30);
    declare_parameter("min_fps", 5);

    get_parameter("base_width", width_);
    get_parameter("base_height", height_);
    get_parameter("max_fps", max_fps_);
    get_parameter("min_fps", min_fps_);

    // High-quality stream: reliable, keep-all for lossless delivery
    rclcpp::QoS hq_qos(rclcpp::KeepAll{});
    hq_qos.reliable();
    hq_pub_ = create_publisher<sensor_msgs::msg::Image>("image_hq", hq_qos);

    // Thumbnail stream: best effort, sensor-data pattern
    rclcpp::SensorDataQoS thumb_qos;
    thumb_pub_ = create_publisher<sensor_msgs::msg::Image>("image_thumbnail", thumb_qos);

    // Status publisher
    status_pub_ = create_publisher<std_msgs::msg::String>("/imaging/status", 10);

    RCLCPP_INFO(get_logger(), "AdaptivePublisher: %dx%d, fps range [%d..%d]",
                width_, height_, min_fps_, max_fps_);
  }

  void run()
  {
    RCLCPP_INFO(get_logger(), "Starting adaptive publishing loop...");

    int current_fps = max_fps_;

    while (rclcpp::ok()) {
      // Adapt FPS based on subscriber count
      int hq_subs = static_cast<int>(hq_pub_->get_subscription_count());
      int thumb_subs = static_cast<int>(thumb_pub_->get_subscription_count());
      int total_subs = hq_subs + thumb_subs;

      // More subscribers → lower FPS to conserve bandwidth
      if (total_subs > 3) {
        current_fps = min_fps_;
      } else if (total_subs > 1) {
        current_fps = (max_fps_ + min_fps_) / 2;
      } else {
        current_fps = max_fps_;
      }

      rclcpp::WallRate rate(rclcpp::Duration(std::chrono::milliseconds(1000 / current_fps)));

      // Publish one frame
      frame_count_++;
      publish_hq_frame();

      // Publish thumbnail every 5th frame
      if (frame_count_ % 5 == 0) {
        publish_thumbnail();
      }

      // Status every second
      if (frame_count_ % current_fps == 0) {
        auto status = std::make_shared<std_msgs::msg::String>();
        status->data() = "fps=" + std::to_string(current_fps)
                       + " hq_subs=" + std::to_string(hq_subs)
                       + " thumb_subs=" + std::to_string(thumb_subs)
                       + " frames=" + std::to_string(frame_count_);
        status_pub_->publish(status);
        RCLCPP_INFO(get_logger(), "%s", status->data().c_str());
      }

      rclcpp::spin_some(this->shared_from_this());
      rate.sleep();
    }
  }

private:
  void publish_hq_frame()
  {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    auto now = get_clock()->now();
    msg->header().stamp().sec() = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    msg->header().stamp().nanosec() = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    msg->header().frame_id() = "camera_link";
    msg->width() = static_cast<uint32_t>(width_);
    msg->height() = static_cast<uint32_t>(height_);
    msg->encoding() = "bgr8";
    msg->is_bigendian() = false;
    msg->step() = static_cast<uint32_t>(width_ * 3);

    // Simulate image content — gradient pattern
    size_t data_size = static_cast<size_t>(width_ * height_ * 3);
    msg->data().resize(data_size);
    for (size_t i = 0; i < data_size; i += 3) {
      msg->data()[i]     = static_cast<uint8_t>((i / 3) % 256);                   // B
      msg->data()[i + 1] = static_cast<uint8_t>((frame_count_ + i / 3) % 256);    // G
      msg->data()[i + 2] = static_cast<uint8_t>((frame_count_ * 3 + i / 3) % 256); // R
    }

    hq_pub_->publish(msg);
  }

  void publish_thumbnail()
  {
    int tw = width_ / 8;
    int th = height_ / 8;

    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    auto now = get_clock()->now();
    msg->header().stamp().sec() = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    msg->header().stamp().nanosec() = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    msg->header().frame_id() = "camera_link";
    msg->width() = static_cast<uint32_t>(tw);
    msg->height() = static_cast<uint32_t>(th);
    msg->encoding() = "bgr8";
    msg->is_bigendian() = false;
    msg->step() = static_cast<uint32_t>(tw * 3);
    msg->data().resize(static_cast<size_t>(tw * th * 3),
                       static_cast<uint8_t>(frame_count_ % 256));

    thumb_pub_->publish(msg);
  }

  int width_, height_, max_fps_, min_fps_;
  uint64_t frame_count_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hq_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thumb_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

// ==========================================================================
// Node: WaitSetReceiver — Polls for images using WaitSet (non-callback).
//       Demonstrates synchronous message retrieval and MessageInfo.
// ==========================================================================
class WaitSetReceiver : public rclcpp::Node
{
public:
  WaitSetReceiver(std::shared_ptr<rclcpp::DomainParticipant> participant)
    : Node(participant, "waitset_receiver", "imaging"),
      hq_count_(0), thumb_count_(0), hq_bytes_(0), thumb_bytes_(0)
  {
    // Create subscriptions (no callback — will use WaitSet + take)
    rclcpp::QoS hq_qos(rclcpp::KeepAll{});
    hq_qos.reliable();
    hq_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_hq", hq_qos,
      [](sensor_msgs::msg::Image::SharedPtr) {}); // Dummy callback (required by API)

    rclcpp::SensorDataQoS thumb_qos;
    thumb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "image_thumbnail", thumb_qos,
      [](sensor_msgs::msg::Image::SharedPtr) {}); // Dummy callback

    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "WaitSetReceiver ready — polling with WaitSet");
  }

  void poll_loop()
  {
    // Build WaitSet with both subscriptions
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(hq_sub_);
    wait_set.add_subscription(thumb_sub_);

    while (rclcpp::ok()) {
      auto result = wait_set.wait(std::chrono::seconds(1));

      if (result.kind() == rclcpp::WaitResultKind::Ready) {
        // Try to take from HQ subscription
        {
          sensor_msgs::msg::Image msg;
          rclcpp::MessageInfo info;
          while (hq_sub_->take(msg, info)) {
            hq_count_++;
            hq_bytes_ += msg.data().size();
          }
        }

        // Try to take from thumbnail subscription
        {
          sensor_msgs::msg::Image msg;
          rclcpp::MessageInfo info;
          while (thumb_sub_->take(msg, info)) {
            thumb_count_++;
            thumb_bytes_ += msg.data().size();
          }
        }
      }

      // Print stats periodically
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - start_time_).count();
      if (static_cast<int>(elapsed) % 3 == 0 && elapsed - last_print_ > 2.5) {
        last_print_ = elapsed;
        RCLCPP_INFO(get_logger(),
                    "WaitSet stats: HQ=%lu (%.1f MB), Thumb=%lu (%.1f KB), elapsed=%.0fs",
                    hq_count_, hq_bytes_ / 1048576.0,
                    thumb_count_, thumb_bytes_ / 1024.0, elapsed);
      }
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr hq_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thumb_sub_;

  uint64_t hq_count_, thumb_count_;
  uint64_t hq_bytes_, thumb_bytes_;
  std::chrono::steady_clock::time_point start_time_;
  double last_print_ = 0.0;
};

// ==========================================================================
// Main
// ==========================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto publisher = std::make_shared<AdaptivePublisher>();
  auto receiver = std::make_shared<WaitSetReceiver>(publisher->get_participant());

  // Run receiver's WaitSet loop in a separate thread
  std::thread receiver_thread([&receiver]() {
    receiver->poll_loop();
  });

  // Publisher uses spin_some loop in main thread
  publisher->run();

  receiver_thread.join();
  rclcpp::shutdown();
  return 0;
}
