/**
 * @file test_shm_zero_copy.cpp
 * @brief Test shared-memory fast path with normal ROS-compatible publish API.
 */
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "fixedsized_msgs/msg/fixed_image_vga_mono8.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ShmZeroCopyTest : public ::testing::Test {
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

namespace {

std::string read_file(const std::string& path) {
  std::ifstream file(path.c_str());
  if (!file) {
    return "";
  }
  std::ostringstream out;
  out << file.rdbuf();
  return out.str();
}

bool file_readable(const std::string& path) {
  std::ifstream file(path.c_str());
  return static_cast<bool>(file);
}

std::string env_value(const char* name) {
  const char* value = std::getenv(name);
  return value == nullptr ? std::string() : std::string(value);
}

std::string strip_file_uri(std::string uri) {
  const std::string prefix = "file://";
  if (uri.compare(0, prefix.size(), prefix) == 0) {
    return uri.substr(prefix.size());
  }
  return uri;
}

std::string first_readable_path(const std::vector<std::string>& candidates) {
  for (const auto& candidate : candidates) {
    if (!candidate.empty() && file_readable(candidate)) {
      return candidate;
    }
  }
  return "";
}

}  // namespace

TEST_F(ShmZeroCopyTest, BackendProfileEnablesSharedMemoryFastPath) {
#if defined(LWRCL_BACKEND_FASTDDS)
  const std::string profile = first_readable_path({
      env_value("FASTDDS_DEFAULT_PROFILES_FILE"),
      env_value("FASTRTPS_DEFAULT_PROFILES_FILE"),
      "/opt/fast-dds-libs/etc/fastdds.xml",
      "/opt/fast-dds/fastdds.xml",
      std::string(LWRCL_TEST_SOURCE_DIR) + "/../lwrcl/fastdds/lwrcl/fastdds.xml",
  });
  ASSERT_FALSE(profile.empty()) << "FastDDS profile was not found";

  const std::string xml = read_file(profile);
  ASSERT_FALSE(xml.empty()) << profile;
  EXPECT_NE(xml.find("<type>SHM</type>"), std::string::npos) << profile;
  EXPECT_NE(xml.find("<transport_id>shm_transport</transport_id>"), std::string::npos) << profile;
  EXPECT_NE(xml.find("<transport_id>udpv4_transport</transport_id>"), std::string::npos) << profile;
  EXPECT_EQ(xml.find("<interfaceWhiteList>"), std::string::npos)
      << "localhost-only UDP prevents remote-host fallback: " << profile;
  EXPECT_EQ(xml.find("<address>127.0.0.1</address>"), std::string::npos)
      << "localhost-only UDP prevents remote-host fallback: " << profile;
#elif defined(LWRCL_BACKEND_CYCLONEDDS)
  const std::string profile = first_readable_path({
      strip_file_uri(env_value("CYCLONEDDS_URI")),
      "/opt/cyclonedds-libs/etc/cyclonedds-lwrcl.xml",
      "/opt/cyclonedds/etc/cyclonedds-lwrcl.xml",
      std::string(LWRCL_TEST_SOURCE_DIR) + "/../lwrcl/cyclonedds/lwrcl/cyclonedds-lwrcl.xml",
  });
  ASSERT_FALSE(profile.empty()) << "CycloneDDS SHM profile was not found";

  const std::string xml = read_file(profile);
  ASSERT_FALSE(xml.empty()) << profile;
  EXPECT_NE(xml.find("<SharedMemory>"), std::string::npos) << profile;
  EXPECT_NE(xml.find("<Enable>true</Enable>"), std::string::npos) << profile;
#else
  GTEST_SKIP() << "SHM zero-copy profile test is only defined for FastDDS and CycloneDDS";
#endif
}

TEST_F(ShmZeroCopyTest, NormalPublishReceivesFixedSizeImage) {
  using Image = fixedsized_msgs::msg::FixedImageVgaMono8;

  auto node = rclcpp::Node::make_shared("shm_zero_copy_test");
  rclcpp::SensorDataQoS qos;

  std::atomic<int> received{0};
  uint32_t rx_width = 0;
  uint8_t rx_first_byte = 0;

  auto sub = node->create_subscription<Image>(
      "shm_zero_copy_image", qos,
      [&](Image::SharedPtr msg) {
        rx_width = msg->width;
        rx_first_byte = msg->data[0];
        received++;
      });

  auto pub = node->create_publisher<Image>("shm_zero_copy_image", qos);

  auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
  while ((pub->get_subscription_count() == 0 || sub->get_publisher_count() == 0) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_GT(pub->get_subscription_count(), 0);
  ASSERT_GT(sub->get_publisher_count(), 0);
  Image msg;
  msg.header.stamp.sec = 123;
  msg.height = 480;
  msg.width = 640;
  msg.encoding.size = 5;
  msg.encoding.data[0] = 'm';
  msg.encoding.data[1] = 'o';
  msg.encoding.data[2] = 'n';
  msg.encoding.data[3] = 'o';
  msg.encoding.data[4] = '8';
  msg.is_bigendian = false;
  msg.step = 640;
  msg.data.fill(0);
  msg.data[0] = 0x42;

  pub->publish(msg);

  auto receive_deadline = std::chrono::steady_clock::now() + 5s;
  while (received.load() == 0 && std::chrono::steady_clock::now() < receive_deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GE(received.load(), 1);
  EXPECT_EQ(rx_width, 640u);
  EXPECT_EQ(rx_first_byte, 0x42u);
}
