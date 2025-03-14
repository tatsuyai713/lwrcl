#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lwrcl_example_waitset");

  auto publisher_ptr = node->create_publisher<sensor_msgs::msg::Image>("TESTTopic1", 10);
  if (!publisher_ptr) {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
    return 1;
  }

  auto subscription_ptr = node->create_subscription<sensor_msgs::msg::Image>(
    "TESTTopic2",
    10,
    [](sensor_msgs::msg::Image::SharedPtr) {
      // no-op
    }
  );
  if (!subscription_ptr) {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return 1;
  }

  rclcpp::WallRate rate(std::chrono::milliseconds(100));

  int data_value = 0;

  struct timespec curTime, lastTime;
  clock_gettime(CLOCK_REALTIME, &lastTime);

  rclcpp::WaitSet wait_set({{subscription_ptr}});

  while (rclcpp::ok()) {
    int32_t publisher_count = subscription_ptr->get_publisher_count();
    std::cout << "Number of publishers: " << publisher_count << std::endl;

    clock_gettime(CLOCK_REALTIME, &curTime);
    if (curTime.tv_nsec < lastTime.tv_nsec) {
      printf("Interval = %10ld.%09ld\n",
        curTime.tv_sec - lastTime.tv_sec - 1,
        curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
    } else {
      printf("Interval = %10ld.%09ld\n",
        curTime.tv_sec - lastTime.tv_sec,
        curTime.tv_nsec - lastTime.tv_nsec);
    }
    lastTime = curTime;

    sensor_msgs::msg::Image pub_message;
    pub_message.header().stamp().sec() = data_value;
    pub_message.header().stamp().nanosec() = data_value;
    pub_message.header().frame_id() = "TEST";
    pub_message.height() = 100;
    pub_message.width() = 200;
    pub_message.encoding() = "H265";
    pub_message.is_bigendian() = false;
    pub_message.step() = 1;
    pub_message.data() = {0, 0, 0, 0, 0, 0};

    publisher_ptr->publish(pub_message);

    auto wait_result = wait_set.wait(std::chrono::seconds(1)); 
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      sensor_msgs::msg::Image received_msg;
      rclcpp::MessageInfo info;
      if (subscription_ptr->take(received_msg, info)) {
        RCLCPP_INFO(node->get_logger(),
          "Received Image data via WaitSet: width=%u height=%u encoding=%s",
          received_msg.width(),
          received_msg.height(),
          received_msg.encoding().c_str());
      }
    } else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) {
      RCLCPP_WARN(node->get_logger(), "WaitSet timed out, no data received in 2 seconds.");
    }

    RCLCPP_WARN(node->get_logger(),
      "Publishing: '%s'", pub_message.encoding().c_str());

    data_value++;
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
