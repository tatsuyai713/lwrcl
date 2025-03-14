#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Callback function for handling received messages
void myCallbackFunction(sensor_msgs::msg::Image::SharedPtr message)
{
  if (message == nullptr)
  {
    std::cerr << "Error: Received null message in callback." << std::endl;
    return;
  }
  // Print the received image data
  std::cout << "Received Image data: ";
  std::cout << "Sec: " << message->header().stamp().sec() << ", ";
  std::cout << "Nanosec: " << message->header().stamp().nanosec() << ", ";
  std::cout << "Width: " << message->width() << ", ";
  std::cout << "Height: " << message->height() << ", ";
  std::cout << "Encoding: " << message->encoding() << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create a node with domain ID 0
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lwrcl_example");

  // Create a publisher with default QoS settings
  auto publisher_ptr = node->create_publisher<sensor_msgs::msg::Image>("TESTTopic1", 10);
  if (publisher_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
    return 1;
  }

  // Create a subscription with default QoS settings
  auto subscriber_ptr = node->create_subscription<sensor_msgs::msg::Image>("TESTTopic2", 10, myCallbackFunction);
  if (subscriber_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return 1;
  }

  int data_value = 0;
  rclcpp::WallRate rate(rclcpp::Duration(std::chrono::milliseconds(100))); // Set rate to 100 milliseconds

  struct timespec curTime, lastTime;
  clock_gettime(CLOCK_REALTIME, &lastTime);

  // Main application loop
  while (rclcpp::ok())
  {
    // Check the number of publishers
    int32_t publisher_count = subscriber_ptr->get_publisher_count();
    std::cout << "Number of publishers: " << publisher_count << std::endl;

    clock_gettime(CLOCK_REALTIME, &curTime);
    // Print the interval between the last two messages
    if (curTime.tv_nsec < lastTime.tv_nsec)
    {
      printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec - 1, curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
    }
    else
    {
      printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec, curTime.tv_nsec - lastTime.tv_nsec);
    }
    lastTime = curTime;

    // Simulate sending data periodically
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

    // Publish the data
    publisher_ptr->publish(pub_message);

    // Handle incoming messages
    rclcpp::spin_some(node);

    RCLCPP_WARN(node->get_logger(), "Publishing: '%s'", pub_message.encoding().c_str());

    data_value++;
    rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
