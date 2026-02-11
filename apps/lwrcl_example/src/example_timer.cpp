#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

void myCallbackFunction(sensor_msgs::msg::Image::SharedPtr message, rclcpp::Node::SharedPtr node)
{
  if (message == nullptr)
  {
    std::cerr << "Error: Received null message in callback." << std::endl;
    return;
  }
  // Print the received image data
  RCLCPP_WARN(node->get_logger(), "Received Image data: Width: %d, Height: %d, Encoding: %s", message->width(), message->height(), message->encoding().c_str());
}

void myTimerFunction(sensor_msgs::msg::Image::SharedPtr my_message, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_ptr)
{
  static int data_value = 0;
  if (publisher_ptr == nullptr)
  {
    std::cerr << "Error: Invalid publisher pointer." << std::endl;
    return;
  }

  // Simulate sending data periodically
  my_message->header().stamp().sec() = data_value;
  my_message->header().stamp().nanosec() = data_value;
  my_message->header().frame_id() = "TEST";
  my_message->height() = 100;
  my_message->width() = 200;
  my_message->encoding() = "H265";
  my_message->is_bigendian() = false;
  my_message->step() = 1;
  my_message->data() = {0, 0, 0, 0, 0, 0};

  std::cout << "Publishing Image data: Sec: " << my_message->header().stamp().sec() << ", Nsec: " << my_message->header().stamp().nanosec() << std::endl;
  std::cout << "Height: " << my_message->height() << ", Width: " << my_message->width() << ", Encoding: " << my_message->encoding() << std::endl;

  // Publish the message
  publisher_ptr->publish(my_message);
  data_value++;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create a node with domain ID 0
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lwrcl_example");

  // Create a publisher with default QoS settings
  auto publisher_ptr = node->create_publisher<sensor_msgs::msg::Image>("TESTTopic2", 10);
  if (publisher_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a publisher." << std::endl;
    return 1;
  }

  // Create a subscription with default QoS settings
  auto subscriber_ptr = node->create_subscription<sensor_msgs::msg::Image>("TESTTopic1", 10, std::bind(myCallbackFunction, std::placeholders::_1, node));
  if (subscriber_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a subscription." << std::endl;
    return 1;
  }

  sensor_msgs::msg::Image::SharedPtr pub_message = std::make_shared<sensor_msgs::msg::Image>();
  auto timer_ptr = node->create_wall_timer(std::chrono::milliseconds(100), [&pub_message, publisher_ptr]()
                                           { myTimerFunction(pub_message, publisher_ptr); });

  if (timer_ptr == nullptr)
  {
    std::cerr << "Error: Failed to create a timer." << std::endl;
    return 1;
  }

  // Spin the node to handle incoming messages
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
