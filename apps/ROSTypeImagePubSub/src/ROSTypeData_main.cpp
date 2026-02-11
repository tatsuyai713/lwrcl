#include "ROSTypeImagePubSub.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting ROSTypeImagePubSub" << std::endl;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeImagePubSub> node = std::make_shared<ROSTypeImagePubSub>("ROSTypeImagePubSub");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
