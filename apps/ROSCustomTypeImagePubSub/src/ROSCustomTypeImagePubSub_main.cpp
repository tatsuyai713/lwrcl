#include "ROSCustomTypeImagePubSub.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting ROSCustomTypeImagePubSub" << std::endl;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSCustomTypeImagePubSub> node = std::make_shared<ROSCustomTypeImagePubSub>("ROSCustomTypeImagePubSub");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
