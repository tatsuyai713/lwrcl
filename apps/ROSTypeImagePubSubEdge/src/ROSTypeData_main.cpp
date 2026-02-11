#include "ROSTypeImagePubSubEdge.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting ROSTypeImagePubSubEdge" << std::endl;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeImagePubSubEdge> node = std::make_shared<ROSTypeImagePubSubEdge>("ROSTypeImagePubSubEdge");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
