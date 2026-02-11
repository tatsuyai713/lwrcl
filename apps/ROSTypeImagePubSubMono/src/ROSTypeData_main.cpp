#include "ROSTypeImagePubSubMono.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting ROSTypeImagePubSubMono" << std::endl;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeImagePubSubMono> node = std::make_shared<ROSTypeImagePubSubMono>("ROSTypeImagePubSubMono");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
