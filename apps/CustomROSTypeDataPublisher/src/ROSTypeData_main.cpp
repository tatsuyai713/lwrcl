#include "ROSTypeDataPublisher.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting ROSTypeDataPublisher" << std::endl;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeDataPublisher> node = std::make_shared<ROSTypeDataPublisher>(0);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
