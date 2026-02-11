#include "ROSTypeImagePubSubMono.hpp"
#include "ROSTypeImagePubSubEdge.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting ROSTypeImagePubSubExecutor" << std::endl;

  // Initialize Executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeImagePubSubMono> node1 = std::make_shared<ROSTypeImagePubSubMono>("ROSTypeImagePubSubMono");
  executor.add_node(node1);
  std::shared_ptr<ROSTypeImagePubSubEdge> node2 = std::make_shared<ROSTypeImagePubSubEdge>(node1->get_participant(), "ROSTypeImagePubSubEdge");
  executor.add_node(node2);

  // Wait for the ROS-like node to finish
  executor.spin();
  executor.cancel();
  rclcpp::shutdown();

  return 0;
}
