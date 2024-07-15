#include "ROSTypeImagePubSubMono.hpp"
#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <string>

int main(int argc, char **argv)
{
  lwrcl::init(argc, argv);
  std::cout << "Starting ROSTypeImagePubSubMono" << std::endl;

  // Buffer to hold the path to the current executable
  char buf[PATH_MAX] = {0};
  std::string exePath, configPath;

  // Attempt to read the symbolic link '/proc/self/exe' to find the path of the current executable
  ssize_t count = readlink("/proc/self/exe", buf, sizeof(buf));
  if (count != -1)
  {
    exePath = std::string(buf, count); // Convert to string and assign
    size_t pos = exePath.rfind('/');   // Find the last occurrence of '/'
    if (pos != std::string::npos)
    {
      configPath = exePath.substr(0, pos + 1); // Extract the path without executable name
    }
  }
  else
  {
    std::cerr << "Failed to determine the path of the executable." << std::endl;
    return 1;
  }

  // Initialize and run the ROS-like node
  std::shared_ptr<ROSTypeImagePubSubMono> rcl_like_node = std::make_shared<ROSTypeImagePubSubMono>("ROSTypeImagePubSubMono");
  configPath += "config/config.yaml"; // Append the relative path of the config file
  std::cout << "Using config file at: " << configPath << std::endl;

  if (rcl_like_node->init_config(configPath))
  {
    lwrcl::spin(rcl_like_node);
    lwrcl::shutdown();
  }
  else
  {
    std::cerr << "Failed to initialize the ROSTypeImagePubSubMono." << std::endl;
    return 1;
  }

  return 0;
}
