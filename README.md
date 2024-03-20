# Fast DDS / ROS 2 Compatible Workspace Guide

This repository provides build scripts and samples for Fast DDS, designed to be compatible with ROS 2 topics. It serves as a bridge for developers looking to integrate Fast DDS with ROS 2 ecosystems, ensuring seamless communication and interoperability between systems using these technologies.

## Features

- **Fast DDS Build Scripts:** Simplify the process of installing and setting up Fast DDS on Ubuntu/Debian systems.
- **ROS 2 Compatible Topics:** Includes samples that demonstrate how to publish and subscribe to ROS 2 topics using Fast DDS, facilitating integration into existing ROS 2 projects.
- **ROS Compatible Libraries:** Offers support for building and installing libraries crucial for ROS compatibility, such as yaml-cpp, ROS data types, and tf2.

## How to Use This Repository

### Preparation: Remove ROS 2 Environment Setup

Remove the ROS 2 environment setup line from your ~/.bashrc:

```
source /opt/ros/humble/setup.bash
```

### Clone the Repository

Clone this repository and enter the directory:

```
git clone --recursive https://github.com/tatsuyai713/FastDDS-ROS2-Compatible-Workspace.git
cd FastDDS-ROS2-Compatible-Workspace
```


### Install Fast DDS

Install Fast DDS and necessary DDS packages on Ubuntu/Debian:

```
cd scripts
./install_fast_dds_ubuntu_debian.sh
# Follow the script instructions...
source ~/.bashrc
```

### Build and Install ROS Compatible Libraries

Build and install libraries for ROS compatibility (yaml-cpp, ROS data types, tf2):

```
cd ../fastdds_ws
./build_libraries.sh install
```

### Build Sample Applications

Compile the sample applications:

```
./build_apps.sh
```

### Build and Install RCL Like Wrapper

Build and install the RCL (ROS Client Library) Like Wrapper for enhanced ROS 2 compatibility:

```
cd ../rcl_like_wrapper
./build_libraries.sh install
```

### Build RCL Like Wrapper Sample Applications

Compile the RCL Like Wrapper sample applications:

```
./build_apps.sh
```

The compiled applications can be found in the apps/build folder.

## Included Open Source Projects

This workspace includes or utilizes the following open-source projects:

- ROS Data Types: https://github.com/rticommunity/ros-data-types
- yaml-cpp: https://github.com/jbeder/yaml-cpp
- Fast-DDS: https://github.com/eProsima/Fast-DDS

This guide provides a comprehensive overview of setting up and using the Fast DDS / ROS 2 Compatible Workspace, ensuring that users can seamlessly integrate Fast DDS into their ROS 2 projects for efficient and interoperable communication.
