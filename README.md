# LWRCL (LightWeight Rclcpp Compatible Library)  with QNX 8.0 support

This repository provides build scripts and samples for Fast DDS, designed to be compatible with ROS 2 topics. It serves as a bridge for developers looking to integrate Fast DDS with ROS 2 ecosystems, ensuring seamless communication and interoperability between systems using these technologies.

This library provides a simplified API similar to ROS 2's rclcpp for working with Fast DDS, enabling easier integration and management of nodes, publishers, subscribers, and timers within the Fast DDS ecosystem.

And also, this library enables to implement ROS 2 compatible applications with Fast DDS on lightweight SBCs such as Raspberry Pi.

## Features

- **Fast DDS Build Scripts:** Simplify the process of installing and setting up Fast DDS on Ubuntu/Debian and QNX systems.
- **ROS 2 Compatible Topics:** Includes samples that demonstrate how to publish and subscribe to ROS 2 topics using Fast DDS, facilitating integration into existing ROS 2 projects.
- **ROS Compatible Libraries:** Offers support for building and installing libraries crucial for ROS compatibility, such as yaml-cpp, ROS data types, and tf2.
- **lwrcl:** Please read `lwrcl/README.md`.

## How to Use This Repository

### Preparation: Remove ROS 2 Environment Setup

Remove the ROS 2 environment setup line from your ~/.bashrc:

```
source /opt/ros/humble/setup.bash
```

### Clone the Repository

Clone this repository and enter the directory:

```bash
git clone --recursive https://github.com/tatsuyai713/lwrcl.git
cd lwrcl
```


### Install Fast DDS

Install Fast DDS and necessary DDS packages on Ubuntu/Debian:

```bash
cd scripts
./install_fast_dds_ubuntu_debian.sh
# Follow the script instructions...
source ~/.bashrc
```

### Build and Install ROS Data Types

```bash
cd ../lwrcl
./build_data_types.sh install
```

### Build and Install LWRCL

Build and install the lwrcl for enhanced ROS 2 compatibility:

```bash
./build_lwrcl.sh install
```

### Build and Install Libraries for ROS Compatibility

```bash
./build_libraries.sh install
```

### Build LWRCL Sample Applications

Compile the lwrcl sample applications:

```bash
./build_apps.sh install
```

The compiled applications can be found in the apps/install folder.

## Included Open Source Projects

This workspace includes or utilizes the following open-source projects:

- ROS Data Types: https://github.com/rticommunity/ros-data-types
- yaml-cpp: https://github.com/jbeder/yaml-cpp
- Fast-DDS: https://github.com/eProsima/Fast-DDS

This guide provides a comprehensive overview of setting up and using the Fast DDS / ROS 2 Compatible Workspace, ensuring that users can seamlessly integrate Fast DDS into their ROS 2 projects for efficient and interoperable communication.

