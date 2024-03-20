#!/bin/bash
sudo pip3 install -U vcstool pyyaml jsonschema
sudo pip install -U vcstool pyyaml jsonschema
sudo apt install -y libasio-dev libtinyxml2-dev libssl-dev libyaml-cpp-dev

INSTALL_PATH=/opt/fast-dds
sudo mv /opt/ros/ /opt/ros_tmp/

cd ~
sudo rm -rf DDS-Router-2.0.0*
wget https://github.com/eProsima/DDS-Router/archive/refs/tags/v2.0.0.tar.gz -O DDS-Router-2.0.0.tar.gz

tar xvf ./DDS-Router-2.0.0.tar.gz

mkdir -p ~/DDS-Router-2.0.0/src
mkdir -p ~/DDS-Router-2.0.0/build
cd ~/DDS-Router-2.0.0
wget https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos
vcs import src < ddsrouter.repos

# CMake Utils
cd ~/DDS-Router-2.0.0
mkdir build/cmake_utils
cd build/cmake_utils
cmake ~/DDS-Router-2.0.0/src/dev-utils/cmake_utils \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
sudo cmake --build . --target install

# C++ Utils
cd ~/DDS-Router-2.0.0
mkdir build/cpp_utils
cd build/cpp_utils
cmake ~/DDS-Router-2.0.0/src/dev-utils/cpp_utils  \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
sudo cmake --build . --target install

# ddsrouter_core
cd ~/DDS-Router-2.0.0
mkdir build/ddsrouter_core
cd build/ddsrouter_core
cmake ~/DDS-Router-2.0.0/src/ddsrouter/ddsrouter_core  \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
sudo cmake --build . --target install

# ddsrouter_yaml
cd ~/DDS-Router-2.0.0
mkdir build/ddsrouter_yaml
cd build/ddsrouter_yaml
cmake ~/DDS-Router-2.0.0/src/ddsrouter/ddsrouter_yaml  \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
sudo cmake --build . --target install

# ddsrouter_tool
cd ~/DDS-Router-2.0.0
mkdir build/ddsrouter_tool
cd build/ddsrouter_tool
cmake ~/DDS-Router-2.0.0/src/ddsrouter/tools/ddsrouter_tool  \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
sudo cmake --build . --target install

cd ~/
sudo rm -rf DDS-Router-2.0.0*

sudo mv /opt/ros_tmp/ /opt/ros/

