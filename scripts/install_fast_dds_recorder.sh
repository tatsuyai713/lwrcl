#!/bin/bash

CURRENT=`pwd`
INSTALL_PATH=/opt/fast-dds
sudo apt install -y qtbase5-dev liblz4-dev libzstd-dev libyaml-cpp-dev

sudo mv /opt/ros/ /opt/ros_tmp/

rm -rf ./fastdds_dev_utils
rm -rf ./fastdds_pipe
rm -rf ./fastdds_recorder

mkdir fastdds_dev_utils
cd fastdds_dev_utils
wget https://github.com/eProsima/dev-utils/archive/refs/tags/v0.4.0.tar.gz
tar xvf v0.4.0.tar.gz
cd dev-utils-0.4.0
cd cmake_utils
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../

cd cpp_utils
mkdir build
cd build
cmake ..
make
sudo make install

cd $CURRENT

mkdir fastdds_pipe
cd fastdds_pipe
wget https://github.com/eProsima/DDS-Pipe/archive/refs/tags/v0.2.0.tar.gz

tar xvf v0.2.0.tar.gz

cd DDS-Pipe-0.2.0

cd ddspipe_core
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH

make
sudo make install
cd ../../

cd ddspipe_participants
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH

make
sudo make install
cd ../../

cd ddspipe_yaml
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH

make
sudo make install

cd $CURRENT

mkdir fastdds_recorder
cd fastdds_recorder
wget https://github.com/eProsima/DDS-Record-Replay/archive/refs/tags/v0.2.0.tar.gz
tar xvf v0.2.0.tar.gz
cd DDS-Record-Replay-0.2.0

cd ddsrecorder_participants
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH

make
sudo make install
cd ../../

cd controller/controller_tool
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH

make
cd ../../../

cd ddsrecorder_yaml
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH

make
sudo make install
cd ../../

cd ddsrecorder
mkdir build
cd build
cmake ..  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH \
  -Dfastcdr_DIR=$INSTALL_PATH/lib/cmake/fastcdr/ \
  -Dfoonathan_memory_DIR=$INSTALL_PATH/lib/foonathan_memory/cmake/ \
  -Dfastrtps_DIR=$INSTALL_PATH/share/fastrtps/cmake/
make
sudo make install

cd $CURRENT

rm -rf ./fastdds_dev_utils
rm -rf ./fastdds_pipe
rm -rf ./fastdds_recorder

sudo mv /opt/ros_tmp/ /opt/ros/

