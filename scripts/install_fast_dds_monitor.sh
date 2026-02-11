#!/bin/bash
sudo apt install -y libqt5charts5-dev qtquickcontrols2-5-dev
sudo apt install -y qtdeclarative5-dev qml-module-qtcharts qml-module-qt-labs-calendar

CURRENT=`pwd`
INSTALL_PATH=/opt/fast-dds
sudo mv /opt/ros/ /opt/ros_tmp/

rm -rf fastdds_statistics
rm -rf fastdds_monitor

mkdir fastdds_statistics
cd fastdds_statistics
wget https://github.com/eProsima/Fast-DDS-statistics-backend/archive/refs/tags/v0.11.0.tar.gz
tar xvf v0.11.0.tar.gz
cd Fast-DDS-statistics-backend-0.11.0
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
make
sudo make install

cd $CURRENT

mkdir fastdds_monitor
cd fastdds_monitor
wget https://github.com/eProsima/Fast-DDS-monitor/archive/refs/tags/v1.5.0.tar.gz

tar xvf v1.5.0.tar.gz

cd Fast-DDS-monitor-1.5.0

mkdir build
cd build
cmake .. \
  -Dfastrtps_DIR=$INSTALL_PATH/share/fastrtps/ \
  -Dfastcdr_DIR=$INSTALL_PATH/lib/cmake/fastcdr/ \
  -Dfoonathan_memory_DIR=$INSTALL_PATH/lib/foonathan_memory/cmake/ \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH
make
sudo make install

cd $CURRENT

rm -rf fastdds_statistics
rm -rf fastdds_monitor

sudo mv /opt/ros_tmp/ /opt/ros/

