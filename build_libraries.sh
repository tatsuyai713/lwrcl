#!/bin/bash

OPT=$1
OPT_NUM=$#

# clean
if [ ! $OPT_NUM -ne 1 ]; then
  if [ "clean" = $OPT ]; then
    sudo rm -rf ./libraries/build
    mkdir -p ./libraries/build
    exit
  fi
fi

cd libraries
mkdir build
cd build

DDS_PATH=/opt/fast-dds
INSTALL_PATH=/opt/fast-dds-libs
sudo mkdir -p $INSTALL_PATH

cmake ..  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_SYSTEM_PREFIX_PATH=$DDS_PATH \
  -DCMAKE_PREFIX_PATH=$DDS_PATH \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH
make -j4

if [ ! $OPT_NUM -ne 1 ]; then
	if [ "install" = $OPT ]; then
                sudo make install
	fi

fi
