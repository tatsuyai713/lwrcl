#!/bin/bash

OPT=$1
OPT_NUM=$#

# clean
if [ ! $OPT_NUM -ne 1 ]; then
  if [ "clean" = $OPT ]; then
    sudo rm -rf ./apps/build
    mkdir -p ./apps/build
    exit
  fi
fi

cd apps
mkdir build
cd build

INSTALL_PATH=/opt/fast-dds
sudo mkdir -p /opt/fast-dds-libs

cmake ..  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_INSTALL_PREFIX=/opt/fast-dds-libs
make -j4


if [ ! $OPT_NUM -ne 1 ]; then
	if [ "install" = $OPT ]; then
                sudo make install
	fi

fi
