#!/bin/bash

OPT=$1
OPT_NUM=$#

# clean
if [ ! $OPT_NUM -ne 1 ]; then
  if [ "clean" = $OPT ]; then
    sudo rm -rf ./build
    mkdir ./build
    exit
  fi
fi

mkdir build
cd build

INSTALL_PATH=/opt/fast-dds
sudo mkdir -p /opt/fast-dds-libs

cmake ..  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_INSTALL_PREFIX=/opt/fast-dds-libs
make -j4


if [ ! $OPT_NUM -ne 1 ]; then
	if [ "install" = $OPT ]; then
                sudo make install
                grep 'export LD_LIBRARY_PATH=/opt/fast-dds-libs/lib:$LD_LIBRARY_PATH' ~/.bashrc
                if [ $? = 0 ]; then
                        echo "LD_LIBRARY_PATH libs are already added"
                else
                        echo 'export LD_LIBRARY_PATH=/opt/fast-dds-libs/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
                        source ~/.bashrc
                fi
                sudo ldconfig
	fi

fi
