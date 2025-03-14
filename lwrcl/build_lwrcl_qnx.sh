#!/bin/bash

# Input sudo password here to avoid prompt after build is completed.
sudo echo "OK"

OPT=$1
OPT_NUM=$#

if [ -z "$QNX_TARGET" ]; then
    echo "Please source QNX path."
    exit 1
fi

# clean
if [ ! $OPT_NUM -ne 1 ]; then
  if [ "clean" = $OPT ]; then
    sudo rm -rf ./lwrcl/build_qnx
    mkdir -p ./lwrcl/build_qnx
    exit
  fi
fi

cd lwrcl
mkdir build_qnx
cd build_qnx

DDS_PATH=/opt/qnx/fast-dds/aarch64le/usr
INSTALL_PATH=/opt/qnx/fast-dds-libs
sudo mkdir -p $INSTALL_PATH

sudo rm $INSTALL_PATH/include/lwrcl.hpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_PATH/lib

cmake ..  -DCMAKE_BUILD_TYPE=Debug \
   -DCMAKE_TOOLCHAIN_FILE=../cmake/qnx_aarch64le.cmake \
  -DOPENSSL_ROOT_DIR=$QNX_TARGET/aarch64le/usr \
  -DOPENSSL_INCLUDE_DIR=$QNX_TARGET/usr/include \
  -Dfastcdr_DIR=$DDS_PATH/lib/cmake/fastcdr/ \
  -Dfastrtps_DIR=$DDS_PATH/share/fastrtps/cmake/ \
  -Dfoonathan_memory_DIR=$DDS_PATH/lib/foonathan_memory/cmake/ \
  -Dtinyxml2_DIR=$DDS_PATH/lib/cmake/tinyxml2/ \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH \
  -DCMAKE_SYSTEM_PREFIX_PATH=$QNX_TARGET/aarch64le/usr/ \
  -DCMAKE_PREFIX_PATH=$QNX_TARGET/aarch64le/usr/

make -j4

if [ ! $OPT_NUM -ne 1 ]; then
	if [ "install" = $OPT ]; then
    sudo make install
    sudo cp ../src/lwrcl/fastdds.xml $DDS_PATH/
	fi

fi
