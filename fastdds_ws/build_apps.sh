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
CURRENT=`pwd`
mkdir build
cd build
DDS_PATH=/opt/fast-dds
LIB_PATH=/opt/fast-dds-libs

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_PATH/lib:$LIB_PATH/lib

mkdir -p ${CURRENT}/install

cmake ..  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_SYSTEM_PREFIX_PATH=$LIB_PATH \
  -DCMAKE_PREFIX_PATH=$LIB_PATH \
  -Dfastcdr_DIR=$DDS_PATH/lib/cmake/fastcdr/ \
  -Dfastrtps_DIR=$DDS_PATH/share/fastrtps/cmake/ \
  -Dfoonathan_memory_DIR=$DDS_PATH/lib/foonathan_memory/cmake/ \
  -Dtinyxml2_DIR=$DDS_PATH/lib/cmake/tinyxml2/ \
  -Dyaml-cpp_DIR=$LIB_PATH/lib/cmake/yaml-cpp/ \
  -DCMAKE_INSTALL_PREFIX=${CURRENT}/install
make -j4


if [ ! $OPT_NUM -ne 1 ]; then
  if [ "install" = $OPT ]; then
    sudo make install
  fi
fi
