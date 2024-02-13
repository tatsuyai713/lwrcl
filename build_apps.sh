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
INSTALL_PATH=/opt/fast-dds-libs

mkdir -p ${CURRENT}/install

cmake ..  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_SYSTEM_PREFIX_PATH=$INSTALL_PATH \
  -DCMAKE_PREFIX_PATH=$INSTALL_PATH \
  -Dfastcdr_DIR=$INSTALL_PATH/lib/cmake/fastcdr/ \
  -Dfastrtps_DIR=$INSTALL_PATH/share/fastrtps/cmake/ \
  -Dfoonathan_memory_DIR=$INSTALL_PATH/lib/foonathan_memory/cmake/ \
  -Dyaml-cpp_DIR=$INSTALL_PATH/lib/cmake/yaml-cpp/ \
  -DCMAKE_INSTALL_PREFIX=${CURRENT}/install
make -j4


if [ ! $OPT_NUM -ne 1 ]; then
  if [ "install" = $OPT ]; then
    sudo make install
  fi
fi
