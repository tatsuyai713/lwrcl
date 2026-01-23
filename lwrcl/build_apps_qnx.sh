#!/bin/bash

OPT=$1
OPT_NUM=$#

if [ -z "$QNX_TARGET" ]; then
    echo "Please source QNX path."
    exit 1
fi

# clean
if [ ! $OPT_NUM -ne 1 ]; then
  if [ "clean" = $OPT ]; then
    sudo rm -rf ./apps/build_qnx
    mkdir -p ./apps/build_qnx
    exit
  fi
fi

cd apps
mkdir -p build_qnx
cd build_qnx

DDS_PATH=/opt/qnx/fast-dds
LIB_PATH=/opt/qnx/fast-dds-libs
OPT_PATH=/opt/

mkdir -p ${CURRENT}/install

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DDS_PATH/aarch64le/usr/lib:$LIB_PATH/aarch64le/usr/lib

cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/qnx_aarch64le.cmake \
    -DCMAKE_SYSTEM_PREFIX_PATH=$QNX_TARGET/aarch64le/usr \
    -DCMAKE_PREFIX_PATH=$QNX_TARGET/aarch64le/usr \
    -Dfastcdr_DIR=$DDS_PATH/aarch64le/usr/lib/cmake/fastcdr/ \
    -Dfastrtps_DIR=$DDS_PATH/aarch64le/usr/share/fastrtps/cmake/ \
    -Dfoonathan_memory_DIR=$DDS_PATH/aarch64le/usr/lib/foonathan_memory/cmake/ \
    -Dtinyxml2_DIR=$DDS_PATH/aarch64le/usr/lib/cmake/tinyxml2/ \
    -DOpenCV_DIR=$OPT_PATH/local/share/OpenCV/ \
    -DCMAKE_INSTALL_PREFIX=${CURRENT}/install 
make -j4


if [ ! $OPT_NUM -ne 1 ]; then
        if [ "install" = $OPT ]; then
                sudo cmake --install . --prefix ${CURRENT}/install
        fi

fi
