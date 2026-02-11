#!/bin/bash
source $HOME/qnx800/qnxsdp-env.sh
export LD_LIBRARY_PATH=/opt/qnx/cyclonedds/lib:/opt/qnx/cyclonedds-libs/lib:$LD_LIBRARY_PATH
rm -rf ./build-qnx
mkdir -p build-qnx

cmake  . -B build-qnx -DCMAKE_TOOLCHAIN_FILE=./cmake/qnx_toolchain.cmake 
cmake --build build-qnx