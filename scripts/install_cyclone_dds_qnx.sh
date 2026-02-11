#!/usr/bin/env bash
set -euo pipefail

INSTALL_PREFIX="/opt/qnx/cyclonedds" 
BUILD_DIR="$HOME/build-cyclonedds-qnx"

sudo mkdir -p ${INSTALL_PREFIX}
sudo chmod 777 -R ${INSTALL_PREFIX}


rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

cp ./cmake/qnx_toolchain.cmake "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -d cyclonedds ]; then
  git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
fi
cd cyclonedds
git checkout 0.10.5

source "$HOME/qnx800/qnxsdp-env.sh"
export OPENSSL_ROOT_DIR=$QNX_TARGET/aarch64le/usr
export OPENSSL_INCLUDE_DIR=$QNX_TARGET/aarch64le/usr/include
sed -i.bak -E 's/^[[:space:]]*(find_package\(Threads REQUIRED\)|target_link_libraries\(ddsrt INTERFACE Threads::Threads\))/# &/' src/ddsrt/CMakeLists.txt
cmake -S . -B build-qnx -DCMAKE_TOOLCHAIN_FILE=../qnx_toolchain.cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} -DBUILD_SHARED_LIBS=ON -DENABLE_SHM=OFF -DENABLE_SOURCE_SPECIFIC_MULTICAST=OFF 
cmake --build build-qnx -j$(nproc)
cmake --install build-qnx

git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
cd cyclonedds-cxx
git checkout 0.10.5
rm -rf build-qnx-cxx
cmake -B build-qnx-cxx \
  -DCMAKE_TOOLCHAIN_FILE=../../qnx_toolchain.cmake \
  -DCMAKE_PREFIX_PATH=${INSTALL_PREFIX} \
  -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} \
  -DBUILD_IDLCPP_GENERATOR=ON \
  -DCYCLONEDDS_CXX_BLD_IDLCPP=ON \
  -DENABLE_TOPIC_DISCOVERY=OFF \
  -DDDSCXX_NO_STD_OPTIONAL=ON \
  -DBUILD_SHARED_LIBS=ON
cmake --build build-qnx-cxx -j$(nproc)
sudo cmake --install build-qnx-cxx

