#!/usr/bin/env bash
set -euo pipefail

CYCLONEDDS_VER="master"  
INSTALL_X64_PREFIX="/opt/cyclonedds" 
BUILD_DIR="$HOME/build-cyclonedds"

sudo mkdir -p ${INSTALL_X64_PREFIX}
sudo chmod 777 -R ${INSTALL_X64_PREFIX}


rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

if [ ! -d cyclonedds ]; then
  git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
fi
cd cyclonedds
git checkout 0.10.5

export OPENSSL_ROOT_DIR=/usr
export OPENSSL_INCLUDE_DIR=/usr/include
cmake -S . -B build-host -DCMAKE_INSTALL_PREFIX=${INSTALL_X64_PREFIX} -DBUILD_SHARED_LIBS=ON -DENABLE_SHM=OFF 
cmake --build build-host -j$(nproc)
sudo cmake --install build-host

git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
cd cyclonedds-cxx
git checkout 0.10.5
rm -rf build-host-cxx
cmake -B build-host-cxx \
  -DCMAKE_PREFIX_PATH=${INSTALL_X64_PREFIX} \
  -DCMAKE_INSTALL_PREFIX=${INSTALL_X64_PREFIX} \
  -DBUILD_IDLCPP_GENERATOR=ON \
  -DCYCLONEDDS_CXX_BLD_IDLCPP=ON \
  -DENABLE_TOPIC_DISCOVERY=OFF \
  -DDDSCXX_NO_STD_OPTIONAL=ON \
  -DBUILD_SHARED_LIBS=ON
cmake --build build-host-cxx -j$(nproc)
sudo cmake --install build-host-cxx

