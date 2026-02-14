#!/usr/bin/env bash
set -euo pipefail

CYCLONEDDS_VER="master"  
INSTALL_X64_PREFIX="/opt/cyclonedds" 
BUILD_DIR="$HOME/build-cyclonedds"

# iceoryx prefix for shared-memory (zero-copy) transport
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"

# Detect whether iceoryx is installed
ENABLE_SHM=OFF
if [ -d "${ICEORYX_PREFIX}/lib/cmake/iceoryx_hoofs" ]; then
    ENABLE_SHM=ON
    echo "=== iceoryx detected at ${ICEORYX_PREFIX} — enabling SHM (zero-copy) ==="
else
    echo "=== iceoryx not found at ${ICEORYX_PREFIX} — SHM disabled ==="
    echo "    Run scripts/install_iceoryx.sh first for zero-copy support."
fi

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

SHM_CMAKE_ARGS=()
if [ "${ENABLE_SHM}" = "ON" ]; then
    SHM_CMAKE_ARGS+=(
        -DENABLE_SHM=ON
        -DCMAKE_PREFIX_PATH="${ICEORYX_PREFIX}"
    )
else
    SHM_CMAKE_ARGS+=( -DENABLE_SHM=OFF )
fi

cmake -S . -B build-host \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_X64_PREFIX} \
    -DBUILD_SHARED_LIBS=ON \
    "${SHM_CMAKE_ARGS[@]}"
cmake --build build-host -j$(nproc)
sudo cmake --install build-host

git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
cd cyclonedds-cxx
git checkout 0.10.5
rm -rf build-host-cxx

CXX_PREFIX_PATH="${INSTALL_X64_PREFIX}"
if [ "${ENABLE_SHM}" = "ON" ]; then
    CXX_PREFIX_PATH="${INSTALL_X64_PREFIX};${ICEORYX_PREFIX}"
fi

cmake -B build-host-cxx \
  -DCMAKE_PREFIX_PATH="${CXX_PREFIX_PATH}" \
  -DCMAKE_INSTALL_PREFIX=${INSTALL_X64_PREFIX} \
  -DBUILD_IDLCPP_GENERATOR=ON \
  -DCYCLONEDDS_CXX_BLD_IDLCPP=ON \
  -DENABLE_TOPIC_DISCOVERY=OFF \
  -DDDSCXX_NO_STD_OPTIONAL=ON \
  -DBUILD_SHARED_LIBS=ON
cmake --build build-host-cxx -j$(nproc)
sudo cmake --install build-host-cxx

