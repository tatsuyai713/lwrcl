#!/bin/bash
set -euo pipefail

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

if [ -z "$QNX_TARGET" ]; then
    echo "Please source QNX path."
    exit 1
fi

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/qnx/fast-dds"
    LWRCL_PREFIX="/opt/qnx/fast-dds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_aarch64le.cmake"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    LWRCL_PREFIX="/opt/qnx/cyclonedds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
else
    echo "Usage: $0 <fastdds|cyclonedds> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/apps/build_qnx"
INSTALL_DIR="${SCRIPT_DIR}/apps/install"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

mkdir -p "$INSTALL_DIR"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/apps"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
)

if [ "$BACKEND" = "fastdds" ]; then
    CMAKE_ARGS+=(
        -DCMAKE_SYSTEM_PREFIX_PATH="${QNX_TARGET}/aarch64le/usr"
        -DCMAKE_PREFIX_PATH="${QNX_TARGET}/aarch64le/usr"
        -Dfastcdr_DIR="${DDS_PREFIX}/aarch64le/usr/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/aarch64le/usr/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/aarch64le/usr/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/aarch64le/usr/lib/cmake/tinyxml2/"
    )
elif [ "$BACKEND" = "cyclonedds" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$INSTALL_DIR"
fi
