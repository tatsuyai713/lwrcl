#!/bin/bash
set -euo pipefail

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/fast-dds"
    LWRCL_PREFIX="/opt/fast-dds-libs"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/cyclonedds-libs"
else
    echo "Usage: $0 <fastdds|cyclonedds> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/data_types/build-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

sudo mkdir -p "$LWRCL_PREFIX"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/data_types"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
)

if [ "$BACKEND" = "fastdds" ]; then
    CMAKE_ARGS+=(
        -DCMAKE_SYSTEM_PREFIX_PATH="$DDS_PREFIX"
        -DCMAKE_PREFIX_PATH="$DDS_PREFIX"
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
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
    sudo cmake --install "$BUILD_DIR" --prefix "$LWRCL_PREFIX"
fi
