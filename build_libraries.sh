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
elif [ "$BACKEND" = "vsomeip" ]; then
    VSOMEIP_PREFIX="/opt/vsomeip"
    LWRCL_PREFIX="/opt/vsomeip-libs"
else
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/libraries/build-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

sudo mkdir -p "$LWRCL_PREFIX"

if [ -n "${DDS_PREFIX:-}" ]; then
    export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/libraries"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Release
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
    -DYAML_BUILD_SHARED_LIBS=ON
    -DYAML_CPP_INSTALL=ON
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
elif [ "$BACKEND" = "vsomeip" ]; then
    CMAKE_ARGS+=(
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$LWRCL_PREFIX"
fi
