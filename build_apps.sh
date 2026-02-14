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
    DDS_PREFIX="/opt/cyclonedds"
    VSOMEIP_PREFIX="/opt/vsomeip"
    LWRCL_PREFIX="/opt/vsomeip-libs"
else
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/apps/build-${BACKEND}"
INSTALL_DIR="${SCRIPT_DIR}/apps/install-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

mkdir -p "$INSTALL_DIR"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

# Add iceoryx libraries if present (used by CycloneDDS SHM/zero-copy)
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/apps"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
)

if [ "$BACKEND" = "fastdds" ]; then
    CMAKE_ARGS+=(
        -DCMAKE_SYSTEM_PREFIX_PATH="$LWRCL_PREFIX"
        -DCMAKE_PREFIX_PATH="$LWRCL_PREFIX"
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"
    )
elif [ "$BACKEND" = "cyclonedds" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
    )
elif [ "$BACKEND" = "vsomeip" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    export LD_LIBRARY_PATH="${VSOMEIP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    CMAKE_ARGS+=(
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$INSTALL_DIR"
fi
