#!/bin/bash
set -euo pipefail

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

# Check if BUILD_FFI is set (default OFF)
BUILD_FFI_OPTION="${BUILD_FFI:-OFF}"

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/fast-dds"
    LWRCL_PREFIX="/opt/fast-dds-libs"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/cyclonedds-libs"
elif [ "$BACKEND" = "vsomeip" ]; then
    VSOMEIP_PREFIX="/opt/vsomeip"
    LWRCL_PREFIX="/opt/vsomeip-libs"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    AUTOSAR_AP_PREFIX="/opt/autosar_ap"
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/autosar-ap-libs"
else
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/lwrcl/build-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

sudo mkdir -p "$LWRCL_PREFIX"

if [ -n "${DDS_PREFIX:-}" ]; then
    export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

# Add iceoryx libraries if present (used by CycloneDDS SHM/zero-copy)
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/lwrcl"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
    -DBUILD_FFI="$BUILD_FFI_OPTION"
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
    export LD_LIBRARY_PATH="${VSOMEIP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    CMAKE_ARGS+=(
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}"
    )
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    export LD_LIBRARY_PATH="${AUTOSAR_AP_PREFIX}/lib:${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DAUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX}"
        -DDDS_PREFIX="${DDS_PREFIX}"
        -DCMAKE_PREFIX_PATH="${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP;${DDS_PREFIX}/lib/cmake"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$LWRCL_PREFIX"
    if [ "$BACKEND" = "fastdds" ]; then
        sudo cp "${SCRIPT_DIR}/lwrcl/fastdds/lwrcl/fastdds.xml" "$DDS_PREFIX/" 2>/dev/null || true
    fi
fi
