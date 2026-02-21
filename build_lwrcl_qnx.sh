#!/bin/bash
set -euo pipefail

# Input sudo password here to avoid prompt after build is completed.
sudo echo "OK"

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

if [ -z "${QNX_TARGET:-}" ] || [ -z "${QNX_HOST:-}" ]; then
    echo "Please source QNX SDP environment (QNX_HOST/QNX_TARGET)."
    exit 1
fi

QNX_ARCH="${AUTOSAR_QNX_ARCH:-aarch64le}"
ICEORYX_PREFIX="/opt/qnx/iceoryx"

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/qnx/fast-dds/aarch64le/usr"
    LWRCL_PREFIX="/opt/qnx/fast-dds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_aarch64le.cmake"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    LWRCL_PREFIX="/opt/qnx/cyclonedds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    AUTOSAR_AP_PREFIX="/opt/qnx/autosar_ap/${QNX_ARCH}"
    LWRCL_PREFIX="/opt/qnx/autosar-ap-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
else
    echo "Usage: $0 <fastdds|cyclonedds|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/lwrcl/build_qnx-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

sudo mkdir -p "$LWRCL_PREFIX"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ "$BACKEND" = "adaptive-autosar" ] && [ -d "${AUTOSAR_AP_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${AUTOSAR_AP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/lwrcl"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
)

if [ "$BACKEND" = "fastdds" ]; then
    CMAKE_ARGS+=(
        -DOPENSSL_ROOT_DIR="${QNX_TARGET}/aarch64le/usr"
        -DOPENSSL_INCLUDE_DIR="${QNX_TARGET}/usr/include"
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
        -DCMAKE_SYSTEM_PREFIX_PATH="${QNX_TARGET}/aarch64le/usr/"
        -DCMAKE_PREFIX_PATH="${QNX_TARGET}/aarch64le/usr/"
    )
elif [ "$BACKEND" = "cyclonedds" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DICEORYX_PREFIX="${ICEORYX_PREFIX}"
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
    )
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DAUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX}"
        -DDDS_PREFIX="${DDS_PREFIX}"
        -DICEORYX_PREFIX="${ICEORYX_PREFIX}"
        -DCMAKE_PREFIX_PATH="${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP;${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
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
