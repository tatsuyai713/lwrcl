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

ICEORYX_PREFIX="/opt/qnx/iceoryx"
CMAKE_DDS_BACKEND="$BACKEND"

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/qnx/fast-dds/aarch64le/usr"
    LWRCL_PREFIX="/opt/qnx/fast-dds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_aarch64le.cmake"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    LWRCL_PREFIX="/opt/qnx/cyclonedds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    # libraries/CMakeLists.txt has no adaptive-autosar backend.
    # Build yaml-cpp with CycloneDDS settings and install it into autosar-ap-libs.
    DDS_PREFIX="/opt/qnx/cyclonedds"
    LWRCL_PREFIX="/opt/qnx/autosar-ap-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
    CMAKE_DDS_BACKEND="cyclonedds"
else
    echo "Usage: $0 <fastdds|cyclonedds|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/libraries/build_qnx-${BACKEND}"

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

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/libraries"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Release
    -DDDS_BACKEND="$CMAKE_DDS_BACKEND"
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
    -DYAML_BUILD_SHARED_LIBS=ON
    -DYAML_CPP_INSTALL=ON
    -DYAML_CPP_BUILD_TOOLS=OFF
    -DYAML_CPP_BUILD_TESTS=OFF
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
elif [ "$BACKEND" = "cyclonedds" ] || [ "$BACKEND" = "adaptive-autosar" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$LWRCL_PREFIX"
    if [ -f "${LWRCL_PREFIX}/include/yaml-cpp/yaml.h" ] && [ -f "${LWRCL_PREFIX}/lib/libyaml-cpp.so" ]; then
        echo "yaml-cpp installed to ${LWRCL_PREFIX}"
    else
        echo "Warning: yaml-cpp installation not detected under ${LWRCL_PREFIX}"
    fi
fi
