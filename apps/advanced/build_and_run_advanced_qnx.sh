#!/bin/bash
# Build advanced examples for QNX targets
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROOT_DIR="$(cd "${APPS_DIR}/.." && pwd)"

BACKEND="${1:-}"
ACTION="${2:-all}"
TARGET="${3:-}"

JOBS=$(nproc 2>/dev/null || echo 4)

if [ -z "${QNX_TARGET:-}" ] || [ -z "${QNX_HOST:-}" ]; then
    echo "Please source QNX SDP environment (QNX_HOST/QNX_TARGET)."
    exit 1
fi

QNX_ARCH="${AUTOSAR_QNX_ARCH:-aarch64le}"
ICEORYX_PREFIX="/opt/qnx/iceoryx"
IDLC_PREFIX="${IDLC_PREFIX:-/opt/cyclonedds}"
HOST_ICEORYX_PREFIX="${HOST_ICEORYX_PREFIX:-/opt/iceoryx}"

if [ -z "$BACKEND" ]; then
    echo "Usage: $0 <fastdds|cyclonedds|adaptive-autosar> [build|run|list|clean] [target]"
    exit 1
fi

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/qnx/fast-dds"
    LWRCL_PREFIX="/opt/qnx/fast-dds-libs"
    TOOLCHAIN_FILE="${ROOT_DIR}/scripts/cmake/qnx_aarch64le.cmake"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    LWRCL_PREFIX="/opt/qnx/cyclonedds-libs"
    TOOLCHAIN_FILE="${ROOT_DIR}/scripts/cmake/qnx_toolchain.cmake"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    AUTOSAR_AP_PREFIX="/opt/qnx/autosar-ap/${QNX_ARCH}"
    VSOMEIP_PREFIX="/opt/qnx/vsomeip"
    LWRCL_PREFIX="/opt/qnx/autosar-ap-libs"
    TOOLCHAIN_FILE="${ROOT_DIR}/scripts/cmake/qnx_toolchain.cmake"
else
    echo "Usage: $0 <fastdds|cyclonedds|adaptive-autosar> [build|run|list|clean] [target]"
    exit 1
fi

BUILD_DIR="${APPS_DIR}/build_qnx-advanced-${BACKEND}"
INSTALL_DIR="${APPS_DIR}/install-qnx-${BACKEND}"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ -d "${IDLC_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${IDLC_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ -d "${HOST_ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${HOST_ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ "$BACKEND" = "adaptive-autosar" ] && [ -d "${AUTOSAR_AP_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${AUTOSAR_AP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ "$BACKEND" = "adaptive-autosar" ] && [ -d "${AUTOSAR_AP_PREFIX}/bin" ]; then
    export PATH="${AUTOSAR_AP_PREFIX}/bin:${PATH}"
fi

# Actions
if [ "$ACTION" = "clean" ]; then
    rm -rf "${BUILD_DIR}"
    echo "Cleaned ${BUILD_DIR}"
    exit 0
fi

if [ "$ACTION" = "list" ]; then
    echo "Available advanced examples (QNX build will create these):"
    echo "  video_stream_pipeline"
    echo "  diagnostic_aggregator"
    echo "  adaptive_image_pipeline"
    echo "  state_machine_controller"
    exit 0
fi

if [ "$ACTION" = "build" ] || [ "$ACTION" = "all" ]; then
    mkdir -p "${BUILD_DIR}"
    CMAKE_ARGS=(
        -S "${APPS_DIR}"
        -B "${BUILD_DIR}"
        -DCMAKE_BUILD_TYPE=Debug
        -DDDS_BACKEND="${BACKEND}"
        -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"
        -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
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
        CMAKE_ARGS+=(
            -DICEORYX_PREFIX="${ICEORYX_PREFIX}"
            -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
        )
    elif [ "$BACKEND" = "adaptive-autosar" ]; then
        CMAKE_ARGS+=(
            -DAUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX}"
            -DDDS_PREFIX="${DDS_PREFIX}"
            -DICEORYX_PREFIX="${ICEORYX_PREFIX}"
            -DCMAKE_PREFIX_PATH="${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP;${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
        )
    fi

    cmake "${CMAKE_ARGS[@]}"

    # Build targets
    cmake --build "${BUILD_DIR}" --target video_stream_pipeline -j "${JOBS}"
    cmake --build "${BUILD_DIR}" --target diagnostic_aggregator -j "${JOBS}"
    cmake --build "${BUILD_DIR}" --target adaptive_image_pipeline -j "${JOBS}"
    cmake --build "${BUILD_DIR}" --target state_machine_controller -j "${JOBS}"

    echo "=== QNX Advanced build complete ==="
    exit 0
fi

# Run: note that QNX-built binaries are for target; running them on host is not supported.
if [ "$ACTION" = "run" ]; then
    if [ -z "${TARGET}" ]; then
        echo "Specify target to run (cannot run QNX binaries on host)."
        exit 1
    fi
    exe="${BUILD_DIR}/advanced/${TARGET}"
    if [ ! -f "${exe}" ]; then
        echo "Executable not found: ${exe}"
        exit 1
    fi
    echo "Note: This script cross-builds for QNX. Running the binary on host is not supported."
    echo "Copy ${exe} to QNX target (or run in QNX environment) to execute."
    exit 0
fi

show_usage() {
    echo "Usage: $0 <fastdds|cyclonedds|adaptive-autosar> [build|run|list|clean] [target]"
}

show_usage
