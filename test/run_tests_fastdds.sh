#!/bin/bash
# ===========================================================================
# run_tests_fastdds.sh – Build & run lwrcl gtest suite against Fast DDS
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DDS_PREFIX="${DDS_PREFIX:-/opt/fast-dds}"
LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/fast-dds-libs}"
BUILD_DIR="${SCRIPT_DIR}/build-fastdds"
SOURCE_LWRCL_LIB_DIR="${ROOT_DIR}/lwrcl/build-fastdds/fastdds/lwrcl"
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif [ "$(uname -s)" = "Darwin" ]; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi
ACTION="${1:-all}"

if [ -d "${SOURCE_LWRCL_LIB_DIR}" ]; then
    export LD_LIBRARY_PATH="${SOURCE_LWRCL_LIB_DIR}:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${SOURCE_LWRCL_LIB_DIR}:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
else
    export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
fi

if [ -f "${ROOT_DIR}/lwrcl/fastdds/lwrcl/fastdds.xml" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${ROOT_DIR}/lwrcl/fastdds/lwrcl/fastdds.xml}"
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${ROOT_DIR}/lwrcl/fastdds/lwrcl/fastdds.xml}"
elif [ -f "${LWRCL_PREFIX}/etc/fastdds.xml" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${LWRCL_PREFIX}/etc/fastdds.xml}"
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${LWRCL_PREFIX}/etc/fastdds.xml}"
elif [ -f "${DDS_PREFIX}/fastdds.xml" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${DDS_PREFIX}/fastdds.xml}"
    export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${DDS_PREFIX}/fastdds.xml}"
fi

# ---------------------------------------------------------------------------
show_usage() {
    echo "Usage: $0 [build|run|clean|all]"
    echo "  build  – configure & compile tests"
    echo "  run    – run ctest (build first if needed)"
    echo "  clean  – remove build directory"
    echo "  all    – build + run (default)"
}

do_build() {
    echo "=== [FastDDS] Configuring tests ==="
    cmake \
        -S "${SCRIPT_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DDDS_BACKEND=fastdds \
        -DCMAKE_SYSTEM_PREFIX_PATH="${LWRCL_PREFIX}" \
        -DCMAKE_PREFIX_PATH="${LWRCL_PREFIX}" \
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/" \
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/" \
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/" \
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/" \
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"

    echo "=== [FastDDS] Building tests ==="
    cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

do_run() {
    if [ ! -d "${BUILD_DIR}" ]; then
        echo "Build directory not found – building first..."
        do_build
    fi

    echo "=== [FastDDS] Running tests ==="
    cd "${BUILD_DIR}"
    ctest --output-on-failure -j "${JOBS}"
    RESULT=$?
    echo ""
    if [ $RESULT -eq 0 ]; then
        echo "=== ALL FastDDS TESTS PASSED ==="
    else
        echo "=== SOME FastDDS TESTS FAILED ==="
    fi
    return $RESULT
}

do_clean() {
    rm -rf "${BUILD_DIR}"
    echo "Cleaned ${BUILD_DIR}"
}

# ---------------------------------------------------------------------------
case "${ACTION}" in
    build) do_build ;;
    run)   do_run ;;
    clean) do_clean ;;
    all)   do_build && do_run ;;
    *)     show_usage; exit 1 ;;
esac
