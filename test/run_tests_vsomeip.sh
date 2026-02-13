#!/bin/bash
# ===========================================================================
# run_tests_vsomeip.sh – Build & run lwrcl gtest suite against vsomeip
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VSOMEIP_PREFIX="/opt/vsomeip"
DDS_PREFIX="/opt/cyclonedds"
LWRCL_PREFIX="/opt/vsomeip-libs"
BUILD_DIR="${SCRIPT_DIR}/build-vsomeip"
JOBS=$(nproc 2>/dev/null || echo 4)
ACTION="${1:-all}"

export LD_LIBRARY_PATH="${VSOMEIP_PREFIX}/lib:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PATH="${VSOMEIP_PREFIX}/bin:${DDS_PREFIX}/bin:${PATH}"
export VSOMEIP_CONFIGURATION="${SCRIPT_DIR}/vsomeip-test.json"

# ---------------------------------------------------------------------------
show_usage() {
    echo "Usage: $0 [build|run|clean|all]"
    echo "  build  – configure & compile tests"
    echo "  run    – run ctest (build first if needed)"
    echo "  clean  – remove build directory"
    echo "  all    – build + run (default)"
}

do_build() {
    echo "=== [vsomeip] Configuring tests ==="
    cmake \
        -S "${SCRIPT_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DDDS_BACKEND=vsomeip \
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${VSOMEIP_PREFIX}/lib/cmake" \
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}" \
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"

    echo "=== [vsomeip] Building tests ==="
    cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

do_run() {
    if [ ! -d "${BUILD_DIR}" ]; then
        echo "Build directory not found – building first..."
        do_build
    fi

    echo "=== [vsomeip] Running tests ==="
    cd "${BUILD_DIR}"
    ctest --output-on-failure -j "${JOBS}"
    RESULT=$?
    echo ""
    if [ $RESULT -eq 0 ]; then
        echo "=== ALL vsomeip TESTS PASSED ==="
    else
        echo "=== SOME vsomeip TESTS FAILED ==="
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
