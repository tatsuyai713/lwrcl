#!/bin/bash
# ===========================================================================
# run_tests_cyclonedds.sh – Build & run lwrcl gtest suite against Cyclone DDS
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DDS_PREFIX="/opt/cyclonedds"
LWRCL_PREFIX="/opt/cyclonedds-libs"
BUILD_DIR="${SCRIPT_DIR}/build-cyclonedds"
JOBS=$(nproc 2>/dev/null || echo 4)
ACTION="${1:-all}"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PATH="${DDS_PREFIX}/bin:${PATH}"

# ---------------------------------------------------------------------------
show_usage() {
    echo "Usage: $0 [build|run|clean|all]"
    echo "  build  – configure & compile tests"
    echo "  run    – run ctest (build first if needed)"
    echo "  clean  – remove build directory"
    echo "  all    – build + run (default)"
}

do_build() {
    echo "=== [CycloneDDS] Configuring tests ==="
    cmake \
        -S "${SCRIPT_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DDDS_BACKEND=cyclonedds \
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake" \
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"

    echo "=== [CycloneDDS] Building tests ==="
    cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

do_run() {
    if [ ! -d "${BUILD_DIR}" ]; then
        echo "Build directory not found – building first..."
        do_build
    fi

    echo "=== [CycloneDDS] Running tests ==="
    cd "${BUILD_DIR}"
    ctest --output-on-failure -j "${JOBS}"
    RESULT=$?
    echo ""
    if [ $RESULT -eq 0 ]; then
        echo "=== ALL CycloneDDS TESTS PASSED ==="
    else
        echo "=== SOME CycloneDDS TESTS FAILED ==="
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
