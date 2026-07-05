#!/bin/bash
# ===========================================================================
# run_tests_cyclonedds.sh – Build & run lwrcl gtest suite against Cyclone DDS
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/cyclonedds-libs}"
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
BUILD_DIR="${SCRIPT_DIR}/build-cyclonedds"
ROUDI_PID=""
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif [ "$(uname -s)" = "Darwin" ]; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi
ACTION="${1:-all}"

if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
else
    export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
fi

if [ -d "${ICEORYX_PREFIX}/bin" ]; then
    export PATH="${ICEORYX_PREFIX}/bin:${DDS_PREFIX}/bin:${PATH}"
else
    export PATH="${DDS_PREFIX}/bin:${PATH}"
fi

cleanup_roudi() {
    if [ -n "${ROUDI_PID}" ] && kill -0 "${ROUDI_PID}" 2>/dev/null; then
        kill -TERM "${ROUDI_PID}" 2>/dev/null || true
        wait "${ROUDI_PID}" 2>/dev/null || true
        ROUDI_PID=""
    fi
}

start_roudi() {
    if [ ! -x "${ICEORYX_PREFIX}/bin/iox-roudi" ]; then
        echo "[ERROR] Missing iox-roudi: ${ICEORYX_PREFIX}/bin/iox-roudi" >&2
        echo "        Reinstall CycloneDDS with iceoryx support or run scripts/install_iceoryx.sh." >&2
        return 1
    fi

    if pgrep -x iox-roudi >/dev/null 2>&1; then
        return 0
    fi

    "${ICEORYX_PREFIX}/bin/iox-roudi" > /tmp/lwrcl_test_iox_roudi.log 2>&1 &
    ROUDI_PID=$!

    for _ in {1..100}; do
        if ! kill -0 "${ROUDI_PID}" 2>/dev/null; then
            echo "[ERROR] iox-roudi failed to start. See /tmp/lwrcl_test_iox_roudi.log" >&2
            return 1
        fi
        if grep -Eiq "RouDi is ready|RouDi is up and running|iox-roudi" /tmp/lwrcl_test_iox_roudi.log 2>/dev/null; then
            return 0
        fi
        sleep 0.1
    done

    return 0
}

trap cleanup_roudi EXIT INT TERM

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
        -DICEORYX_PREFIX="${ICEORYX_PREFIX}" \
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

    RESULT=0
    if [ -x "${ICEORYX_PREFIX}/bin/iox-roudi" ]; then
        start_roudi || RESULT=$?
    fi
    env -u CYCLONEDDS_URI ctest --output-on-failure -j 1 -E '^test_shm_zero_copy$' || RESULT=$?

    if ctest -N -R '^test_shm_zero_copy$' | grep -q 'test_shm_zero_copy'; then
        DEFAULT_CYCLONEDDS_URI=""
        if [ -n "${CYCLONEDDS_URI:-}" ]; then
            DEFAULT_CYCLONEDDS_URI="${CYCLONEDDS_URI}"
        elif [ -f "${LWRCL_PREFIX}/etc/cyclonedds-lwrcl.xml" ]; then
            DEFAULT_CYCLONEDDS_URI="file://${LWRCL_PREFIX}/etc/cyclonedds-lwrcl.xml"
        elif [ -f "${DDS_PREFIX}/etc/cyclonedds-lwrcl.xml" ]; then
            DEFAULT_CYCLONEDDS_URI="file://${DDS_PREFIX}/etc/cyclonedds-lwrcl.xml"
        elif [ -f "${ROOT_DIR}/lwrcl/cyclonedds/lwrcl/cyclonedds-lwrcl.xml" ]; then
            DEFAULT_CYCLONEDDS_URI="file://${ROOT_DIR}/lwrcl/cyclonedds/lwrcl/cyclonedds-lwrcl.xml"
        fi

        if [ -z "${DEFAULT_CYCLONEDDS_URI}" ]; then
            echo "[ERROR] Missing CycloneDDS iceoryx config." >&2
            echo "        Reinstall lwrcl or CycloneDDS with SHM support." >&2
            RESULT=1
        else
            start_roudi || RESULT=$?
            CYCLONEDDS_URI="${DEFAULT_CYCLONEDDS_URI}" \
            ctest --output-on-failure -R '^test_shm_zero_copy$' || RESULT=$?
        fi
    fi

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
