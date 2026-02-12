#!/bin/bash
# ===========================================================================
# build_and_run_advanced.sh – Build & run advanced lwrcl examples
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROOT_DIR="$(cd "${APPS_DIR}/.." && pwd)"

BACKEND="${1:-}"
ACTION="${2:-all}"
TARGET="${3:-}"

JOBS=$(nproc 2>/dev/null || echo 4)

# ---------------------------------------------------------------------------
show_usage() {
    cat <<EOF
Usage: $0 <fastdds|cyclonedds> [action] [target]

Actions:
  build              – Configure & build advanced examples
  run <target>       – Run a specific example
  list               – List available executables
  clean              – Remove build directory
  all                – Build only (default)

Targets (for 'run'):
  video_stream_pipeline
  diagnostic_aggregator
  adaptive_image_pipeline
  state_machine_controller

Examples:
  $0 fastdds build
  $0 fastdds run video_stream_pipeline
  $0 cyclonedds run diagnostic_aggregator -- --params-file config/diagnostic_aggregator.yaml
  $0 fastdds clean
EOF
}

# ---------------------------------------------------------------------------
if [ -z "$BACKEND" ]; then
    show_usage
    exit 1
fi

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/fast-dds"
    LWRCL_PREFIX="/opt/fast-dds-libs"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/cyclonedds-libs"
else
    echo "ERROR: Unknown backend '$BACKEND'. Use 'fastdds' or 'cyclonedds'."
    show_usage
    exit 1
fi

BUILD_DIR="${APPS_DIR}/build-advanced-${BACKEND}"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PATH="${DDS_PREFIX}/bin:${PATH}"

# ---------------------------------------------------------------------------
do_build() {
    echo "=== Building advanced examples (${BACKEND}) ==="

    CMAKE_ARGS=(
        -S "${APPS_DIR}"
        -B "${BUILD_DIR}"
        -DCMAKE_BUILD_TYPE=Debug
        -DDDS_BACKEND="${BACKEND}"
    )

    if [ "$BACKEND" = "fastdds" ]; then
        CMAKE_ARGS+=(
            -DCMAKE_SYSTEM_PREFIX_PATH="${DDS_PREFIX}"
            -DCMAKE_PREFIX_PATH="${DDS_PREFIX}"
            -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
            -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
            -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
            -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
            -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"
        )
    elif [ "$BACKEND" = "cyclonedds" ]; then
        CMAKE_ARGS+=(
            -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
            -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"
        )
    fi

    cmake "${CMAKE_ARGS[@]}"

    # Build only the advanced targets
    cmake --build "${BUILD_DIR}" --target video_stream_pipeline -j "${JOBS}"
    cmake --build "${BUILD_DIR}" --target diagnostic_aggregator -j "${JOBS}"
    cmake --build "${BUILD_DIR}" --target adaptive_image_pipeline -j "${JOBS}"
    cmake --build "${BUILD_DIR}" --target state_machine_controller -j "${JOBS}"

    echo "=== Build complete ==="
}

do_run() {
    local target="$1"
    shift

    if [ -z "$target" ]; then
        echo "ERROR: Specify a target to run. Use '$0 $BACKEND list' to see available targets."
        exit 1
    fi

    local exe="${BUILD_DIR}/advanced/${target}"
    if [ ! -f "$exe" ]; then
        echo "Executable not found at ${exe}"
        echo "Trying to build first..."
        do_build
    fi

    if [ ! -f "$exe" ]; then
        echo "ERROR: Build failed or target '${target}' does not exist."
        echo "Available targets:"
        do_list
        exit 1
    fi

    # Copy config files next to the executable
    cp -r "${SCRIPT_DIR}/config" "${BUILD_DIR}/advanced/" 2>/dev/null || true

    echo "=== Running: ${target} (${BACKEND}) ==="
    cd "${BUILD_DIR}/advanced"
    "./${target}" "$@"
}

do_list() {
    echo "Available advanced examples:"
    echo "  video_stream_pipeline     – H265 video stream simulation pipeline"
    echo "  diagnostic_aggregator     – Multi-node diagnostic monitoring system"
    echo "  adaptive_image_pipeline   – Adaptive QoS image pipeline with WaitSet"
    echo "  state_machine_controller  – Timer-driven state machine with service control"
}

do_clean() {
    rm -rf "${BUILD_DIR}"
    echo "Cleaned ${BUILD_DIR}"
}

# ---------------------------------------------------------------------------
case "${ACTION}" in
    build)  do_build ;;
    run)
        # Pass only extra args (after the 3 positional: backend, run, target)
        shift 3 || true
        do_run "${TARGET}" "$@"
        ;;
    list)   do_list ;;
    clean)  do_clean ;;
    all)    do_build ;;
    *)      show_usage; exit 1 ;;
esac
