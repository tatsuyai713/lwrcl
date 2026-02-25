#!/bin/bash
# Run all 4 latency benchmark patterns and print a summary table.
# Runs each pattern RUNS times and reports the median of each metric.
#
# Usage:
#   ./run_all.sh [RUNS]   (default: 5)
#
# Prerequisites: build binaries first with build_lwrcl.sh and build_ros2.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${SCRIPT_DIR}/bin"
RUNS="${1:-5}"
ROS_ROOT="${ROS_ROOT:-/opt/ros/humble}"

# Colour helpers (skip if not a tty)
if [ -t 1 ]; then
    BOLD="\033[1m"; RESET="\033[0m"
else
    BOLD=""; RESET=""
fi

check_bin() {
    if [ ! -x "$1" ]; then
        echo "ERROR: $1 not found. Run the build scripts first."
        exit 1
    fi
}

check_bin "${BIN}/lat_lwrcl_cyclone"
check_bin "${BIN}/lat_lwrcl_fastdds"
check_bin "${BIN}/lat_ros2"

# median of a list of floats (space-separated)
median() {
    echo "$@" | tr ' ' '\n' | sort -n | awk -v n="$#" 'NR==int((n+1)/2){print}'
}

run_bench() {
    local label="$1"; shift
    local cmd=("$@")
    echo -e "${BOLD}=== ${label} ===${RESET}"
    local p50s=() p90s=() p99s=() mins=()
    for i in $(seq 1 "${RUNS}"); do
        local out
        out=$(timeout 90 "${cmd[@]}" 2>/dev/null)
        local p50 p90 p99 mn
        p50=$(echo "$out" | awk '/^p50:/{printf "%.1f", $2}')
        p90=$(echo "$out" | awk '/^p90:/{printf "%.1f", $2}')
        p99=$(echo "$out" | awk '/^p99:/{printf "%.1f", $2}')
        mn=$(echo "$out"  | awk '/^Min:/{printf "%.1f", $2}')
        echo "  run $i: p50=${p50} p90=${p90} p99=${p99} Min=${mn} us"
        p50s+=("$p50"); p90s+=("$p90"); p99s+=("$p99"); mins+=("$mn")
    done
    MED_P50=$(median "${p50s[@]}")
    MED_P90=$(median "${p90s[@]}")
    MED_P99=$(median "${p99s[@]}")
    MED_MIN=$(median "${mins[@]}")
    echo "  → median:  p50=${MED_P50}  p90=${MED_P90}  p99=${MED_P99}  Min=${MED_MIN} us"
    echo ""
}

# ── lwrcl + CycloneDDS ────────────────────────────────────────────────────────
LWRCL_CDDS_CMD=(
    env
    LD_LIBRARY_PATH="/opt/iceoryx/lib:/opt/cyclonedds/lib:/opt/cyclonedds-libs/lib:${LD_LIBRARY_PATH:-}"
    "${BIN}/lat_lwrcl_cyclone"
)
run_bench "lwrcl + CycloneDDS" "${LWRCL_CDDS_CMD[@]}"
LWRCL_CDDS_P50=$MED_P50; LWRCL_CDDS_P90=$MED_P90; LWRCL_CDDS_P99=$MED_P99; LWRCL_CDDS_MIN=$MED_MIN

# ── lwrcl + FastDDS ───────────────────────────────────────────────────────────
LWRCL_FDDS_CMD=(
    env
    LD_LIBRARY_PATH="/opt/fast-dds/lib:/opt/fast-dds-libs/lib:${LD_LIBRARY_PATH:-}"
    "${BIN}/lat_lwrcl_fastdds"
)
run_bench "lwrcl + FastDDS" "${LWRCL_FDDS_CMD[@]}"
LWRCL_FDDS_P50=$MED_P50; LWRCL_FDDS_P90=$MED_P90; LWRCL_FDDS_P99=$MED_P99; LWRCL_FDDS_MIN=$MED_MIN

# ── ROS 2 + CycloneDDS ─────────────────────────────────────────────────────────
if [ -f "${ROS_ROOT}/setup.bash" ]; then
    # Temporarily disable nounset so ROS 2 setup.bash can use unset variables
    set +u
    source "${ROS_ROOT}/setup.bash"
    set -u
    ROS 2_CDDS_CMD=(
        env
        LD_LIBRARY_PATH="/opt/iceoryx/lib:${ROS_ROOT}/lib:${LD_LIBRARY_PATH:-}"
        RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        "${BIN}/lat_ros2"
    )
    run_bench "ROS 2 + CycloneDDS" "${ROS 2_CDDS_CMD[@]}"
    ROS 2_CDDS_P50=$MED_P50; ROS 2_CDDS_P90=$MED_P90; ROS 2_CDDS_P99=$MED_P99; ROS 2_CDDS_MIN=$MED_MIN

    ROS 2_FDDS_CMD=(
        env
        LD_LIBRARY_PATH="${ROS_ROOT}/lib:${LD_LIBRARY_PATH:-}"
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        "${BIN}/lat_ros2"
    )
    run_bench "ROS 2 + FastDDS" "${ROS 2_FDDS_CMD[@]}"
    ROS 2_FDDS_P50=$MED_P50; ROS 2_FDDS_P90=$MED_P90; ROS 2_FDDS_P99=$MED_P99; ROS 2_FDDS_MIN=$MED_MIN
else
    echo "WARNING: ROS 2 not found at ${ROS_ROOT}. Skipping ROS 2 benchmarks."
    ROS 2_CDDS_P50="-"; ROS 2_CDDS_P90="-"; ROS 2_CDDS_P99="-"; ROS 2_CDDS_MIN="-"
    ROS 2_FDDS_P50="-"; ROS 2_FDDS_P90="-"; ROS 2_FDDS_P99="-"; ROS 2_FDDS_MIN="-"
fi

# ── Summary table ─────────────────────────────────────────────────────────────
echo -e "${BOLD}╔══════════════════════════════════════════════════════════════════════╗${RESET}"
echo -e "${BOLD}║            Latency Summary (one-way = round-trip / 2)  [µs]         ║${RESET}"
echo -e "${BOLD}╠══════════════════════════════════════════════════════════════════════╣${RESET}"
printf "${BOLD}║ %-6s │ %18s │ %15s │ %17s │ %15s ║\n${RESET}" \
    "Metric" "lwrcl+CycloneDDS" "lwrcl+FastDDS" "ROS 2+CycloneDDS" "ROS 2+FastDDS"
echo -e "${BOLD}╠══════════════════════════════════════════════════════════════════════╣${RESET}"
printf "║ %-6s │ %18s │ %15s │ %17s │ %15s ║\n" \
    "p50"  "${LWRCL_CDDS_P50}" "${LWRCL_FDDS_P50}" "${ROS 2_CDDS_P50}" "${ROS 2_FDDS_P50}"
printf "║ %-6s │ %18s │ %15s │ %17s │ %15s ║\n" \
    "p90"  "${LWRCL_CDDS_P90}" "${LWRCL_FDDS_P90}" "${ROS 2_CDDS_P90}" "${ROS 2_FDDS_P90}"
printf "║ %-6s │ %18s │ %15s │ %17s │ %15s ║\n" \
    "p99"  "${LWRCL_CDDS_P99}" "${LWRCL_FDDS_P99}" "${ROS 2_CDDS_P99}" "${ROS 2_FDDS_P99}"
printf "║ %-6s │ %18s │ %15s │ %17s │ %15s ║\n" \
    "Min"  "${LWRCL_CDDS_MIN}" "${LWRCL_FDDS_MIN}" "${ROS 2_CDDS_MIN}" "${ROS 2_FDDS_MIN}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════════════════════╝${RESET}"
echo ""
echo "Conditions: ${RUNS} runs × 1000 samples (200 warm-up), 1 ms timer, std_msgs::msg::String"
echo "Each value = median across ${RUNS} runs."
