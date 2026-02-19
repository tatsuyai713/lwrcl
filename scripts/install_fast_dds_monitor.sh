#!/usr/bin/env bash
set -euo pipefail

STATS_BACKEND_TAG="0.11.0"
MONITOR_TAG="1.5.0"
INSTALL_PREFIX="/opt/fast-dds"
BUILD_DIR="${HOME}/build-fast-dds-monitor"
_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"
HIDE_ROS="ON"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --prefix <path>            Fast-DDS install prefix (default: ${INSTALL_PREFIX})
  --stats-tag <version>      Statistics backend version (default: ${STATS_BACKEND_TAG})
  --monitor-tag <version>    Monitor version (default: ${MONITOR_TAG})
  --build-dir <path>         Build directory (default: \${HOME}/build-fast-dds-monitor)
  --jobs <N>                 Parallel build jobs (default: auto)
  --no-hide-ros              Do not temporarily move /opt/ros
  --skip-system-deps         Skip installing system dependencies
  --force                    Force reinstall even if already present
  --help                     Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --stats-tag)
      STATS_BACKEND_TAG="$2"
      shift 2
      ;;
    --monitor-tag)
      MONITOR_TAG="$2"
      shift 2
      ;;
    --build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    --jobs)
      JOBS="$2"
      shift 2
      ;;
    --no-hide-ros)
      HIDE_ROS="OFF"
      shift
      ;;
    --skip-system-deps)
      SKIP_SYSTEM_DEPS="ON"
      shift
      ;;
    --force)
      FORCE_REINSTALL="ON"
      shift
      ;;
    --help)
      usage
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
  else
    echo "[ERROR] Please run as root or install sudo." >&2
    exit 1
  fi
fi

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]] && command -v apt-get >/dev/null 2>&1; then
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y libqt5charts5-dev qtquickcontrols2-5-dev \
    qtdeclarative5-dev qml-module-qtcharts qml-module-qt-labs-calendar
fi

# Temporarily hide /opt/ros to avoid CMake conflicts
ROS_HIDDEN="OFF"
if [[ "${HIDE_ROS}" == "ON" ]] && [[ -d /opt/ros ]]; then
  ${SUDO} mv /opt/ros/ /opt/ros_tmp/
  ROS_HIDDEN="ON"
fi

cleanup() {
  if [[ "${ROS_HIDDEN}" == "ON" ]] && [[ -d /opt/ros_tmp ]]; then
    ${SUDO} mv /opt/ros_tmp/ /opt/ros/
  fi
}
trap cleanup EXIT

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# Statistics Backend
cd "${BUILD_DIR}"
wget "https://github.com/eProsima/Fast-DDS-statistics-backend/archive/refs/tags/v${STATS_BACKEND_TAG}.tar.gz"
tar xf "v${STATS_BACKEND_TAG}.tar.gz"
cmake -S "Fast-DDS-statistics-backend-${STATS_BACKEND_TAG}" -B "stats-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_SYSTEM_PREFIX_PATH="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}"
cmake --build "stats-build" -j"${JOBS}"
${SUDO} cmake --install "stats-build"

# Monitor
cd "${BUILD_DIR}"
wget "https://github.com/eProsima/Fast-DDS-monitor/archive/refs/tags/v${MONITOR_TAG}.tar.gz"
tar xf "v${MONITOR_TAG}.tar.gz"
cmake -S "Fast-DDS-monitor-${MONITOR_TAG}" -B "monitor-build" \
  -Dfastrtps_DIR="${INSTALL_PREFIX}/share/fastrtps/" \
  -Dfastcdr_DIR="${INSTALL_PREFIX}/lib/cmake/fastcdr/" \
  -Dfoonathan_memory_DIR="${INSTALL_PREFIX}/lib/foonathan_memory/cmake/" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_SYSTEM_PREFIX_PATH="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}"
cmake --build "monitor-build" -j"${JOBS}"
${SUDO} cmake --install "monitor-build"

echo "[OK] Installed Fast-DDS Monitor v${MONITOR_TAG} + Statistics Backend v${STATS_BACKEND_TAG} to ${INSTALL_PREFIX}"
