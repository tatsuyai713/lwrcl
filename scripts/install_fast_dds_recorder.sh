#!/usr/bin/env bash
set -euo pipefail

DEV_UTILS_TAG="0.4.0"
DDS_PIPE_TAG="0.2.0"
RECORDER_TAG="0.2.0"
INSTALL_PREFIX="/opt/fast-dds"
BUILD_DIR="${HOME}/build-fast-dds-recorder"
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
  --prefix <path>          Fast-DDS install prefix (default: ${INSTALL_PREFIX})
  --recorder-tag <ver>     DDS-Record-Replay version (default: ${RECORDER_TAG})
  --pipe-tag <ver>         DDS-Pipe version (default: ${DDS_PIPE_TAG})
  --dev-utils-tag <ver>    Dev-utils version (default: ${DEV_UTILS_TAG})
  --build-dir <path>       Build directory (default: \${HOME}/build-fast-dds-recorder)
  --jobs <N>               Parallel build jobs (default: auto)
  --no-hide-ros            Do not temporarily move /opt/ros
  --skip-system-deps       Skip installing system dependencies
  --force                  Force reinstall even if already present
  --help                   Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --recorder-tag)
      RECORDER_TAG="$2"
      shift 2
      ;;
    --pipe-tag)
      DDS_PIPE_TAG="$2"
      shift 2
      ;;
    --dev-utils-tag)
      DEV_UTILS_TAG="$2"
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
  ${SUDO} apt-get install -y qtbase5-dev liblz4-dev libzstd-dev libyaml-cpp-dev
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

CMAKE_COMMON_ARGS=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
  -DCMAKE_SYSTEM_PREFIX_PATH="${INSTALL_PREFIX}"
  -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}"
)

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# --- Dev Utils ---
cd "${BUILD_DIR}"
wget "https://github.com/eProsima/dev-utils/archive/refs/tags/v${DEV_UTILS_TAG}.tar.gz" -O "dev-utils-${DEV_UTILS_TAG}.tar.gz"
tar xf "dev-utils-${DEV_UTILS_TAG}.tar.gz"
DEV_UTILS_SRC="${BUILD_DIR}/dev-utils-${DEV_UTILS_TAG}"

# cmake_utils
cmake -S "${DEV_UTILS_SRC}/cmake_utils" -B "${BUILD_DIR}/cmake_utils-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/cmake_utils-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/cmake_utils-build"

# cpp_utils
cmake -S "${DEV_UTILS_SRC}/cpp_utils" -B "${BUILD_DIR}/cpp_utils-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/cpp_utils-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/cpp_utils-build"

# --- DDS Pipe ---
cd "${BUILD_DIR}"
wget "https://github.com/eProsima/DDS-Pipe/archive/refs/tags/v${DDS_PIPE_TAG}.tar.gz" -O "DDS-Pipe-${DDS_PIPE_TAG}.tar.gz"
tar xf "DDS-Pipe-${DDS_PIPE_TAG}.tar.gz"
PIPE_SRC="${BUILD_DIR}/DDS-Pipe-${DDS_PIPE_TAG}"

# ddspipe_core
cmake -S "${PIPE_SRC}/ddspipe_core" -B "${BUILD_DIR}/ddspipe_core-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddspipe_core-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddspipe_core-build"

# ddspipe_participants
cmake -S "${PIPE_SRC}/ddspipe_participants" -B "${BUILD_DIR}/ddspipe_participants-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddspipe_participants-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddspipe_participants-build"

# ddspipe_yaml
cmake -S "${PIPE_SRC}/ddspipe_yaml" -B "${BUILD_DIR}/ddspipe_yaml-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddspipe_yaml-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddspipe_yaml-build"

# --- DDS Record/Replay ---
cd "${BUILD_DIR}"
wget "https://github.com/eProsima/DDS-Record-Replay/archive/refs/tags/v${RECORDER_TAG}.tar.gz" -O "DDS-Record-Replay-${RECORDER_TAG}.tar.gz"
tar xf "DDS-Record-Replay-${RECORDER_TAG}.tar.gz"
REC_SRC="${BUILD_DIR}/DDS-Record-Replay-${RECORDER_TAG}"

# ddsrecorder_participants
cmake -S "${REC_SRC}/ddsrecorder_participants" -B "${BUILD_DIR}/ddsrecorder_participants-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddsrecorder_participants-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddsrecorder_participants-build"

# controller_tool
cmake -S "${REC_SRC}/controller/controller_tool" -B "${BUILD_DIR}/controller_tool-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/controller_tool-build" -j"${JOBS}" || true

# ddsrecorder_yaml
cmake -S "${REC_SRC}/ddsrecorder_yaml" -B "${BUILD_DIR}/ddsrecorder_yaml-build" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddsrecorder_yaml-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddsrecorder_yaml-build"

# ddsrecorder
cmake -S "${REC_SRC}/ddsrecorder" -B "${BUILD_DIR}/ddsrecorder-build" \
  "${CMAKE_COMMON_ARGS[@]}" \
  -Dfastcdr_DIR="${INSTALL_PREFIX}/lib/cmake/fastcdr/" \
  -Dfoonathan_memory_DIR="${INSTALL_PREFIX}/lib/foonathan_memory/cmake/" \
  -Dfastrtps_DIR="${INSTALL_PREFIX}/share/fastrtps/cmake/"
cmake --build "${BUILD_DIR}/ddsrecorder-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddsrecorder-build"

echo "[OK] Installed DDS-Record-Replay v${RECORDER_TAG} to ${INSTALL_PREFIX}"
