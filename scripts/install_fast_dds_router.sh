#!/usr/bin/env bash
set -euo pipefail

ROUTER_TAG="2.0.0"
INSTALL_PREFIX="/opt/fast-dds"
BUILD_DIR="${HOME}/build-fast-dds-router"
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
  --prefix <path>       Fast-DDS install prefix (default: ${INSTALL_PREFIX})
  --tag <version>       DDS-Router version (default: ${ROUTER_TAG})
  --build-dir <path>    Build directory (default: \${HOME}/build-fast-dds-router)
  --jobs <N>            Parallel build jobs (default: auto)
  --no-hide-ros         Do not temporarily move /opt/ros
  --skip-system-deps    Skip installing system dependencies
  --force               Force reinstall even if already present
  --help                Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --tag)
      ROUTER_TAG="$2"
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
  ${SUDO} pip3 install -U vcstool pyyaml jsonschema || true
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y libasio-dev libtinyxml2-dev libssl-dev libyaml-cpp-dev
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
cd "${BUILD_DIR}"

wget "https://github.com/eProsima/DDS-Router/archive/refs/tags/v${ROUTER_TAG}.tar.gz" -O "DDS-Router-${ROUTER_TAG}.tar.gz"
tar xf "DDS-Router-${ROUTER_TAG}.tar.gz"

SRC_DIR="${BUILD_DIR}/DDS-Router-${ROUTER_TAG}"
mkdir -p "${SRC_DIR}/src"
cd "${SRC_DIR}"
wget "https://raw.githubusercontent.com/eProsima/DDS-Router/main/ddsrouter.repos"
vcs import src < ddsrouter.repos

CMAKE_COMMON_ARGS=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
  -DCMAKE_SYSTEM_PREFIX_PATH="${INSTALL_PREFIX}"
  -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}"
)

# CMake Utils
cmake -S "${SRC_DIR}/src/dev-utils/cmake_utils" -B "${BUILD_DIR}/cmake_utils" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/cmake_utils" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/cmake_utils"

# C++ Utils
cmake -S "${SRC_DIR}/src/dev-utils/cpp_utils" -B "${BUILD_DIR}/cpp_utils" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/cpp_utils" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/cpp_utils"

# ddsrouter_core
cmake -S "${SRC_DIR}/src/ddsrouter/ddsrouter_core" -B "${BUILD_DIR}/ddsrouter_core" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddsrouter_core" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddsrouter_core"

# ddsrouter_yaml
cmake -S "${SRC_DIR}/src/ddsrouter/ddsrouter_yaml" -B "${BUILD_DIR}/ddsrouter_yaml" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddsrouter_yaml" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddsrouter_yaml"

# ddsrouter_tool
cmake -S "${SRC_DIR}/src/ddsrouter/tools/ddsrouter_tool" -B "${BUILD_DIR}/ddsrouter_tool" "${CMAKE_COMMON_ARGS[@]}"
cmake --build "${BUILD_DIR}/ddsrouter_tool" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/ddsrouter_tool"

echo "[OK] Installed DDS-Router v${ROUTER_TAG} to ${INSTALL_PREFIX}"
