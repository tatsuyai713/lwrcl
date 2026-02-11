#!/bin/bash
set -euo pipefail

OPT="${1:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}"

NDK="${ANDROID_NDK_HOME:-${ANDROID_NDK:-${NDK:-}}}"
if [ -z "${NDK}" ]; then
  echo "ANDROID_NDK_HOME is not set."
  exit 1
fi

TOOLCHAIN_FILE="${NDK}/build/cmake/android.toolchain.cmake"
if [ ! -f "${TOOLCHAIN_FILE}" ]; then
  echo "Android toolchain file not found: ${TOOLCHAIN_FILE}"
  exit 1
fi

ANDROID_ABI="${ANDROID_ABI:-arm64-v8a}"
ANDROID_API="${ANDROID_API:-31}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
ANDROID_PREFIX="${ANDROID_PREFIX:-${ROOT_DIR}/android/${ANDROID_ABI}}"
FAST_DDS_PREFIX="${FAST_DDS_PREFIX:-${ANDROID_PREFIX}}"
JOBS="${JOBS:-4}"

if [ "${OPT}" = "clean" ]; then
  rm -rf "${ROOT_DIR}/data_types/build_android/${ANDROID_ABI}"
  exit 0
fi

BUILD_DIR="${ROOT_DIR}/data_types/build_android/${ANDROID_ABI}"

cmake -S "${ROOT_DIR}/data_types" -B "${BUILD_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DANDROID_ABI="${ANDROID_ABI}" \
  -DANDROID_PLATFORM="android-${ANDROID_API}" \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${ANDROID_PREFIX}" \
  -DDDS_BACKEND=fastdds \
  -DCMAKE_SYSTEM_PREFIX_PATH="${FAST_DDS_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${FAST_DDS_PREFIX}" \
  -Dfastcdr_DIR="${FAST_DDS_PREFIX}/lib/cmake/fastcdr" \
  -Dfastrtps_DIR="${FAST_DDS_PREFIX}/share/fastrtps/cmake" \
  -Dfoonathan_memory_DIR="${FAST_DDS_PREFIX}/lib/foonathan_memory/cmake" \
  -Dtinyxml2_DIR="${FAST_DDS_PREFIX}/lib/cmake/tinyxml2" \
  -DLWRCL_INSTALL_PREFIX="${FAST_DDS_PREFIX}"

cmake --build "${BUILD_DIR}" -j "${JOBS}"

if [ "${OPT}" = "install" ]; then
  cmake --install "${BUILD_DIR}" --prefix "${ANDROID_PREFIX}"
fi
