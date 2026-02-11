#!/bin/bash

set -euo pipefail

fast_dds_version="${FAST_DDS_VERSION:-2.11.2}"
foonathan_memory_vendor_version="${FOONATHAN_MEMORY_VENDOR_VERSION:-1.3.1}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

FAST_DDS_WORK_DIR="${FAST_DDS_WORK_DIR:-${ROOT_DIR}/dds_build_android}"

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

HOST_ARCH="$(uname -m)"
HOST_TAG="linux-x86_64"
case "${HOST_ARCH}" in
  x86_64) HOST_TAG="linux-x86_64" ;;
  aarch64|arm64) HOST_TAG="linux-aarch64" ;;
esac

NDK_PREBUILT="${NDK}/toolchains/llvm/prebuilt/${HOST_TAG}"
if [ ! -d "${NDK_PREBUILT}" ]; then
  echo "NDK host toolchain not found: ${NDK_PREBUILT}"
  echo "Host arch: ${HOST_ARCH}. Install an NDK with ${HOST_TAG} support or use an x86_64 environment."
  exit 1
fi

ANDROID_ABI="${ANDROID_ABI:-arm64-v8a}"
ANDROID_API="${ANDROID_API:-31}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
ANDROID_PREFIX="${ANDROID_PREFIX:-${ROOT_DIR}/lwrcl/android/${ANDROID_ABI}}"
FAST_DDS_PREFIX="${FAST_DDS_PREFIX:-${ANDROID_PREFIX}}"
JOBS="${JOBS:-4}"

OPT="${1:-}"
if [ "${OPT}" = "clean" ]; then
  rm -rf "${FAST_DDS_WORK_DIR}"
  exit 0
fi

mkdir -p "${FAST_DDS_WORK_DIR}"
cd "${FAST_DDS_WORK_DIR}"

if [ ! -d "Fast-DDS" ]; then
  git clone https://github.com/eProsima/Fast-DDS.git -b "v${fast_dds_version}" --depth 1
fi

cd Fast-DDS
WORKSPACE="$PWD"
git submodule update --init "${WORKSPACE}/thirdparty/asio" "${WORKSPACE}/thirdparty/fastcdr" "${WORKSPACE}/thirdparty/tinyxml2"
cd "${FAST_DDS_WORK_DIR}"

if [ ! -d "foonathan_memory_vendor" ]; then
  git clone https://github.com/eProsima/foonathan_memory_vendor.git -b "v${foonathan_memory_vendor_version}"
fi

echo "Building foonathan_memory_vendor..."
cmake -S "${FAST_DDS_WORK_DIR}/foonathan_memory_vendor" -B "${FAST_DDS_WORK_DIR}/foonathan_memory_vendor/build" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DANDROID_ABI="${ANDROID_ABI}" \
  -DANDROID_PLATFORM="android-${ANDROID_API}" \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${FAST_DDS_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF
cmake --build "${FAST_DDS_WORK_DIR}/foonathan_memory_vendor/build" -j "${JOBS}"
cmake --install "${FAST_DDS_WORK_DIR}/foonathan_memory_vendor/build" --prefix "${FAST_DDS_PREFIX}"

echo "Building Fast-CDR..."
cmake -S "${WORKSPACE}/thirdparty/fastcdr" -B "${WORKSPACE}/thirdparty/fastcdr/build_android_${ANDROID_ABI}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DANDROID_ABI="${ANDROID_ABI}" \
  -DANDROID_PLATFORM="android-${ANDROID_API}" \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${FAST_DDS_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF
cmake --build "${WORKSPACE}/thirdparty/fastcdr/build_android_${ANDROID_ABI}" -j "${JOBS}"
cmake --install "${WORKSPACE}/thirdparty/fastcdr/build_android_${ANDROID_ABI}" --prefix "${FAST_DDS_PREFIX}"

echo "Building TinyXML2..."
cmake -S "${WORKSPACE}/thirdparty/tinyxml2" -B "${WORKSPACE}/thirdparty/tinyxml2/build_android_${ANDROID_ABI}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DANDROID_ABI="${ANDROID_ABI}" \
  -DANDROID_PLATFORM="android-${ANDROID_API}" \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${FAST_DDS_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF
cmake --build "${WORKSPACE}/thirdparty/tinyxml2/build_android_${ANDROID_ABI}" -j "${JOBS}"
cmake --install "${WORKSPACE}/thirdparty/tinyxml2/build_android_${ANDROID_ABI}" --prefix "${FAST_DDS_PREFIX}"

echo "Building Fast DDS..."
cmake -S "${WORKSPACE}" -B "${WORKSPACE}/build_android_${ANDROID_ABI}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DANDROID_ABI="${ANDROID_ABI}" \
  -DANDROID_PLATFORM="android-${ANDROID_API}" \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${FAST_DDS_PREFIX}" \
  -Dfastcdr_DIR="${FAST_DDS_PREFIX}/lib/cmake/fastcdr" \
  -Dfoonathan_memory_DIR="${FAST_DDS_PREFIX}/lib/foonathan_memory/cmake" \
  -Dtinyxml2_DIR="${FAST_DDS_PREFIX}/lib/cmake/tinyxml2" \
  -DASIO_INCLUDE_DIR="${WORKSPACE}/thirdparty/asio/asio/include" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_TESTS=OFF
cmake --build "${WORKSPACE}/build_android_${ANDROID_ABI}" -j "${JOBS}"
cmake --install "${WORKSPACE}/build_android_${ANDROID_ABI}" --prefix "${FAST_DDS_PREFIX}"

echo "Fast DDS for Android installed to: ${FAST_DDS_PREFIX}"
