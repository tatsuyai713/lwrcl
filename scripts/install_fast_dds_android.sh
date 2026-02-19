#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

FAST_DDS_TAG="2.11.2"
FOONATHAN_MEMORY_TAG="1.3.1"
ANDROID_ABI="arm64-v8a"
ANDROID_API="31"
BUILD_TYPE="Release"
BUILD_DIR="${ROOT_DIR}/dds_build_android"
INSTALL_PREFIX=""
JOBS="4"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS] [clean]

Options:
  --prefix <path>       Install prefix (default: \${ROOT}/lwrcl/android/\${ABI})
  --tag <version>       Fast-DDS version (default: ${FAST_DDS_TAG})
  --abi <abi>           Android ABI (default: ${ANDROID_ABI})
  --api <level>         Android API level (default: ${ANDROID_API})
  --ndk <path>          Android NDK path (default: \$ANDROID_NDK_HOME)
  --build-dir <path>    Build directory (default: \${ROOT}/dds_build_android)
  --jobs <N>            Parallel build jobs (default: ${JOBS})
  --help                Show this help message

Positional:
  clean                 Remove build directory and exit
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
      FAST_DDS_TAG="$2"
      shift 2
      ;;
    --abi)
      ANDROID_ABI="$2"
      shift 2
      ;;
    --api)
      ANDROID_API="$2"
      shift 2
      ;;
    --ndk)
      ANDROID_NDK_HOME="$2"
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
    --help)
      usage
      ;;
    clean)
      rm -rf "${BUILD_DIR}"
      echo "[OK] Cleaned build directory: ${BUILD_DIR}"
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

NDK="${ANDROID_NDK_HOME:-${ANDROID_NDK:-${NDK:-}}}"
if [[ -z "${NDK}" ]]; then
  echo "[ERROR] ANDROID_NDK_HOME is not set. Use --ndk <path> or export ANDROID_NDK_HOME." >&2
  exit 1
fi

TOOLCHAIN_FILE="${NDK}/build/cmake/android.toolchain.cmake"
if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
  echo "[ERROR] Android toolchain file not found: ${TOOLCHAIN_FILE}" >&2
  exit 1
fi

HOST_ARCH="$(uname -m)"
HOST_TAG="linux-x86_64"
case "${HOST_ARCH}" in
  x86_64) HOST_TAG="linux-x86_64" ;;
  aarch64|arm64) HOST_TAG="linux-aarch64" ;;
esac

NDK_PREBUILT="${NDK}/toolchains/llvm/prebuilt/${HOST_TAG}"
if [[ ! -d "${NDK_PREBUILT}" ]]; then
  echo "[ERROR] NDK host toolchain not found: ${NDK_PREBUILT}" >&2
  echo "Host arch: ${HOST_ARCH}. Install an NDK with ${HOST_TAG} support or use an x86_64 environment."
  exit 1
fi

if [[ -z "${INSTALL_PREFIX}" ]]; then
  INSTALL_PREFIX="${ROOT_DIR}/lwrcl/android/${ANDROID_ABI}"
fi

echo "=== Building Fast-DDS v${FAST_DDS_TAG} for Android (ABI=${ANDROID_ABI}, API=${ANDROID_API}) ==="

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

if [[ ! -d "Fast-DDS" ]]; then
  git clone https://github.com/eProsima/Fast-DDS.git -b "v${FAST_DDS_TAG}" --depth 1
fi

cd Fast-DDS
WORKSPACE="$PWD"
git submodule update --init "${WORKSPACE}/thirdparty/asio" "${WORKSPACE}/thirdparty/fastcdr" "${WORKSPACE}/thirdparty/tinyxml2"
cd "${BUILD_DIR}"

if [[ ! -d "foonathan_memory_vendor" ]]; then
  git clone https://github.com/eProsima/foonathan_memory_vendor.git -b "v${FOONATHAN_MEMORY_TAG}"
fi

ANDROID_CMAKE_ARGS=(
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"
  -DANDROID_ABI="${ANDROID_ABI}"
  -DANDROID_PLATFORM="android-${ANDROID_API}"
  -DANDROID_STL=c++_shared
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
  -DBUILD_SHARED_LIBS=ON
  -DBUILD_TESTING=OFF
)

echo "Building foonathan_memory_vendor..."
cmake -S "${BUILD_DIR}/foonathan_memory_vendor" -B "${BUILD_DIR}/foonathan_memory_vendor/build" "${ANDROID_CMAKE_ARGS[@]}"
cmake --build "${BUILD_DIR}/foonathan_memory_vendor/build" -j "${JOBS}"
cmake --install "${BUILD_DIR}/foonathan_memory_vendor/build" --prefix "${INSTALL_PREFIX}"

echo "Building Fast-CDR..."
cmake -S "${WORKSPACE}/thirdparty/fastcdr" -B "${WORKSPACE}/thirdparty/fastcdr/build_android_${ANDROID_ABI}" "${ANDROID_CMAKE_ARGS[@]}"
cmake --build "${WORKSPACE}/thirdparty/fastcdr/build_android_${ANDROID_ABI}" -j "${JOBS}"
cmake --install "${WORKSPACE}/thirdparty/fastcdr/build_android_${ANDROID_ABI}" --prefix "${INSTALL_PREFIX}"

echo "Building TinyXML2..."
cmake -S "${WORKSPACE}/thirdparty/tinyxml2" -B "${WORKSPACE}/thirdparty/tinyxml2/build_android_${ANDROID_ABI}" "${ANDROID_CMAKE_ARGS[@]}"
cmake --build "${WORKSPACE}/thirdparty/tinyxml2/build_android_${ANDROID_ABI}" -j "${JOBS}"
cmake --install "${WORKSPACE}/thirdparty/tinyxml2/build_android_${ANDROID_ABI}" --prefix "${INSTALL_PREFIX}"

echo "Building Fast DDS..."
cmake -S "${WORKSPACE}" -B "${WORKSPACE}/build_android_${ANDROID_ABI}" \
  "${ANDROID_CMAKE_ARGS[@]}" \
  -Dfastcdr_DIR="${INSTALL_PREFIX}/lib/cmake/fastcdr" \
  -Dfoonathan_memory_DIR="${INSTALL_PREFIX}/lib/foonathan_memory/cmake" \
  -Dtinyxml2_DIR="${INSTALL_PREFIX}/lib/cmake/tinyxml2" \
  -DASIO_INCLUDE_DIR="${WORKSPACE}/thirdparty/asio/asio/include" \
  -DBUILD_TESTS=OFF
cmake --build "${WORKSPACE}/build_android_${ANDROID_ABI}" -j "${JOBS}"
cmake --install "${WORKSPACE}/build_android_${ANDROID_ABI}" --prefix "${INSTALL_PREFIX}"

echo "[OK] Installed Fast-DDS v${FAST_DDS_TAG} for Android to ${INSTALL_PREFIX}"
