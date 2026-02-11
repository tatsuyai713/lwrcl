#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

FLUTTER_APP_DIR="${FLUTTER_APP_DIR:-}"
if [ -z "${FLUTTER_APP_DIR}" ]; then
  echo "FLUTTER_APP_DIR is not set (path to your Flutter app)."
  exit 1
fi

ANDROID_ABI="${ANDROID_ABI:-arm64-v8a}"
ANDROID_API="${ANDROID_API:-31}"
ANDROID_PREFIX="${ANDROID_PREFIX:-${ROOT_DIR}/lwrcl/android/${ANDROID_ABI}}"
LIB_DIR="${ANDROID_PREFIX}/lib"

if [ ! -d "${LIB_DIR}" ]; then
  echo "Library directory not found: ${LIB_DIR}"
  exit 1
fi

JNI_DIR="${FLUTTER_APP_DIR}/android/app/src/main/jniLibs/${ANDROID_ABI}"
mkdir -p "${JNI_DIR}"

echo "Copying .so files from ${LIB_DIR} to ${JNI_DIR}"
cp -f "${LIB_DIR}"/*.so "${JNI_DIR}/"

if [ "${COPY_LIBCXX_SHARED:-1}" = "1" ]; then
  NDK="${ANDROID_NDK_HOME:-${ANDROID_NDK:-${NDK:-}}}"
  if [ -z "${NDK}" ]; then
    echo "ANDROID_NDK_HOME is not set; skipping libc++_shared.so"
    exit 0
  fi

  case "${ANDROID_ABI}" in
    arm64-v8a) TRIPLE="aarch64-linux-android" ;;
    armeabi-v7a) TRIPLE="arm-linux-androideabi" ;;
    x86) TRIPLE="i686-linux-android" ;;
    x86_64) TRIPLE="x86_64-linux-android" ;;
    *)
      echo "Unsupported ANDROID_ABI: ${ANDROID_ABI} (skipping libc++_shared.so)"
      exit 0
      ;;
  esac

  PREBUILT_DIR="$(ls -1 "${NDK}/toolchains/llvm/prebuilt" | head -n 1)"
  if [ -z "${PREBUILT_DIR}" ]; then
    echo "NDK prebuilt dir not found; skipping libc++_shared.so"
    exit 0
  fi

  CXX_SHARED_PATH="${NDK}/toolchains/llvm/prebuilt/${PREBUILT_DIR}/sysroot/usr/lib/${TRIPLE}/${ANDROID_API}/libc++_shared.so"
  if [ ! -f "${CXX_SHARED_PATH}" ]; then
    CXX_SHARED_PATH="${NDK}/toolchains/llvm/prebuilt/${PREBUILT_DIR}/sysroot/usr/lib/${TRIPLE}/libc++_shared.so"
  fi

  if [ -f "${CXX_SHARED_PATH}" ]; then
    echo "Copying libc++_shared.so to ${JNI_DIR}"
    cp -f "${CXX_SHARED_PATH}" "${JNI_DIR}/"
  else
    echo "libc++_shared.so not found for ${ANDROID_ABI}; skipping"
  fi
fi
