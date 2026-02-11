#!/bin/bash

set -euo pipefail

# Based on: https://qiita.com/taiyodayo/items/2067d276031c3cf42b55

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANDROID_SDK_SRC="/usr/lib/android-sdk"
ANDROID_HOME="${ANDROID_HOME:-$HOME/android-sdk}"

CMDLINE_TOOLS_URL="${CMDLINE_TOOLS_URL:-https://dl.google.com/android/repository/commandlinetools-linux-10406996_latest.zip}"
CMDLINE_TOOLS_ZIP="${CMDLINE_TOOLS_ZIP:-$HOME/commandlinetools.zip}"
WORK_DIR="${WORK_DIR:-$HOME/android-sdk-tmp}"

ANDROID_PLATFORM="${ANDROID_PLATFORM:-android-29}"
ANDROID_BUILD_TOOLS="${ANDROID_BUILD_TOOLS:-29.0.3}"

HOST_ARCH="$(uname -m)"
HOST_TAG="linux-x86_64"
case "${HOST_ARCH}" in
  x86_64) HOST_TAG="linux-x86_64" ;;
  aarch64|arm64) HOST_TAG="linux-aarch64" ;;
esac

if [ -z "${ANDROID_NDK_VERSION:-}" ]; then
  if [ "${HOST_TAG}" = "linux-aarch64" ]; then
    ANDROID_NDK_VERSION="26.1.10909125"
  else
    ANDROID_NDK_VERSION="25.2.9519653"
  fi
fi
ACCEPT_LICENSES="${ACCEPT_LICENSES:-1}"
# Auto-enable aarch64 toolchain build if host is aarch64 and prebuilt is missing.
if [ "${HOST_TAG}" = "linux-aarch64" ]; then
  BUILD_AARCH64_TOOLCHAIN_DEFAULT=1
else
  BUILD_AARCH64_TOOLCHAIN_DEFAULT=0
fi
BUILD_AARCH64_TOOLCHAIN="${BUILD_AARCH64_TOOLCHAIN:-${BUILD_AARCH64_TOOLCHAIN_DEFAULT}}"
AARCH64_TOOLCHAIN_SOURCE_ROOT="${AARCH64_TOOLCHAIN_SOURCE_ROOT:-$HOME/llvm-toolchain}"

if ! command -v apt >/dev/null 2>&1; then
  echo "apt not found. This script targets Ubuntu/Debian."
  exit 1
fi

#sudo apt update
sudo apt install -y android-sdk openjdk-17-jdk unzip wget

if [ ! -d "${ANDROID_SDK_SRC}" ]; then
  echo "Android SDK not found at ${ANDROID_SDK_SRC}"
  exit 1
fi

if [ ! -d "${ANDROID_HOME}" ]; then
  sudo cp -rf "${ANDROID_SDK_SRC}" "${ANDROID_HOME}"
  sudo chown -R "$(id -u)":"$(id -g)" "${ANDROID_HOME}"
fi

mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}"

if [ ! -f "${CMDLINE_TOOLS_ZIP}" ]; then
  wget -O "${CMDLINE_TOOLS_ZIP}" "${CMDLINE_TOOLS_URL}"
fi

TMP_DIR="$(mktemp -d)"
unzip -q "${CMDLINE_TOOLS_ZIP}" -d "${TMP_DIR}"
mkdir -p "${ANDROID_HOME}/cmdline-tools"
if [ -d "${ANDROID_HOME}/cmdline-tools/latest" ]; then
  rm -rf "${ANDROID_HOME}/cmdline-tools/latest"
fi
mv "${TMP_DIR}/cmdline-tools" "${ANDROID_HOME}/cmdline-tools/latest"
rm -rf "${TMP_DIR}"

JAVA_HOME="$(readlink -f /usr/bin/javac | sed "s:/bin/javac::")"
export JAVA_HOME
export ANDROID_HOME
export PATH="$PATH:$JAVA_HOME/bin:$ANDROID_HOME/cmdline-tools/latest/bin:$ANDROID_HOME/tools/bin:$ANDROID_HOME/platform-tools/bin"

SHELL_RC=""
if [ -n "${ZSH_VERSION:-}" ]; then
  SHELL_RC="$HOME/.zshrc"
elif [ -n "${BASH_VERSION:-}" ]; then
  SHELL_RC="$HOME/.bashrc"
else
  SHELL_RC="$HOME/.bashrc"
fi

MARK_BEGIN="# >>> lwrcl android sdk >>>"
MARK_END="# <<< lwrcl android sdk <<<"

if [ -f "${SHELL_RC}" ]; then
  awk "BEGIN{skip=0} \
       \$0==\"${MARK_BEGIN}\"{skip=1} \
       \$0==\"${MARK_END}\"{skip=0; next} \
       skip==0{print}" "${SHELL_RC}" > "${SHELL_RC}.tmp"
  mv "${SHELL_RC}.tmp" "${SHELL_RC}"
fi

{
  echo "${MARK_BEGIN}"
  echo "JAVA_HOME=\$(readlink -f /usr/bin/javac | sed \"s:/bin/javac::\")"
  echo "export JAVA_HOME"
  echo "export ANDROID_HOME=\${HOME}/android-sdk"
  echo "export ANDROID_SDK_ROOT=\$ANDROID_HOME"
  echo "export PATH=\$PATH:\$JAVA_HOME/bin:\$ANDROID_HOME/cmdline-tools/latest/bin:\$ANDROID_HOME/tools/bin:\$ANDROID_HOME/platform-tools/bin"
  echo ""
  echo "if [ -z \"\${ANDROID_NDK_VERSION:-}\" ]; then export ANDROID_NDK_VERSION=${ANDROID_NDK_VERSION}; fi"
  echo "if [ -z \"\${ANDROID_NDK_HOME:-}\" ] && [ -d \"\$ANDROID_HOME/ndk/\$ANDROID_NDK_VERSION\" ]; then"
  echo "  export ANDROID_NDK_HOME=\"\$ANDROID_HOME/ndk/\$ANDROID_NDK_VERSION\""
  echo "fi"
  echo "# Auto-detect newest installed NDK if ANDROID_NDK_HOME is unset"
  echo "if [ -z \"\${ANDROID_NDK_HOME:-}\" ] && [ -d \"\$ANDROID_HOME/ndk\" ]; then"
  echo "  ANDROID_NDK_HOME=\$(ls -d \"\$ANDROID_HOME/ndk/\"* 2>/dev/null | sort -V | tail -n 1)"
  echo "  export ANDROID_NDK_HOME"
  echo "fi"
  echo ""
  echo "# Defaults for lwrcl/fastdds Android builds (override per shell if needed)"
  echo "if [ -z \"\${ANDROID_ABI:-}\" ]; then export ANDROID_ABI=arm64-v8a; fi"
  echo "if [ -z \"\${ANDROID_API:-}\" ]; then export ANDROID_API=31; fi"
  echo "if [ -z \"\${LWRCL_ROOT:-}\" ]; then"
  echo "  # Optional: set LWRCL_ROOT to your repo path to enable ANDROID_PREFIX defaults"
  echo "  :"
  echo "fi"
  echo "if [ -z \"\${ANDROID_PREFIX:-}\" ] && [ -n \"\${LWRCL_ROOT:-}\" ]; then"
  echo "  export ANDROID_PREFIX=\"\$LWRCL_ROOT/lwrcl/android/\${ANDROID_ABI}\""
  echo "fi"
  echo "if [ -z \"\${FAST_DDS_PREFIX:-}\" ] && [ -n \"\${ANDROID_PREFIX:-}\" ]; then"
  echo "  export FAST_DDS_PREFIX=\"\$ANDROID_PREFIX\""
  echo "fi"
  echo "${MARK_END}"
} >> "${SHELL_RC}"

SDKMANAGER="${ANDROID_HOME}/cmdline-tools/latest/bin/sdkmanager"
if [ ! -x "${SDKMANAGER}" ]; then
  echo "sdkmanager not found at ${SDKMANAGER}"
  exit 1
fi

if [ "${ACCEPT_LICENSES}" = "1" ]; then
  set +e
  yes | "${SDKMANAGER}" --sdk_root="${ANDROID_HOME}" --licenses >/dev/null
  LICENSE_STATUS=$?
  set -e
  if [ "${LICENSE_STATUS}" -ne 0 ] && [ "${LICENSE_STATUS}" -ne 141 ]; then
    echo "sdkmanager --licenses failed with code ${LICENSE_STATUS}"
    exit "${LICENSE_STATUS}"
  fi
fi

set +e
"${SDKMANAGER}" --sdk_root="${ANDROID_HOME}" \
  "platform-tools" \
  "platforms;${ANDROID_PLATFORM}" \
  "build-tools;${ANDROID_BUILD_TOOLS}" \
  "ndk;${ANDROID_NDK_VERSION}"
SDK_STATUS=$?
set -e

if [ "${SDK_STATUS}" -ne 0 ]; then
  echo "sdkmanager returned ${SDK_STATUS}. If you see build-tools location warnings, they are usually harmless."
fi

NDK_PREBUILT="${ANDROID_HOME}/ndk/${ANDROID_NDK_VERSION}/toolchains/llvm/prebuilt/${HOST_TAG}"
if [ ! -d "${NDK_PREBUILT}" ]; then
  echo "NDK host toolchain not found: ${NDK_PREBUILT}"
  echo "Host arch: ${HOST_ARCH}."
  if [ "${HOST_TAG}" = "linux-aarch64" ] && [ "${BUILD_AARCH64_TOOLCHAIN}" = "1" ]; then
    BUILD_SCRIPT="${SCRIPT_DIR}/build_ndk_aarch64_linux.sh"
    if [ ! -x "${BUILD_SCRIPT}" ]; then
      echo "build script not found: ${BUILD_SCRIPT}"
      exit 1
    fi
    echo "Building aarch64 host toolchain into NDK (this takes a long time)..."
    NDK_ROOT="${ANDROID_HOME}/ndk/${ANDROID_NDK_VERSION}" \
    NDK_HOST_TAG="${HOST_TAG}" \
    SOURCE_ROOT="${AARCH64_TOOLCHAIN_SOURCE_ROOT}" \
    "${BUILD_SCRIPT}"
  else
    echo "You may need a different NDK version or an x86_64 environment."
    echo "Example:"
    echo "  ANDROID_NDK_VERSION=26.1.10909125 ./scripts/install_android_sdk_ubuntu.sh"
    echo "  or run an amd64 container: docker run --platform=linux/amd64 ..."
    echo "If you want to skip aarch64 toolchain build, set:"
    echo "  BUILD_AARCH64_TOOLCHAIN=0 ./scripts/install_android_sdk_ubuntu.sh"
    exit 1
  fi
fi

echo "Android SDK setup complete."
echo "Reload your shell: source ${SHELL_RC}"
