#!/usr/bin/env bash
set -euo pipefail

# Based on: https://qiita.com/taiyodayo/items/2067d276031c3cf42b55

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ANDROID_SDK_SRC="/usr/lib/android-sdk"
ANDROID_HOME="${HOME}/android-sdk"
ANDROID_PLATFORM="android-29"
ANDROID_BUILD_TOOLS="29.0.3"
ANDROID_NDK_VERSION=""
ACCEPT_LICENSES="1"
BUILD_AARCH64_TOOLCHAIN=""
AARCH64_TOOLCHAIN_SOURCE_ROOT="${HOME}/llvm-toolchain"
SKIP_SYSTEM_DEPS="OFF"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --android-home <path>     Android SDK install path (default: \${HOME}/android-sdk)
  --platform <platform>     Android platform (default: ${ANDROID_PLATFORM})
  --build-tools <version>   Build tools version (default: ${ANDROID_BUILD_TOOLS})
  --ndk-version <version>   NDK version (default: auto per arch)
  --no-accept-licenses      Do not auto-accept licenses
  --no-aarch64-toolchain    Do not build aarch64 toolchain
  --skip-system-deps        Skip installing system dependencies
  --help                    Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --android-home)
      ANDROID_HOME="$2"
      shift 2
      ;;
    --platform)
      ANDROID_PLATFORM="$2"
      shift 2
      ;;
    --build-tools)
      ANDROID_BUILD_TOOLS="$2"
      shift 2
      ;;
    --ndk-version)
      ANDROID_NDK_VERSION="$2"
      shift 2
      ;;
    --no-accept-licenses)
      ACCEPT_LICENSES="0"
      shift
      ;;
    --no-aarch64-toolchain)
      BUILD_AARCH64_TOOLCHAIN="0"
      shift
      ;;
    --skip-system-deps)
      SKIP_SYSTEM_DEPS="ON"
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

HOST_ARCH="$(uname -m)"
HOST_TAG="linux-x86_64"
case "${HOST_ARCH}" in
  x86_64) HOST_TAG="linux-x86_64" ;;
  aarch64|arm64) HOST_TAG="linux-aarch64" ;;
esac

if [[ -z "${ANDROID_NDK_VERSION}" ]]; then
  if [[ "${HOST_TAG}" = "linux-aarch64" ]]; then
    ANDROID_NDK_VERSION="26.1.10909125"
  else
    ANDROID_NDK_VERSION="25.2.9519653"
  fi
fi

if [[ -z "${BUILD_AARCH64_TOOLCHAIN}" ]]; then
  if [[ "${HOST_TAG}" = "linux-aarch64" ]]; then
    BUILD_AARCH64_TOOLCHAIN="1"
  else
    BUILD_AARCH64_TOOLCHAIN="0"
  fi
fi

CMDLINE_TOOLS_URL="https://dl.google.com/android/repository/commandlinetools-linux-10406996_latest.zip"
CMDLINE_TOOLS_ZIP="${HOME}/commandlinetools.zip"
WORK_DIR="${HOME}/android-sdk-tmp"

if ! command -v apt >/dev/null 2>&1; then
  echo "[ERROR] apt not found. This script targets Ubuntu/Debian." >&2
  exit 1
fi

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
  else
    echo "[ERROR] Please run as root or install sudo." >&2
    exit 1
  fi
fi

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]]; then
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y android-sdk openjdk-17-jdk unzip wget
fi

if [[ ! -d "${ANDROID_SDK_SRC}" ]]; then
  echo "[ERROR] Android SDK not found at ${ANDROID_SDK_SRC}" >&2
  exit 1
fi

if [[ ! -d "${ANDROID_HOME}" ]]; then
  ${SUDO} cp -rf "${ANDROID_SDK_SRC}" "${ANDROID_HOME}"
  ${SUDO} chown -R "$(id -u)":"$(id -g)" "${ANDROID_HOME}"
fi

mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}"

if [[ ! -f "${CMDLINE_TOOLS_ZIP}" ]]; then
  wget -O "${CMDLINE_TOOLS_ZIP}" "${CMDLINE_TOOLS_URL}"
fi

TMP_DIR="$(mktemp -d)"
unzip -q "${CMDLINE_TOOLS_ZIP}" -d "${TMP_DIR}"
mkdir -p "${ANDROID_HOME}/cmdline-tools"
if [[ -d "${ANDROID_HOME}/cmdline-tools/latest" ]]; then
  rm -rf "${ANDROID_HOME}/cmdline-tools/latest"
fi
mv "${TMP_DIR}/cmdline-tools" "${ANDROID_HOME}/cmdline-tools/latest"
rm -rf "${TMP_DIR}"

JAVA_HOME="$(readlink -f /usr/bin/javac | sed "s:/bin/javac::")"
export JAVA_HOME
export ANDROID_HOME
export PATH="$PATH:$JAVA_HOME/bin:$ANDROID_HOME/cmdline-tools/latest/bin:$ANDROID_HOME/tools/bin:$ANDROID_HOME/platform-tools/bin"

SHELL_RC=""
if [[ -n "${ZSH_VERSION:-}" ]]; then
  SHELL_RC="$HOME/.zshrc"
else
  SHELL_RC="$HOME/.bashrc"
fi

MARK_BEGIN="# >>> lwrcl android sdk >>>"
MARK_END="# <<< lwrcl android sdk <<<"

if [[ -f "${SHELL_RC}" ]]; then
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
  echo "export ANDROID_HOME=${ANDROID_HOME}"
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
if [[ ! -x "${SDKMANAGER}" ]]; then
  echo "[ERROR] sdkmanager not found at ${SDKMANAGER}" >&2
  exit 1
fi

if [[ "${ACCEPT_LICENSES}" = "1" ]]; then
  set +e
  yes | "${SDKMANAGER}" --sdk_root="${ANDROID_HOME}" --licenses >/dev/null
  LICENSE_STATUS=$?
  set -e
  if [[ "${LICENSE_STATUS}" -ne 0 ]] && [[ "${LICENSE_STATUS}" -ne 141 ]]; then
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

if [[ "${SDK_STATUS}" -ne 0 ]]; then
  echo "sdkmanager returned ${SDK_STATUS}. If you see build-tools location warnings, they are usually harmless."
fi

NDK_PREBUILT="${ANDROID_HOME}/ndk/${ANDROID_NDK_VERSION}/toolchains/llvm/prebuilt/${HOST_TAG}"
if [[ ! -d "${NDK_PREBUILT}" ]]; then
  echo "NDK host toolchain not found: ${NDK_PREBUILT}"
  echo "Host arch: ${HOST_ARCH}."
  if [[ "${HOST_TAG}" = "linux-aarch64" ]] && [[ "${BUILD_AARCH64_TOOLCHAIN}" = "1" ]]; then
    BUILD_SCRIPT="${SCRIPT_DIR}/build_ndk_aarch64_linux.sh"
    if [[ ! -x "${BUILD_SCRIPT}" ]]; then
      echo "[ERROR] build script not found: ${BUILD_SCRIPT}" >&2
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
    echo "  ./scripts/install_android_sdk_ubuntu.sh --ndk-version 26.1.10909125"
    echo "  or run an amd64 container: docker run --platform=linux/amd64 ..."
    echo "If you want to skip aarch64 toolchain build, use:"
    echo "  ./scripts/install_android_sdk_ubuntu.sh --no-aarch64-toolchain"
    exit 1
  fi
fi

echo "[OK] Android SDK setup complete."
echo "Reload your shell: source ${SHELL_RC}"
