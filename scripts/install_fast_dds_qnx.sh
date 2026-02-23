#!/usr/bin/env bash
set -euo pipefail

QNX_PATH="qnx800"
FAST_DDS_TAG="2.11.2"
FOONATHAN_MEMORY_TAG="1.3.1"
GOOGLETEST_TAG="1.13.0"
INSTALL_PREFIX="/opt/qnx/fast-dds"
BUILD_DIR="${HOME}/build-fast-dds-qnx"
_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"
TARGET_VARIANT="${TARGET_VARIANT:-aarch64le}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "${SCRIPT_DIR}/patches" ]]; then
  PATCHES_DIR="$(cd "${SCRIPT_DIR}/patches" && pwd)"
elif [[ -d "${SCRIPT_DIR}/../patches" ]]; then
  PATCHES_DIR="$(cd "${SCRIPT_DIR}/../patches" && pwd)"
else
  echo "[ERROR] QNX patch directory not found (expected scripts/patches)." >&2
  exit 1
fi

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --prefix <path>       Install prefix (default: ${INSTALL_PREFIX})
  --tag <version>       Fast-DDS version (default: ${FAST_DDS_TAG})
  --qnx-path <name>    QNX SDP directory name in HOME (default: ${QNX_PATH})
  --build-dir <path>    Build directory (default: \${HOME}/build-fast-dds-qnx)
  --jobs <N>            Parallel build jobs (default: auto)
  --target-variant <v>  QNX target variant: aarch64le|x86_64o (default: ${TARGET_VARIANT})
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
      FAST_DDS_TAG="$2"
      shift 2
      ;;
    --qnx-path)
      QNX_PATH="$2"
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
    --target-variant)
      TARGET_VARIANT="$2"
      shift 2
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

case "${TARGET_VARIANT}" in
  aarch64le)
    QNX_BUILD_DIR_SUFFIX="nto/aarch64/le"
    ;;
  x86_64o)
    QNX_BUILD_DIR_SUFFIX="nto/x86_64/o"
    ;;
  *)
    echo "[ERROR] Unsupported --target-variant: ${TARGET_VARIANT}" >&2
    echo "        Supported values: aarch64le, x86_64o" >&2
    exit 1
    ;;
esac

if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ -f "${INSTALL_PREFIX}/include/fastrtps/fastrtps_all.h" ]]; then
  echo "[INFO] Fast-DDS already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
  exit 0
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

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]] && command -v apt-get >/dev/null 2>&1; then
  wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | ${SUDO} tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
  ${SUDO} apt-add-repository -y "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y gcc g++ make cmake automake autoconf unzip git openssl curl tar wget p11-kit dos2unix
fi

${SUDO} mkdir -p "${INSTALL_PREFIX}"
${SUDO} chmod 777 -R "${INSTALL_PREFIX}"

# shellcheck disable=SC1090
source ~/"${QNX_PATH}/qnxsdp-env.sh"

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

git clone https://github.com/eProsima/Fast-DDS.git -b "v${FAST_DDS_TAG}" --depth 1
cd Fast-DDS
WORKSPACE="$PWD"

echo "Patch Fast-DDS"
git apply "${PATCHES_DIR}/fastdds-qnx.patch"
echo "Patch Fast-DDS done"

git submodule update --init "${WORKSPACE}/thirdparty/asio" "${WORKSPACE}/thirdparty/fastcdr" "${WORKSPACE}/thirdparty/tinyxml2"

cd "${WORKSPACE}/thirdparty/asio"
echo "Patch Asio"
git apply "${WORKSPACE}/build_qnx/qnx_patches/asio_qnx.patch"
echo "Patch Asio done"

cd "${WORKSPACE}/thirdparty/fastcdr"
echo "Patch FastCDR"
git apply "${WORKSPACE}/build_qnx/qnx_patches/fastcdr_qnx.patch"
echo "Patch FastCDR done"

cd "${WORKSPACE}/thirdparty/tinyxml2"
# Keep compatibility with environments where unix2dos is unavailable.
if command -v unix2dos >/dev/null 2>&1; then
  unix2dos "${WORKSPACE}/build_qnx/qnx_patches/tinyxml2_qnx.patch"
fi
echo "Patch TinyXML2"
if git apply "${WORKSPACE}/build_qnx/qnx_patches/tinyxml2_qnx.patch"; then
  echo "Patch TinyXML2 done"
else
  echo "[WARN] tinyxml2_qnx.patch did not apply cleanly; continuing with upstream tinyxml2."
fi

cd "${WORKSPACE}"
git clone https://github.com/eProsima/foonathan_memory_vendor.git -b "v${FOONATHAN_MEMORY_TAG}"
git clone https://github.com/google/googletest.git
cd googletest
git checkout "v${GOOGLETEST_TAG}"
git apply "${WORKSPACE}/build_qnx/qnx_patches/googletest_qnx.patch"

cd "${WORKSPACE}/build_qnx/${QNX_BUILD_DIR_SUFFIX}"
make INSTALL_ROOT_nto="${INSTALL_PREFIX}"
make install INSTALL_ROOT_nto="${INSTALL_PREFIX}"

echo "[OK] Installed Fast-DDS v${FAST_DDS_TAG} (QNX) to ${INSTALL_PREFIX}"
