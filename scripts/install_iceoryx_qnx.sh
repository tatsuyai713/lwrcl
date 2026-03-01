#!/usr/bin/env bash
set -euo pipefail

# QNX iceoryx cross-build script.
#
# Usage:
#   ./scripts/install_iceoryx_qnx.sh install --arch aarch64le
#   ./scripts/install_iceoryx_qnx.sh clean

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
DEFAULT_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/qnx_toolchain.cmake"

ACTION="build"
if [[ $# -gt 0 ]] && [[ "$1" != --* ]]; then
  ACTION="$1"
  shift
fi

ARCH="aarch64le"
OUT_ROOT="/opt/qnx"
ICEORYX_TAG="v2.0.6"
QNX_PATH="qnx800"
QNX_ENV_FILE=""
TOOLCHAIN_FILE="${DEFAULT_TOOLCHAIN_FILE}"

INSTALL_PREFIX="${OUT_ROOT}/iceoryx"
BUILD_DIR="${HOME}/build-iceoryx-qnx"

_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1

SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

log_info()  { echo "[INFO] $*"; }
log_warn()  { echo "[WARN] $*" >&2; }
log_error() { echo "[ERROR] $*" >&2; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [ACTION] [OPTIONS]

Actions:
  build    Build but do not install (default)
  install  Build and install
  clean    Remove build directory

Options:
  --prefix <path>         Install prefix (default: ${INSTALL_PREFIX})
  --arch <aarch64le|x86_64>  QNX target arch (default: ${ARCH})
  --tag <version>         iceoryx tag (default: ${ICEORYX_TAG})
  --build-dir <path>      Build directory (default: ${BUILD_DIR})
  --toolchain-file <path> QNX toolchain file (default: ${DEFAULT_TOOLCHAIN_FILE})
  --qnx-path <name>       QNX SDP directory name in HOME (default: ${QNX_PATH})
  --qnx-env <file>        qnxsdp-env.sh path (overrides --qnx-path)
  --jobs <N>              Parallel jobs (default: auto)
  --skip-system-deps      Skip apt dependency installation
  --force                 Force reinstall
  --help                  Show this help
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --arch)
      ARCH="$2"
      shift 2
      ;;
    --tag)
      ICEORYX_TAG="$2"
      shift 2
      ;;
    --build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    --toolchain-file)
      TOOLCHAIN_FILE="$2"
      shift 2
      ;;
    --qnx-path)
      QNX_PATH="$2"
      shift 2
      ;;
    --qnx-env)
      QNX_ENV_FILE="$2"
      shift 2
      ;;
    --jobs)
      JOBS="$2"
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
      log_error "Unknown argument: $1"
      exit 1
      ;;
  esac
done

if [[ "${ACTION}" == "clean" ]]; then
  rm -rf "${BUILD_DIR}"
  log_info "Cleaned ${BUILD_DIR}"
  exit 0
fi

if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ -f "${INSTALL_PREFIX}/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]]; then
  log_info "iceoryx already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
  exit 0
fi

# ---------------------------------------------------------------------------
# Setup QNX environment
# ---------------------------------------------------------------------------
if [[ -z "${QNX_HOST:-}" || -z "${QNX_TARGET:-}" ]]; then
  if [[ -z "${QNX_ENV_FILE}" ]]; then
    QNX_ENV_FILE="${HOME}/${QNX_PATH}/qnxsdp-env.sh"
  fi
  if [[ ! -f "${QNX_ENV_FILE}" ]]; then
    log_error "QNX SDP environment script not found: ${QNX_ENV_FILE}"
    exit 1
  fi
  # shellcheck disable=SC1090
  _SAVED_SCRIPT_DIR="${SCRIPT_DIR}"; source "${QNX_ENV_FILE}"; SCRIPT_DIR="${_SAVED_SCRIPT_DIR}"
fi

if [[ -z "${QNX_HOST:-}" || -z "${QNX_TARGET:-}" ]]; then
  log_error "QNX_HOST/QNX_TARGET are not set. Source qnxsdp-env.sh first."
  exit 1
fi

export PATH="${QNX_HOST}/usr/bin:${PATH}"

if ! command -v qcc >/dev/null 2>&1 || ! command -v q++ >/dev/null 2>&1; then
  log_error "qcc/q++ not found in PATH after sourcing QNX environment."
  exit 1
fi

if ! command -v cmake >/dev/null 2>&1; then
  log_error "cmake not found. Install cmake first."
  exit 1
fi

if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
  log_error "Toolchain file not found: ${TOOLCHAIN_FILE}"
  exit 1
fi

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]] && command -v apt-get >/dev/null 2>&1; then
  SUDO=""
  if [[ "${EUID}" -ne 0 ]]; then
    SUDO="sudo"
  fi
  log_info "Installing build dependencies (git, cmake 3.x, make)..."
  ${SUDO} apt-get update -q
  _cmake3=$(apt-cache policy cmake 2>/dev/null | grep -oP '\s+\K3\.[0-9]+\.[0-9]+[^\s]*' | head -1)
  if [[ -n "${_cmake3}" ]]; then
    ${SUDO} apt-get install -y --no-install-recommends git "cmake=${_cmake3}" "cmake-data=${_cmake3}" make
  else
    ${SUDO} apt-get install -y --no-install-recommends git cmake make
  fi
fi

SOURCE_DIR="${BUILD_DIR}/iceoryx"
BUILD_QNX_DIR="${BUILD_DIR}/build-qnx"

mkdir -p "${INSTALL_PREFIX}"
chmod 777 "${INSTALL_PREFIX}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

log_info "Build iceoryx for QNX"
log_info "  tag=${ICEORYX_TAG} arch=${ARCH}"
log_info "  prefix=${INSTALL_PREFIX}"

log_info "Cloning iceoryx ${ICEORYX_TAG}..."
git clone --depth 1 --branch "${ICEORYX_TAG}" https://github.com/eclipse-iceoryx/iceoryx.git "${SOURCE_DIR}"

log_info "Configuring cmake build..."
cmake -S "${SOURCE_DIR}/iceoryx_meta" -B "${BUILD_QNX_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DQNX_ARCH="${ARCH}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TEST=OFF \
  -DINTROSPECTION=OFF \
  -DBUILD_DOC=OFF \
  -DEXAMPLES=OFF

log_info "Building iceoryx (jobs=${JOBS})..."
cmake --build "${BUILD_QNX_DIR}" -j"${JOBS}"

if [[ "${ACTION}" == "install" ]]; then
  log_info "Installing iceoryx to ${INSTALL_PREFIX}..."
  cmake --install "${BUILD_QNX_DIR}"
fi

log_info "iceoryx QNX build complete (action=${ACTION})"
log_info "  install_prefix=${INSTALL_PREFIX}"
