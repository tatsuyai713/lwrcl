#!/usr/bin/env bash
set -euo pipefail

# QNX iceoryx cross-build script.
#
# Usage:
#   ./qnx/scripts/build_iceoryx_qnx.sh install --arch aarch64le
#   ./qnx/scripts/build_iceoryx_qnx.sh clean

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

ACTION="build"
if [[ $# -gt 0 ]] && [[ "$1" != --* ]]; then
  ACTION="$1"
  shift
fi

ARCH="$(qnx_default_arch)"
OUT_ROOT="$(qnx_default_out_root)"
JOBS="$(qnx_default_jobs)"
ICEORYX_TAG="v2.0.6"

INSTALL_PREFIX="${OUT_ROOT}/iceoryx"
WORK_DIR="${AUTOSAR_QNX_WORK_ROOT:-${REPO_ROOT}/out/qnx/work}/iceoryx/${ARCH}"
THIRD_PARTY_PREFIX="${OUT_ROOT}/third_party"
TOOLCHAIN_FILE="$(qnx_resolve_toolchain_file)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arch)
      ARCH="$2"
      shift 2
      ;;
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --third-party-prefix)
      THIRD_PARTY_PREFIX="$2"
      shift 2
      ;;
    --toolchain-file)
      TOOLCHAIN_FILE="$2"
      shift 2
      ;;
    --jobs)
      JOBS="$2"
      shift 2
      ;;
    --tag)
      ICEORYX_TAG="$2"
      shift 2
      ;;
    *)
      qnx_error "Unknown argument: $1"
      exit 1
      ;;
  esac
done

qnx_setup_env
qnx_require_cmd "cmake"

if [[ "${ACTION}" == "clean" ]]; then
  rm -rf "${WORK_DIR}"
  qnx_info "Cleaned ${WORK_DIR}"
  exit 0
fi

SOURCE_DIR="${WORK_DIR}/iceoryx"
BUILD_DIR="${WORK_DIR}/build-qnx"

mkdir -p "${WORK_DIR}"
mkdir -p "${INSTALL_PREFIX}"
chmod 777 "${INSTALL_PREFIX}"

qnx_info "Build iceoryx for QNX"
qnx_info "  tag=${ICEORYX_TAG} arch=${ARCH}"
qnx_info "  prefix=${INSTALL_PREFIX} work_dir=${WORK_DIR}"

qnx_clone_or_update "https://github.com/eclipse-iceoryx/iceoryx.git" "${ICEORYX_TAG}" "${SOURCE_DIR}"

CMAKE_PREFIX_PATH_VALUE="${THIRD_PARTY_PREFIX};${THIRD_PARTY_PREFIX}/lib/cmake;${THIRD_PARTY_PREFIX}/lib64/cmake"

cmake -S "${SOURCE_DIR}/iceoryx_meta" -B "${BUILD_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DQNX_ARCH="${ARCH}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH_VALUE}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TEST=OFF \
  -DINTROSPECTION=OFF \
  -DBUILD_DOC=OFF \
  -DEXAMPLES=OFF

cmake --build "${BUILD_DIR}" -j"${JOBS}"

if [[ "${ACTION}" == "install" ]]; then
  cmake --install "${BUILD_DIR}"
fi

qnx_info "iceoryx QNX build complete (action=${ACTION})"
qnx_info "  install_prefix=${INSTALL_PREFIX}"
