#!/usr/bin/env bash
set -euo pipefail

CYCLONEDDS_TAG="0.10.5"
CYCLONEDDS_CXX_TAG="0.10.5"
INSTALL_PREFIX="/opt/cyclonedds"
ICEORYX_PREFIX="/opt/iceoryx"
BUILD_DIR="${HOME}/build-cyclonedds"
_nproc="$(nproc 2>/dev/null || echo 4)"
# Cap jobs by available memory (~1.5 GB per compiler job) to avoid OOM kills
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1
ENABLE_SHM="AUTO" # AUTO | ON | OFF
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --tag)
      CYCLONEDDS_TAG="$2"
      shift 2
      ;;
    --cxx-tag)
      CYCLONEDDS_CXX_TAG="$2"
      shift 2
      ;;
    --iceoryx-prefix)
      ICEORYX_PREFIX="$2"
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
    --enable-shm)
      ENABLE_SHM="ON"
      shift
      ;;
    --disable-shm)
      ENABLE_SHM="OFF"
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
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ -x "${INSTALL_PREFIX}/bin/idlc" ]] && [[ -f "${INSTALL_PREFIX}/include/ddscxx/dds/dds.hpp" ]]; then
  echo "[INFO] cyclonedds(+cxx) already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
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
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y --no-install-recommends libssl-dev bison flex
fi

if [[ "${ENABLE_SHM}" == "AUTO" ]]; then
  if [[ -d "${ICEORYX_PREFIX}/lib/cmake/iceoryx_hoofs" ]]; then
    ENABLE_SHM="ON"
  else
    ENABLE_SHM="OFF"
  fi
fi

${SUDO} mkdir -p "${INSTALL_PREFIX}"

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# Build CycloneDDS core (includes idlc)
git clone --depth 1 --branch "${CYCLONEDDS_TAG}" https://github.com/eclipse-cyclonedds/cyclonedds.git "${BUILD_DIR}/cyclonedds"

core_extra_args=()
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  core_extra_args+=( -DENABLE_SHM=ON -DCMAKE_PREFIX_PATH="${ICEORYX_PREFIX}" )
else
  core_extra_args+=( -DENABLE_SHM=OFF )
fi

cmake -S "${BUILD_DIR}/cyclonedds" -B "${BUILD_DIR}/cyclonedds-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DENABLE_LTO=OFF \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_IDLC=ON \
  -DENABLE_SOURCE_SPECIFIC_MULTICAST=ON \
  "${core_extra_args[@]}"

cmake --build "${BUILD_DIR}/cyclonedds-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/cyclonedds-build"

# Build CycloneDDS-CXX
git clone --depth 1 --branch "${CYCLONEDDS_CXX_TAG}" https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git "${BUILD_DIR}/cyclonedds-cxx"

cxx_prefix_path="${INSTALL_PREFIX}"
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  cxx_prefix_path="${INSTALL_PREFIX};${ICEORYX_PREFIX}"
fi

cmake -S "${BUILD_DIR}/cyclonedds-cxx" -B "${BUILD_DIR}/cyclonedds-cxx-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${cxx_prefix_path}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_IDLCPP_GENERATOR=ON \
  -DCYCLONEDDS_CXX_BLD_IDLCPP=ON \
  -DDDSCXX_NO_STD_OPTIONAL=ON

cmake --build "${BUILD_DIR}/cyclonedds-cxx-build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/cyclonedds-cxx-build"

echo "[OK] Installed cyclonedds ${CYCLONEDDS_TAG} (+cxx ${CYCLONEDDS_CXX_TAG}) to ${INSTALL_PREFIX} (SHM=${ENABLE_SHM})"
