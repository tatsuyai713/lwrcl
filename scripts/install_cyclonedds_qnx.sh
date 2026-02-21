#!/usr/bin/env bash
set -euo pipefail

CYCLONEDDS_TAG="0.10.5"
CYCLONEDDS_CXX_TAG="0.10.5"
INSTALL_PREFIX="/opt/qnx/cyclonedds"
ICEORYX_PREFIX="/opt/qnx/iceoryx"
ARCH="aarch64le"
BUILD_DIR="${HOME}/build-cyclonedds-qnx"
TOOLCHAIN_FILE=""
ENABLE_SHM="AUTO" # AUTO | ON | OFF
BUILD_HOST_IDLC="ON"
PATCH_THREADS_FOR_QNX="ON"
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"
QNX_ENV_FILE=""
QNX_PATH=""

if [[ -d "${HOME}/qnx803" ]]; then
  QNX_PATH="qnx803"
elif [[ -d "${HOME}/qnx800" ]]; then
  QNX_PATH="qnx800"
else
  QNX_PATH="qnx803"
fi

_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/qnx_toolchain.cmake"

log_info() {
  echo "[INFO] $*"
}

log_warn() {
  echo "[WARN] $*" >&2
}

log_error() {
  echo "[ERROR] $*" >&2
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --prefix <path>              Install prefix (default: ${INSTALL_PREFIX})
  --tag <version>              CycloneDDS C tag (default: ${CYCLONEDDS_TAG})
  --cxx-tag <version>          CycloneDDS C++ tag (default: ${CYCLONEDDS_CXX_TAG})
  --arch <aarch64le|x86_64>    QNX target arch (default: ${ARCH})
  --iceoryx-prefix <path>      QNX iceoryx prefix (default: ${ICEORYX_PREFIX})
  --build-dir <path>           Build directory (default: ${BUILD_DIR})
  --toolchain-file <path>      QNX toolchain file (default: ${DEFAULT_TOOLCHAIN_FILE})
  --qnx-path <name>            QNX SDP directory name in HOME (default: auto)
  --qnx-env <file>             qnxsdp-env.sh path (overrides --qnx-path)
  --jobs <N>                   Parallel jobs (default: auto)
  --enable-shm                 Force CycloneDDS SHM ON (requires QNX iceoryx)
  --disable-shm                Force CycloneDDS SHM OFF
  --without-host-idlc          Do not build/install host idlc tool
  --without-qnx-threads-patch  Disable QNX Threads patch in ddsrt CMake
  --skip-system-deps           Skip apt dependency installation
  --force                      Force reinstall
  --help                       Show this help
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
      CYCLONEDDS_TAG="$2"
      shift 2
      ;;
    --cxx-tag)
      CYCLONEDDS_CXX_TAG="$2"
      shift 2
      ;;
    --arch)
      ARCH="$2"
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
    --enable-shm)
      ENABLE_SHM="ON"
      shift
      ;;
    --disable-shm)
      ENABLE_SHM="OFF"
      shift
      ;;
    --without-host-idlc)
      BUILD_HOST_IDLC="OFF"
      shift
      ;;
    --without-qnx-threads-patch)
      PATCH_THREADS_FOR_QNX="OFF"
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
    --help)
      usage
      ;;
    *)
      log_error "Unknown argument: $1"
      exit 1
      ;;
  esac
done

if [[ -z "${TOOLCHAIN_FILE}" ]]; then
  TOOLCHAIN_FILE="${DEFAULT_TOOLCHAIN_FILE}"
fi

if [[ "${FORCE_REINSTALL}" != "ON" ]] \
  && [[ -f "${INSTALL_PREFIX}/include/ddsc/dds.h" ]] \
  && [[ -f "${INSTALL_PREFIX}/include/ddscxx/dds/dds.hpp" ]] \
  && [[ -x "${INSTALL_PREFIX}/bin/idlc" ]]; then
  log_info "cyclonedds(+cxx,+idlc) already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
  exit 0
fi

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
  else
    log_error "Please run as root or install sudo."
    exit 1
  fi
fi

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]] && command -v apt-get >/dev/null 2>&1; then
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y --no-install-recommends \
    libssl-dev bison flex git cmake make gcc g++ pkg-config
fi

if [[ -z "${QNX_HOST:-}" || -z "${QNX_TARGET:-}" ]]; then
  if [[ -z "${QNX_ENV_FILE}" ]]; then
    QNX_ENV_FILE="${HOME}/${QNX_PATH}/qnxsdp-env.sh"
  fi
  if [[ ! -f "${QNX_ENV_FILE}" ]]; then
    log_error "QNX SDP environment script not found: ${QNX_ENV_FILE}"
    exit 1
  fi
  # shellcheck disable=SC1090
  source "${QNX_ENV_FILE}"
fi

if [[ -z "${QNX_HOST:-}" || -z "${QNX_TARGET:-}" ]]; then
  log_error "QNX_HOST/QNX_TARGET are not set. Source qnxsdp-env.sh first."
  exit 1
fi

if ! command -v qcc >/dev/null 2>&1 || ! command -v q++ >/dev/null 2>&1; then
  log_error "qcc/q++ not found in PATH after sourcing QNX environment."
  exit 1
fi

if [[ ! -f "${TOOLCHAIN_FILE}" ]]; then
  log_error "Toolchain file not found: ${TOOLCHAIN_FILE}"
  exit 1
fi

if [[ "${ENABLE_SHM}" == "AUTO" ]]; then
  if [[ -f "${ICEORYX_PREFIX}/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]]; then
    ENABLE_SHM="ON"
  else
    ENABLE_SHM="OFF"
  fi
fi

if [[ "${ENABLE_SHM}" == "ON" ]] && [[ ! -f "${ICEORYX_PREFIX}/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]]; then
  log_error "ENABLE_SHM=ON but iceoryx is not found at ${ICEORYX_PREFIX}."
  log_error "Install QNX iceoryx first (e.g. scripts/install_iceoryx_qnx.sh) or pass --disable-shm."
  exit 1
fi

SOURCE_DIR="${BUILD_DIR}/cyclonedds"
BUILD_QNX_DIR="${BUILD_DIR}/cyclonedds-build-qnx"
SOURCE_CXX_DIR="${BUILD_DIR}/cyclonedds-cxx"
BUILD_CXX_DIR="${BUILD_DIR}/cyclonedds-cxx-build-qnx"
BUILD_HOST_IDLC_DIR="${BUILD_DIR}/cyclonedds-host-idlc"
HOST_TOOLS_PREFIX="${INSTALL_PREFIX}/host-tools"

${SUDO} mkdir -p "${INSTALL_PREFIX}"
${SUDO} chmod 777 -R "${INSTALL_PREFIX}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

log_info "Build CycloneDDS for QNX"
log_info "  tags: C=${CYCLONEDDS_TAG} C++=${CYCLONEDDS_CXX_TAG} arch=${ARCH}"
log_info "  prefix=${INSTALL_PREFIX} shm=${ENABLE_SHM}"

git clone --depth 1 --branch "${CYCLONEDDS_TAG}" \
  https://github.com/eclipse-cyclonedds/cyclonedds.git "${SOURCE_DIR}"

if [[ "${PATCH_THREADS_FOR_QNX}" == "ON" ]]; then
  DDSRT_CMAKE="${SOURCE_DIR}/src/ddsrt/CMakeLists.txt"
  if [[ -f "${DDSRT_CMAKE}" ]]; then
    sed -i.bak -E \
      's/^([[:space:]]*)(find_package\(Threads REQUIRED\)|target_link_libraries\(ddsrt INTERFACE Threads::Threads\))/\1# \2/' \
      "${DDSRT_CMAKE}"
    rm -f "${DDSRT_CMAKE}.bak"
  fi
fi

OPENSSL_ROOT_DIR="${QNX_TARGET}/${ARCH}/usr"
if [[ ! -d "${OPENSSL_ROOT_DIR}" ]]; then
  OPENSSL_ROOT_DIR="${QNX_TARGET}/usr"
fi

CORE_EXTRA_ARGS=()
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  CORE_EXTRA_ARGS+=(
    -DCMAKE_PREFIX_PATH="${ICEORYX_PREFIX};${ICEORYX_PREFIX}/lib/cmake"
  )
fi

cmake -S "${SOURCE_DIR}" -B "${BUILD_QNX_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DQNX_ARCH="${ARCH}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_DDSPERF=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_IDLC=OFF \
  -DENABLE_LTO=OFF \
  -DENABLE_SOURCE_SPECIFIC_MULTICAST=OFF \
  -DENABLE_SHM="${ENABLE_SHM}" \
  -DOPENSSL_ROOT_DIR="${OPENSSL_ROOT_DIR}" \
  -DOPENSSL_INCLUDE_DIR="${QNX_TARGET}/usr/include" \
  "${CORE_EXTRA_ARGS[@]}"

cmake --build "${BUILD_QNX_DIR}" -j"${JOBS}"
cmake --install "${BUILD_QNX_DIR}"

git clone --depth 1 --branch "${CYCLONEDDS_CXX_TAG}" \
  https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git "${SOURCE_CXX_DIR}"

CXX_PREFIX_PATH="${INSTALL_PREFIX}"
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  CXX_PREFIX_PATH="${INSTALL_PREFIX};${ICEORYX_PREFIX};${ICEORYX_PREFIX}/lib/cmake"
fi

cmake -S "${SOURCE_CXX_DIR}" -B "${BUILD_CXX_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DQNX_ARCH="${ARCH}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${CXX_PREFIX_PATH}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_IDLCPP_GENERATOR=ON \
  -DCYCLONEDDS_CXX_BLD_IDLCPP=ON \
  -DDDSCXX_NO_STD_OPTIONAL=ON \
  -DENABLE_TOPIC_DISCOVERY=OFF

cmake --build "${BUILD_CXX_DIR}" -j"${JOBS}"
cmake --install "${BUILD_CXX_DIR}"

if [[ "${BUILD_HOST_IDLC}" == "ON" ]]; then
  log_info "Build host idlc and install into ${INSTALL_PREFIX}/bin/idlc"

  cmake -S "${SOURCE_DIR}" -B "${BUILD_HOST_IDLC_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${HOST_TOOLS_PREFIX}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DBUILD_DDSPERF=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_IDLC=ON \
    -DENABLE_SHM=OFF \
    -DENABLE_SOURCE_SPECIFIC_MULTICAST=OFF

  cmake --build "${BUILD_HOST_IDLC_DIR}" --target idlc -j"${JOBS}"
  cmake --install "${BUILD_HOST_IDLC_DIR}"

  ${SUDO} mkdir -p "${INSTALL_PREFIX}/bin"
  cat <<'IDLC_WRAPPER' | ${SUDO} tee "${INSTALL_PREFIX}/bin/idlc" >/dev/null
#!/usr/bin/env bash
set -euo pipefail
THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_TOOLS_DIR="$(cd "${THIS_DIR}/.." && pwd)/host-tools"
export LD_LIBRARY_PATH="${HOST_TOOLS_DIR}/lib:${HOST_TOOLS_DIR}/lib64:${LD_LIBRARY_PATH:-}"
exec "${HOST_TOOLS_DIR}/bin/idlc" "$@"
IDLC_WRAPPER
  ${SUDO} chmod +x "${INSTALL_PREFIX}/bin/idlc"
fi

if [[ "${ENABLE_SHM}" == "ON" ]]; then
  ${SUDO} mkdir -p "${INSTALL_PREFIX}/etc"
  cat <<'CYCLONEDDS_XML' | ${SUDO} tee "${INSTALL_PREFIX}/etc/cyclonedds-lwrcl.xml" >/dev/null
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <SharedMemory>
      <Enable>true</Enable>
      <LogLevel>warn</LogLevel>
    </SharedMemory>
  </Domain>
</CycloneDDS>
CYCLONEDDS_XML
fi

log_info "Installed CycloneDDS ${CYCLONEDDS_TAG} (+cxx ${CYCLONEDDS_CXX_TAG}) to ${INSTALL_PREFIX}"
log_info "  SHM=${ENABLE_SHM}  host_idlc=${BUILD_HOST_IDLC}"
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  log_info "CycloneDDS SHM config: ${INSTALL_PREFIX}/etc/cyclonedds-lwrcl.xml"
fi
