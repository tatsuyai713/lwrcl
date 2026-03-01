#!/usr/bin/env bash
set -euo pipefail

# QNX CycloneDDS cross-build script (C library + C++ binding + host idlc).
#
# The Threads patch comments out find_package(Threads) and
# target_link_libraries(ddsrt INTERFACE Threads::Threads) which are
# problematic under QNX cross-compilation.
#
# When SHM is AUTO (default), iceoryx is auto-installed if not found.
#
# Usage:
#   ./scripts/install_cyclonedds_qnx.sh install --arch aarch64le
#   ./scripts/install_cyclonedds_qnx.sh install --arch aarch64le --enable-shm
#   ./scripts/install_cyclonedds_qnx.sh clean

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
DEFAULT_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/qnx_toolchain.cmake"

ACTION="build"
if [[ $# -gt 0 ]] && [[ "$1" != --* ]]; then
  ACTION="$1"
  shift
fi

CYCLONEDDS_TAG="0.10.5"
CYCLONEDDS_CXX_TAG="0.10.5"
ARCH="aarch64le"
OUT_ROOT="/opt/qnx"
QNX_PATH="qnx800"
QNX_ENV_FILE=""
TOOLCHAIN_FILE="${DEFAULT_TOOLCHAIN_FILE}"
ENABLE_SHM="AUTO"   # AUTO | ON | OFF
SKIP_ICEORYX="OFF"
BUILD_HOST_IDLC="ON"
PATCH_THREADS_FOR_QNX="ON"
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

INSTALL_PREFIX="${OUT_ROOT}/cyclonedds"
ICEORYX_PREFIX="${OUT_ROOT}/iceoryx"
BUILD_DIR="${HOME}/build-cyclonedds-qnx"

_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1

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
  --prefix <path>              Install prefix (default: ${INSTALL_PREFIX})
  --tag <version>              CycloneDDS C tag (default: ${CYCLONEDDS_TAG})
  --cxx-tag <version>          CycloneDDS C++ tag (default: ${CYCLONEDDS_CXX_TAG})
  --arch <aarch64le|x86_64>    QNX target arch (default: ${ARCH})
  --iceoryx-prefix <path>      QNX iceoryx prefix (default: ${ICEORYX_PREFIX})
  --build-dir <path>           Build directory (default: ${BUILD_DIR})
  --toolchain-file <path>      QNX toolchain file (default: ${DEFAULT_TOOLCHAIN_FILE})
  --qnx-path <name>            QNX SDP directory name in HOME (default: ${QNX_PATH})
  --qnx-env <file>             qnxsdp-env.sh path (overrides --qnx-path)
  --jobs <N>                   Parallel jobs (default: auto)
  --enable-shm                 Force CycloneDDS SHM ON (requires QNX iceoryx)
  --disable-shm                Force CycloneDDS SHM OFF
  --skip-iceoryx               Skip auto-installation of iceoryx
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
    --skip-iceoryx)
      SKIP_ICEORYX="ON"
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

if [[ "${ACTION}" == "clean" ]]; then
  rm -rf "${BUILD_DIR}"
  log_info "Cleaned ${BUILD_DIR}"
  exit 0
fi

if [[ "${FORCE_REINSTALL}" != "ON" ]] \
  && [[ -f "${INSTALL_PREFIX}/include/ddsc/dds.h" ]] \
  && [[ -f "${INSTALL_PREFIX}/include/ddscxx/dds/dds.hpp" ]] \
  && [[ -x "${INSTALL_PREFIX}/bin/idlc" ]]; then
  log_info "cyclonedds(+cxx,+idlc) already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
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
  log_info "Installing build dependencies (libssl-dev, bison, flex, git, cmake 3.x, make, gcc, g++)..."
  ${SUDO} apt-get update -q
  _cmake3=$(apt-cache policy cmake 2>/dev/null | grep -oP '\s+\K3\.[0-9]+\.[0-9]+[^\s]*' | head -1)
  if [[ -n "${_cmake3}" ]]; then
    ${SUDO} apt-get install -y --no-install-recommends \
      libssl-dev bison flex git "cmake=${_cmake3}" "cmake-data=${_cmake3}" make gcc g++ pkg-config
  else
    ${SUDO} apt-get install -y --no-install-recommends \
      libssl-dev bison flex git cmake make gcc g++ pkg-config
  fi
fi

# ---------------------------------------------------------------------------
# Resolve SHM mode and install iceoryx for QNX if needed
# ---------------------------------------------------------------------------
if [[ "${ENABLE_SHM}" == "AUTO" ]]; then
  if [[ -f "${ICEORYX_PREFIX}/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]]; then
    log_info "iceoryx found at ${ICEORYX_PREFIX} – enabling SHM."
    ENABLE_SHM="ON"
  elif [[ "${SKIP_ICEORYX}" == "ON" ]]; then
    log_info "iceoryx not found and --skip-iceoryx set – building without SHM."
    ENABLE_SHM="OFF"
  else
    log_info "iceoryx not found – installing iceoryx for QNX SHM support."
    bash "${SCRIPT_DIR}/install_iceoryx_qnx.sh" install \
      --arch "${ARCH}" \
      --prefix "${ICEORYX_PREFIX}" \
      --toolchain-file "${TOOLCHAIN_FILE}"
    ENABLE_SHM="ON"
  fi
fi

if [[ "${ENABLE_SHM}" == "ON" ]] && [[ ! -f "${ICEORYX_PREFIX}/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]]; then
  if [[ "${SKIP_ICEORYX}" == "ON" ]]; then
    log_error "--enable-shm requested but iceoryx not found at ${ICEORYX_PREFIX} and --skip-iceoryx set."
    exit 1
  fi
  log_info "--enable-shm set but iceoryx not found – installing iceoryx for QNX."
  bash "${SCRIPT_DIR}/install_iceoryx_qnx.sh" install \
    --arch "${ARCH}" \
    --prefix "${ICEORYX_PREFIX}" \
    --toolchain-file "${TOOLCHAIN_FILE}"
fi

mkdir -p "${INSTALL_PREFIX}"
chmod 777 "${INSTALL_PREFIX}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

SOURCE_DIR="${BUILD_DIR}/cyclonedds"
BUILD_QNX_DIR="${BUILD_DIR}/cyclonedds-build-qnx"
SOURCE_CXX_DIR="${BUILD_DIR}/cyclonedds-cxx"
BUILD_CXX_DIR="${BUILD_DIR}/cyclonedds-cxx-build-qnx"
BUILD_HOST_IDLC_DIR="${BUILD_DIR}/cyclonedds-host-idlc"
HOST_TOOLS_PREFIX="${INSTALL_PREFIX}/host-tools"

log_info "Build CycloneDDS for QNX"
log_info "  tags: C=${CYCLONEDDS_TAG} C++=${CYCLONEDDS_CXX_TAG} arch=${ARCH}"
log_info "  prefix=${INSTALL_PREFIX} shm=${ENABLE_SHM}"

# --- CycloneDDS C library ---
log_info "Cloning CycloneDDS C ${CYCLONEDDS_TAG}..."
git clone --depth 1 --branch "${CYCLONEDDS_TAG}" \
  https://github.com/eclipse-cyclonedds/cyclonedds.git "${SOURCE_DIR}"

# Patch: comment out Threads that break QNX cross-compilation
if [[ "${PATCH_THREADS_FOR_QNX}" == "ON" ]]; then
  DDSRT_CMAKE="${SOURCE_DIR}/src/ddsrt/CMakeLists.txt"
  if [[ -f "${DDSRT_CMAKE}" ]]; then
    if grep -qE '^\s*find_package\(Threads REQUIRED\)' "${DDSRT_CMAKE}"; then
      log_info "Patching ddsrt CMakeLists.txt: commenting out Threads"
      sed -i.bak -E \
        's/^([[:space:]]*)(find_package\(Threads REQUIRED\)|target_link_libraries\(ddsrt INTERFACE Threads::Threads\))/\1# \2/' \
        "${DDSRT_CMAKE}"
      rm -f "${DDSRT_CMAKE}.bak"
    fi
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

log_info "Configuring cmake build (CycloneDDS C)..."
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

log_info "Building CycloneDDS C (jobs=${JOBS})..."
cmake --build "${BUILD_QNX_DIR}" -j"${JOBS}"
# C library must always be installed so C++ binding can find it
log_info "Installing CycloneDDS C to ${INSTALL_PREFIX}..."
cmake --install "${BUILD_QNX_DIR}"

# --- CycloneDDS C++ binding ---
log_info "Cloning CycloneDDS C++ ${CYCLONEDDS_CXX_TAG}..."
git clone --depth 1 --branch "${CYCLONEDDS_CXX_TAG}" \
  https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git "${SOURCE_CXX_DIR}"

CXX_PREFIX_PATH="${INSTALL_PREFIX}"
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  CXX_PREFIX_PATH="${INSTALL_PREFIX};${ICEORYX_PREFIX};${ICEORYX_PREFIX}/lib/cmake"
fi

log_info "Configuring cmake build (CycloneDDS C++)..."
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

log_info "Building CycloneDDS C++ (jobs=${JOBS})..."
cmake --build "${BUILD_CXX_DIR}" -j"${JOBS}"

if [[ "${ACTION}" == "install" ]]; then
  log_info "Installing CycloneDDS C++ to ${INSTALL_PREFIX}..."
  cmake --install "${BUILD_CXX_DIR}"
fi

# --- Build host idlc tool (runs on build host, not QNX target) ---
if [[ "${BUILD_HOST_IDLC}" == "ON" ]]; then
  log_info "Build host idlc and install into ${INSTALL_PREFIX}/bin/idlc"

  log_info "Configuring cmake build (host idlc)..."
  cmake -S "${SOURCE_DIR}" -B "${BUILD_HOST_IDLC_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${HOST_TOOLS_PREFIX}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DBUILD_DDSPERF=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_IDLC=ON \
    -DENABLE_LTO=OFF \
    -DENABLE_SSL=OFF \
    -DCMAKE_C_FLAGS="-fno-lto" \
    -DCMAKE_SHARED_LINKER_FLAGS="-fno-lto" \
    -DCMAKE_EXE_LINKER_FLAGS="-fno-lto" \
    -DENABLE_SHM=OFF \
    -DENABLE_SOURCE_SPECIFIC_MULTICAST=OFF

  log_info "Building host idlc..."
  cmake --build "${BUILD_HOST_IDLC_DIR}" --target idlc -j"${JOBS}"
  log_info "Installing host idlc to ${HOST_TOOLS_PREFIX}..."
  cmake --install "${BUILD_HOST_IDLC_DIR}"

  mkdir -p "${INSTALL_PREFIX}/bin"
  cat > "${INSTALL_PREFIX}/bin/idlc" <<'IDLC_WRAPPER'
#!/usr/bin/env bash
set -euo pipefail
THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_TOOLS_DIR="$(cd "${THIS_DIR}/.." && pwd)/host-tools"
export LD_LIBRARY_PATH="${HOST_TOOLS_DIR}/lib:${HOST_TOOLS_DIR}/lib64:${LD_LIBRARY_PATH:-}"
exec "${HOST_TOOLS_DIR}/bin/idlc" "$@"
IDLC_WRAPPER
  chmod +x "${INSTALL_PREFIX}/bin/idlc"
fi

# --- Generate SHM config XML ---
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  mkdir -p "${INSTALL_PREFIX}/etc"
  cat > "${INSTALL_PREFIX}/etc/cyclonedds-lwrcl.xml" <<'CYCLONEDDS_XML'
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

log_info "CycloneDDS QNX build complete (action=${ACTION})"
log_info "  install_prefix=${INSTALL_PREFIX}"
log_info "  SHM=${ENABLE_SHM}  host_idlc=${BUILD_HOST_IDLC}"
if [[ "${ENABLE_SHM}" == "ON" ]]; then
  log_info "CycloneDDS SHM config: ${INSTALL_PREFIX}/etc/cyclonedds-lwrcl.xml"
fi
