#!/usr/bin/env bash
# ===========================================================================
# install_vsomeip_qnx.sh – Cross-build vsomeip + CDR library for QNX
# ===========================================================================
# This script:
#   1. Cross-compiles vsomeip for QNX using qcc/q++
#   2. Extracts the CDR serialization library from cyclonedds-cxx and
#      cross-compiles it for QNX
#   3. Creates stub headers and CMake configs so find_package(CycloneDDS)
#      resolves to the CDR-only library
#
# Requires Boost pre-built for QNX (see build_boost_qnx.sh or
# qnx/scripts/build_boost_qnx.sh).
#
# Usage:
#   ./scripts/install_vsomeip_qnx.sh install --arch aarch64le
#   ./scripts/install_vsomeip_qnx.sh clean
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
# Resolve the repo root relative to this script (scripts/ is one level below root)
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/qnx_toolchain.cmake"

ACTION="build"
if [[ $# -gt 0 ]] && [[ "$1" != --* ]]; then
  ACTION="$1"
  shift
fi

VSOMEIP_TAG="3.4.10"
CYCLONEDDS_CXX_TAG="0.10.5"
ARCH="aarch64le"
OUT_ROOT="/opt/qnx"
QNX_PATH="qnx800"
QNX_ENV_FILE=""
TOOLCHAIN_FILE="${DEFAULT_TOOLCHAIN_FILE}"
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

INSTALL_PREFIX="${OUT_ROOT}/vsomeip"
BOOST_PREFIX="${OUT_ROOT}/third_party"
BUILD_DIR="${HOME}/build-vsomeip-qnx"

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
  --tag <version>              vsomeip version (default: ${VSOMEIP_TAG})
  --cxx-tag <version>          CycloneDDS-CXX version for CDR (default: ${CYCLONEDDS_CXX_TAG})
  --arch <aarch64le|x86_64>    QNX target arch (default: ${ARCH})
  --boost-prefix <path>        Path to QNX-built Boost (default: ${BOOST_PREFIX})
  --build-dir <path>           Build directory (default: ${BUILD_DIR})
  --toolchain-file <path>      QNX toolchain file (default: ${DEFAULT_TOOLCHAIN_FILE})
  --qnx-path <name>            QNX SDP directory name in HOME (default: ${QNX_PATH})
  --qnx-env <file>             qnxsdp-env.sh path (overrides --qnx-path)
  --jobs <N>                   Parallel jobs (default: auto)
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
      VSOMEIP_TAG="$2"
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
    --boost-prefix)
      BOOST_PREFIX="$2"
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

if [[ "${FORCE_REINSTALL}" != "ON" ]] \
  && [[ -f "${INSTALL_PREFIX}/lib/libvsomeip3.so" ]] \
  && [[ -f "${INSTALL_PREFIX}/lib/liblwrcl_cdr.a" ]]; then
  log_info "vsomeip + CDR already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
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

if ! command -v python3 >/dev/null 2>&1; then
  log_error "python3 not found. Install python3 first."
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
  log_info "Installing build dependencies (git, cmake 3.x, make, python3)..."
  ${SUDO} apt-get update -q
  _cmake3=$(apt-cache policy cmake 2>/dev/null | grep -oP '\s+\K3\.[0-9]+\.[0-9]+[^\s]*' | head -1)
  if [[ -n "${_cmake3}" ]]; then
    ${SUDO} apt-get install -y --no-install-recommends git "cmake=${_cmake3}" "cmake-data=${_cmake3}" make python3
  else
    ${SUDO} apt-get install -y --no-install-recommends git cmake make python3
  fi
fi

mkdir -p "${INSTALL_PREFIX}"
chmod 777 "${INSTALL_PREFIX}"
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

BOOST_LIBDIR="${BOOST_PREFIX}/lib"

log_info "Build vsomeip for QNX"
log_info "  tag=${VSOMEIP_TAG} arch=${ARCH}"
log_info "  prefix=${INSTALL_PREFIX} boost=${BOOST_PREFIX}"

# ---------------------------------------------------------------------------
# 1. Clone and cross-build vsomeip
# ---------------------------------------------------------------------------
SOURCE_DIR="${BUILD_DIR}/vsomeip"
BUILD_QNX_DIR="${BUILD_DIR}/build-qnx"

log_info "Cloning vsomeip ${VSOMEIP_TAG}..."
git clone --depth 1 --branch "${VSOMEIP_TAG}" https://github.com/COVESA/vsomeip.git "${SOURCE_DIR}"

# Apply QNX support patch (idempotent) if available
QNX_PATCH_SCRIPT="${REPO_ROOT}/qnx/patches/apply_vsomeip_qnx_support.py"
if [[ -f "${QNX_PATCH_SCRIPT}" ]]; then
  log_info "Applying QNX support patch to vsomeip CMakeLists.txt"
  python3 "${QNX_PATCH_SCRIPT}" "${SOURCE_DIR}/CMakeLists.txt"
fi

log_info "Configuring cmake build (vsomeip)..."
BOOST_VERSION_STRING="$(ls "${BOOST_PREFIX}/include/" 2>/dev/null | grep -oP 'boost-\K[\d_]+' | head -1 || true)"
if [[ -n "${BOOST_VERSION_STRING}" ]]; then
  # Convert "1_86_0" → 108600
  IFS='_' read -r _bmaj _bmin _bpat <<< "${BOOST_VERSION_STRING}"
  BOOST_VERSION_INT=$(( ${_bmaj:-1} * 100000 + ${_bmin:-0} * 100 + ${_bpat:-0} ))
else
  BOOST_VERSION_INT=108600  # default: 1.86.0
fi
log_info "Using Boost version integer: ${BOOST_VERSION_INT}"

cmake -S "${SOURCE_DIR}" -B "${BUILD_QNX_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DQNX_ARCH="${ARCH}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBoost_FOUND=TRUE \
  -DBoost_DIR="${BOOST_PREFIX}" \
  -DBoost_INCLUDE_DIR="${BOOST_PREFIX}/include" \
  -DBoost_LIBRARY_DIR="${BOOST_LIBDIR}" \
  -DBOOST_LIBRARYDIR="${BOOST_LIBDIR}" \
  -DBoost_VERSION="${BOOST_VERSION_INT}" \
  -DCONFIG_DIR="${SOURCE_DIR}" \
  -DBUILD_SHARED_LIBS=OFF \
  -DENABLE_SIGNAL_HANDLING=OFF \
  -DDISABLE_DLT=ON \
  -DCMAKE_EXE_LINKER_FLAGS="-lsocket" \
  -DCMAKE_SHARED_LINKER_FLAGS="-lsocket" \
  -DCMAKE_CXX_FLAGS="-DSA_RESTART=0x0040 -DBOOST_ASIO_DISABLE_STD_EXPERIMENTAL_STRING_VIEW"

log_info "Building vsomeip (jobs=${JOBS})..."
cmake --build "${BUILD_QNX_DIR}" -j"${JOBS}"

if [[ "${ACTION}" == "install" ]]; then
  log_info "Installing vsomeip to ${INSTALL_PREFIX}..."
  cmake --install "${BUILD_QNX_DIR}"
fi

# Install default QNX config
mkdir -p "${INSTALL_PREFIX}/etc"
cat > "${INSTALL_PREFIX}/etc/vsomeip-autosar-qnx.json" <<CFG
{
  "unicast": "127.0.0.1",
  "logging": {
    "level": "info",
    "console": "true",
    "file": { "enable": "false" },
    "dlt": "false"
  },
  "applications": [],
  "services": [],
  "routing": "adaptive_autosar_qnx_routing",
  "service-discovery": {
    "enable": "true",
    "multicast": "224.244.224.245",
    "port": "30490",
    "protocol": "udp",
    "initial_delay_min": "10",
    "initial_delay_max": "100",
    "repetitions_base_delay": "200",
    "repetitions_max": "3",
    "ttl": "3",
    "cyclic_offer_delay": "2000",
    "request_response_delay": "1500"
  }
}
CFG

# ---------------------------------------------------------------------------
# 2. Clone cyclonedds-cxx and extract CDR headers + cross-compile CDR lib
# ---------------------------------------------------------------------------
log_info "Building CDR serialization library for QNX"

CXX_SOURCE_DIR="${BUILD_DIR}/cyclonedds-cxx"
CDR_BUILD_QNX_DIR="${BUILD_DIR}/cdr-build-qnx"

log_info "Cloning cyclonedds-cxx ${CYCLONEDDS_CXX_TAG}..."
git clone --depth 1 --branch "${CYCLONEDDS_CXX_TAG}" \
  https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git "${CXX_SOURCE_DIR}"

CDR_SRC_BASE="src/ddscxx"
CDR_INC="${CXX_SOURCE_DIR}/${CDR_SRC_BASE}/include"
CDR_CPP_ABS="${CXX_SOURCE_DIR}/${CDR_SRC_BASE}/src/org/eclipse/cyclonedds/core/cdr"

# --- 2a. Install CDR headers ---
log_info "Installing CDR headers"
for HDR in \
    "org/eclipse/cyclonedds/core/cdr/cdr_enums.hpp" \
    "org/eclipse/cyclonedds/core/cdr/entity_properties.hpp" \
    "org/eclipse/cyclonedds/core/cdr/cdr_stream.hpp" \
    "org/eclipse/cyclonedds/core/cdr/basic_cdr_ser.hpp" \
    "org/eclipse/cyclonedds/core/cdr/extended_cdr_v1_ser.hpp" \
    "org/eclipse/cyclonedds/core/cdr/extended_cdr_v2_ser.hpp" \
    "org/eclipse/cyclonedds/core/type_helpers.hpp"
do
    HDR_DIR=$(dirname "${HDR}")
    mkdir -p "${INSTALL_PREFIX}/include/${HDR_DIR}"
    cp "${CDR_INC}/${HDR}" "${INSTALL_PREFIX}/include/${HDR}"
done

# Patch cdr_stream.hpp: add missing #include <cstring> for memcpy
CDR_STREAM_HDR="${INSTALL_PREFIX}/include/org/eclipse/cyclonedds/core/cdr/cdr_stream.hpp"
if ! grep -q '<cstring>' "${CDR_STREAM_HDR}" 2>/dev/null; then
    sed -i 's|#include <cassert>|#include <cassert>\n#include <cstring>|' "${CDR_STREAM_HDR}"
    log_info "Patched cdr_stream.hpp: added #include <cstring>"
fi

# --- 2b. Create stub headers for external CycloneDDS dependencies ---
log_info "Creating stub headers"

# dds/ddsrt/endian.h
mkdir -p "${INSTALL_PREFIX}/include/dds/ddsrt"
cat > "${INSTALL_PREFIX}/include/dds/ddsrt/endian.h" <<'ENDIAN_H'
/*
 * Stub endian.h – standalone replacement for CycloneDDS ddsrt/endian.h
 */
#ifndef DDSRT_ENDIAN_H
#define DDSRT_ENDIAN_H

#define DDSRT_LITTLE_ENDIAN 1
#define DDSRT_BIG_ENDIAN    2

#if defined(__BYTE_ORDER__)
  #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    #define DDSRT_ENDIAN DDSRT_LITTLE_ENDIAN
  #else
    #define DDSRT_ENDIAN DDSRT_BIG_ENDIAN
  #endif
#elif defined(_WIN32) || defined(__x86_64__) || defined(__i386__) \
      || defined(__aarch64__) || defined(__arm__)
  #define DDSRT_ENDIAN DDSRT_LITTLE_ENDIAN
#else
  #define DDSRT_ENDIAN DDSRT_BIG_ENDIAN
#endif

#endif /* DDSRT_ENDIAN_H */
ENDIAN_H

# dds/core/macros.hpp
mkdir -p "${INSTALL_PREFIX}/include/dds/core"
cat > "${INSTALL_PREFIX}/include/dds/core/macros.hpp" <<'MACROS_HPP'
/*
 * Stub macros.hpp – standalone replacement for CycloneDDS dds/core/macros.hpp
 */
#ifndef OMG_DDS_CORE_MACROS_HPP_
#define OMG_DDS_CORE_MACROS_HPP_

#define OMG_DDS_API
#define DDSCXX_WARNING_MSVC_OFF(x)
#define DDSCXX_WARNING_MSVC_ON(x)

#endif /* OMG_DDS_CORE_MACROS_HPP_ */
MACROS_HPP

# dds/topic/TopicTraits.hpp
mkdir -p "${INSTALL_PREFIX}/include/dds/topic"
cat > "${INSTALL_PREFIX}/include/dds/topic/TopicTraits.hpp" <<'TOPICTRAITS_HPP'
/*
 * Stub TopicTraits.hpp – standalone replacement for CDR-only builds.
 */
#ifndef OMG_DDS_TOPIC_TOPIC_TRAITS_HPP_
#define OMG_DDS_TOPIC_TOPIC_TRAITS_HPP_

#include <string>
#include <cstdint>
#include <cstddef>

#include "org/eclipse/cyclonedds/core/cdr/basic_cdr_ser.hpp"
#include "org/eclipse/cyclonedds/core/cdr/cdr_enums.hpp"

namespace org { namespace eclipse { namespace cyclonedds { namespace topic {

using core::cdr::extensibility;
using core::cdr::encoding_version;
using core::cdr::allowable_encodings_t;

template <class TOPIC> class TopicTraits {
public:
    static constexpr bool isKeyless() { return false; }
    static constexpr bool defaultToXCDR2Encoding() { return false; }
    static constexpr const char* getTypeName() { return ""; }
    static constexpr size_t getSampleSize() { return sizeof(TOPIC); }
    static constexpr bool isSelfContained() { return true; }
    static constexpr allowable_encodings_t allowableEncodings() { return 0xFFFFFFFFu; }
    static constexpr extensibility getExtensibility() { return extensibility::ext_final; }
};

} } } }

namespace dds { namespace topic {

template <typename T>
struct is_topic_type { enum { value = 0 }; };

template <typename T>
struct topic_type_support { };

template <typename T>
struct topic_type_name {
    static std::string value() { return "Undefined"; }
};

} }

#define REGISTER_TOPIC_TYPE(TOPIC_TYPE) \
    namespace dds { namespace topic { \
    template<> struct is_topic_type<TOPIC_TYPE> { \
        enum { value = 1 }; \
    }; } }

#endif /* OMG_DDS_TOPIC_TOPIC_TRAITS_HPP_ */
TOPICTRAITS_HPP

# org/eclipse/cyclonedds/topic/datatopic.hpp
mkdir -p "${INSTALL_PREFIX}/include/org/eclipse/cyclonedds/topic"
cat > "${INSTALL_PREFIX}/include/org/eclipse/cyclonedds/topic/datatopic.hpp" <<'DATATOPIC_HPP'
/*
 * Stub datatopic.hpp – empty replacement for CDR-only builds.
 */
#ifndef CYCLONEDDS_TOPIC_DATATOPIC_HPP_
#define CYCLONEDDS_TOPIC_DATATOPIC_HPP_

#define DDSCXX_IMPL_TYPE(T)

#endif /* CYCLONEDDS_TOPIC_DATATOPIC_HPP_ */
DATATOPIC_HPP

# dds/dds.hpp
mkdir -p "${INSTALL_PREFIX}/include/dds"
cat > "${INSTALL_PREFIX}/include/dds/dds.hpp" <<'DDS_HPP'
/*
 * Stub dds/dds.hpp – standalone replacement for CDR-only builds.
 */
#ifndef DDS_DDS_HPP_
#define DDS_DDS_HPP_

#include "dds/topic/TopicTraits.hpp"

#endif /* DDS_DDS_HPP_ */
DDS_HPP

# --- 2c. Cross-compile CDR static library for QNX ---
log_info "Cross-compiling CDR static library for QNX"

mkdir -p "${CDR_BUILD_QNX_DIR}"

cat > "${CDR_BUILD_QNX_DIR}/CMakeLists.txt" <<CDR_CMAKE
cmake_minimum_required(VERSION 3.14)
project(lwrcl_cdr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(lwrcl_cdr STATIC
    ${CDR_CPP_ABS}/entity_properties.cpp
    ${CDR_CPP_ABS}/cdr_stream.cpp
    ${CDR_CPP_ABS}/basic_cdr_ser.cpp
    ${CDR_CPP_ABS}/extended_cdr_v1_ser.cpp
    ${CDR_CPP_ABS}/extended_cdr_v2_ser.cpp
)

target_include_directories(lwrcl_cdr PUBLIC
    ${INSTALL_PREFIX}/include
)

target_compile_definitions(lwrcl_cdr PUBLIC
    DDSCXX_NO_STD_OPTIONAL
)

install(TARGETS lwrcl_cdr
        ARCHIVE DESTINATION lib)
CDR_CMAKE

log_info "Configuring cmake build (CDR library)..."
cmake "${CDR_BUILD_QNX_DIR}" \
  -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}" \
  -DQNX_ARCH="${ARCH}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -B "${CDR_BUILD_QNX_DIR}/build"

log_info "Building CDR library (jobs=${JOBS})..."
cmake --build "${CDR_BUILD_QNX_DIR}/build" -j"${JOBS}"
log_info "Installing CDR library to ${INSTALL_PREFIX}..."
cmake --install "${CDR_BUILD_QNX_DIR}/build"

# --- 2d. Create CMake config stubs ---
log_info "Creating CMake config stubs"

mkdir -p "${INSTALL_PREFIX}/lib/cmake/CycloneDDS"
cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS/CycloneDDSConfig.cmake" <<'CYCLONE_CFG'
# Stub CycloneDDSConfig.cmake – CDR-only build (no DDS runtime)
if(TARGET CycloneDDS::ddsc)
    return()
endif()

get_filename_component(_CDR_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

add_library(CycloneDDS::ddsc STATIC IMPORTED GLOBAL)
set_target_properties(CycloneDDS::ddsc PROPERTIES
    IMPORTED_LOCATION "${_CDR_PREFIX}/lib/liblwrcl_cdr.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_CDR_PREFIX}/include"
)

foreach(_comp ${CycloneDDS_FIND_COMPONENTS})
    set(CycloneDDS_${_comp}_FOUND TRUE)
endforeach()

set(CycloneDDS_FOUND TRUE)
unset(_CDR_PREFIX)
CYCLONE_CFG

cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS/CycloneDDSConfigVersion.cmake" <<'CYCLONE_VER'
set(PACKAGE_VERSION "0.10.5")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
CYCLONE_VER

mkdir -p "${INSTALL_PREFIX}/lib/cmake/CycloneDDS-CXX"
cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfig.cmake" <<'CYCLONECXX_CFG'
# Stub CycloneDDS-CXXConfig.cmake – CDR-only build (no DDS runtime)
if(TARGET CycloneDDS-CXX::ddscxx)
    return()
endif()

get_filename_component(_CDR_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

add_library(CycloneDDS-CXX::ddscxx STATIC IMPORTED GLOBAL)
set_target_properties(CycloneDDS-CXX::ddscxx PROPERTIES
    IMPORTED_LOCATION "${_CDR_PREFIX}/lib/liblwrcl_cdr.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_CDR_PREFIX}/include"
)

set(CycloneDDS-CXX_FOUND TRUE)
unset(_CDR_PREFIX)
CYCLONECXX_CFG

cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfigVersion.cmake" <<'CYCLONECXX_VER'
set(PACKAGE_VERSION "0.10.5")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
CYCLONECXX_VER

log_info "vsomeip QNX build complete (action=${ACTION})"
log_info "  install_prefix=${INSTALL_PREFIX}"
log_info "  CDR library: ${INSTALL_PREFIX}/lib/liblwrcl_cdr.a"
