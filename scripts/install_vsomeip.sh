#!/usr/bin/env bash
# ===========================================================================
# install_vsomeip.sh – Build and install vsomeip (SOME/IP) + CDR library
# ===========================================================================
# This script installs:
#   1. vsomeip (SOME/IP transport)
#   2. CDR serialization library extracted from cyclonedds-cxx (no DDS runtime)
#   3. Stub CMake configs so CycloneDDS find_package() resolves to the CDR lib
#
# Dependencies: Boost >= 1.66, cmake, g++ with C++17 support
# ===========================================================================
set -euo pipefail

VSOMEIP_TAG="3.4.10"
CYCLONEDDS_CXX_TAG="0.10.5"
INSTALL_PREFIX="/opt/vsomeip"
BUILD_DIR="${HOME}/build-vsomeip"
_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --prefix <path>       Install prefix (default: ${INSTALL_PREFIX})
  --tag <version>       vsomeip version (default: ${VSOMEIP_TAG})
  --cxx-tag <version>   CycloneDDS-CXX version for CDR (default: ${CYCLONEDDS_CXX_TAG})
  --build-dir <path>    Build directory (default: \${HOME}/build-vsomeip)
  --jobs <N>            Parallel build jobs (default: auto)
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
      VSOMEIP_TAG="$2"
      shift 2
      ;;
    --cxx-tag)
      CYCLONEDDS_CXX_TAG="$2"
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

if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ -f "${INSTALL_PREFIX}/lib/libvsomeip3.so" ]] && [[ -f "${INSTALL_PREFIX}/lib/liblwrcl_cdr.a" ]]; then
  echo "[INFO] vsomeip + CDR already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
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

echo "=== Installing vsomeip ${VSOMEIP_TAG} + CDR library to ${INSTALL_PREFIX} ==="

# ---------------------------------------------------------------------------
# 1. Install system dependencies
# ---------------------------------------------------------------------------
if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]]; then
  if command -v apt-get &>/dev/null; then
    echo "--- Installing system dependencies (apt) ---"
    ${SUDO} apt-get update -qq
    ${SUDO} apt-get install -y --no-install-recommends \
        build-essential cmake git \
        libboost-system-dev libboost-thread-dev libboost-log-dev \
        libboost-filesystem-dev
  elif command -v pacman &>/dev/null; then
    echo "--- Installing system dependencies (pacman) ---"
    ${SUDO} pacman -Sy --noconfirm base-devel cmake git boost
  else
    echo "[WARN] Unknown package manager. Ensure cmake, git, and boost are installed."
  fi
fi

# ---------------------------------------------------------------------------
# 2. Create install directory
# ---------------------------------------------------------------------------
${SUDO} mkdir -p "${INSTALL_PREFIX}"
${SUDO} chmod 777 -R "${INSTALL_PREFIX}"

# ---------------------------------------------------------------------------
# 3. Clone and build vsomeip
# ---------------------------------------------------------------------------
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

if [ ! -d vsomeip ]; then
    git clone https://github.com/COVESA/vsomeip.git
fi
cd vsomeip
git checkout "${VSOMEIP_TAG}"

mkdir -p build && cd build

cmake .. \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_SIGNAL_HANDLING=1 \
    -DBUILD_SHARED_LIBS=ON \
    -DENABLE_MULTIPLE_ROUTING_MANAGERS=1 \
    -DCMAKE_CXX_FLAGS="-Wno-stringop-overflow"

make -j"${JOBS}"
${SUDO} make install

# Re-apply permissions after ${SUDO} make install (which creates root-owned dirs)
${SUDO} chmod 777 -R "${INSTALL_PREFIX}"

# ---------------------------------------------------------------------------
# 4. Create default vsomeip configuration
# ---------------------------------------------------------------------------
${SUDO} mkdir -p "${INSTALL_PREFIX}/etc"
cat <<'VSOMEIP_CFG' | ${SUDO} tee "${INSTALL_PREFIX}/etc/vsomeip-lwrcl.json" > /dev/null
{
    "unicast": "127.0.0.1",
    "logging": {
        "level": "warning",
        "console": "true",
        "file": { "enable": "false" },
        "dlt": "false"
    },
    "applications": [],
    "services": [],
    "routing": "lwrcl_routing",
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
VSOMEIP_CFG

# ---------------------------------------------------------------------------
# 5. Build CDR serialization library from cyclonedds-cxx (no DDS runtime)
# ---------------------------------------------------------------------------
echo ""
echo "=== Building CDR serialization library ==="

cd "${BUILD_DIR}"
if [ ! -d cyclonedds-cxx ]; then
    git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git
fi
cd cyclonedds-cxx
git checkout "${CYCLONEDDS_CXX_TAG}"

CDR_SRC_BASE="src/ddscxx"
CDR_INC="${CDR_SRC_BASE}/include"
CDR_CPP="${CDR_SRC_BASE}/src/org/eclipse/cyclonedds/core/cdr"

# --- 5a. Install CDR headers from cyclonedds-cxx ---
echo "--- Installing CDR headers ---"
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
    echo "  Patched cdr_stream.hpp: added #include <cstring>"
fi

# --- 5b. Create stub headers for external CycloneDDS dependencies ---
echo "--- Creating stub headers ---"

# dds/ddsrt/endian.h – Platform endianness detection (pure preprocessor)
mkdir -p "${INSTALL_PREFIX}/include/dds/ddsrt"
cat > "${INSTALL_PREFIX}/include/dds/ddsrt/endian.h" << 'ENDIAN_H'
/*
 * Stub endian.h – standalone replacement for CycloneDDS ddsrt/endian.h
 * Provides DDSRT_ENDIAN, DDSRT_LITTLE_ENDIAN, DDSRT_BIG_ENDIAN macros.
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

# dds/core/macros.hpp – OMG_DDS_API / warning macros
mkdir -p "${INSTALL_PREFIX}/include/dds/core"
cat > "${INSTALL_PREFIX}/include/dds/core/macros.hpp" << 'MACROS_HPP'
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

# dds/topic/TopicTraits.hpp – stub for generated data type headers
mkdir -p "${INSTALL_PREFIX}/include/dds/topic"
cat > "${INSTALL_PREFIX}/include/dds/topic/TopicTraits.hpp" << 'TOPICTRAITS_HPP'
/*
 * Stub TopicTraits.hpp – standalone replacement for CDR-only builds.
 * Provides all declarations that idlc-generated code specializes.
 */
#ifndef OMG_DDS_TOPIC_TOPIC_TRAITS_HPP_
#define OMG_DDS_TOPIC_TOPIC_TRAITS_HPP_

#include <string>
#include <cstdint>
#include <cstddef>

#include "org/eclipse/cyclonedds/core/cdr/basic_cdr_ser.hpp"
#include "org/eclipse/cyclonedds/core/cdr/cdr_enums.hpp"

/* ------------------------------------------------------------------ */
/* org::eclipse::cyclonedds::topic::TopicTraits<TOPIC>                */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* dds::topic – is_topic_type, topic_type_name, topic_type_support    */
/* ------------------------------------------------------------------ */
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

# org/eclipse/cyclonedds/topic/datatopic.hpp – empty stub
mkdir -p "${INSTALL_PREFIX}/include/org/eclipse/cyclonedds/topic"
cat > "${INSTALL_PREFIX}/include/org/eclipse/cyclonedds/topic/datatopic.hpp" << 'DATATOPIC_HPP'
/*
 * Stub datatopic.hpp – empty replacement for CDR-only builds.
 * The real datatopic.hpp provides DDS transport serdata functions
 * which are not needed for SOME/IP transport.
 */
#ifndef CYCLONEDDS_TOPIC_DATATOPIC_HPP_
#define CYCLONEDDS_TOPIC_DATATOPIC_HPP_
#endif /* CYCLONEDDS_TOPIC_DATATOPIC_HPP_ */
DATATOPIC_HPP

# dds/dds.hpp – stub for snake_case wrapper headers
mkdir -p "${INSTALL_PREFIX}/include/dds"
cat > "${INSTALL_PREFIX}/include/dds/dds.hpp" << 'DDS_HPP'
/*
 * Stub dds/dds.hpp – standalone replacement for CDR-only builds.
 * The real dds/dds.hpp pulls in the entire CycloneDDS-CXX API.
 * This stub only provides the TopicTraits header which is needed
 * by idlc-generated data type headers.
 */
#ifndef DDS_DDS_HPP_
#define DDS_DDS_HPP_

#include "dds/topic/TopicTraits.hpp"

#endif /* DDS_DDS_HPP_ */
DDS_HPP

# --- 5c. Build CDR static library ---
echo "--- Building CDR static library ---"

CDR_BUILD_DIR="${BUILD_DIR}/cdr-build"
mkdir -p "${CDR_BUILD_DIR}"

# Create a minimal CMakeLists.txt for building the CDR library
cat > "${CDR_BUILD_DIR}/CMakeLists.txt" << CDR_CMAKE
cmake_minimum_required(VERSION 3.14)
project(lwrcl_cdr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The CDR source files from cyclonedds-cxx
set(CDR_SRC_DIR "${BUILD_DIR}/cyclonedds-cxx/${CDR_CPP}")

add_library(lwrcl_cdr STATIC
    \${CDR_SRC_DIR}/entity_properties.cpp
    \${CDR_SRC_DIR}/cdr_stream.cpp
    \${CDR_SRC_DIR}/basic_cdr_ser.cpp
    \${CDR_SRC_DIR}/extended_cdr_v1_ser.cpp
    \${CDR_SRC_DIR}/extended_cdr_v2_ser.cpp
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

cd "${CDR_BUILD_DIR}"
cmake . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON

make -j"${JOBS}"
make install

# --- 5d. Create CMake config stubs for CycloneDDS / CycloneDDS-CXX ---
# These allow find_package(CycloneDDS) to resolve to the CDR-only library
# when building data types for the vsomeip backend.
echo "--- Creating CMake config stubs ---"

mkdir -p "${INSTALL_PREFIX}/lib/cmake/CycloneDDS"
cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS/CycloneDDSConfig.cmake" << 'CYCLONE_CFG'
# ===========================================================================
# Stub CycloneDDSConfig.cmake – CDR-only build (no DDS runtime)
# ===========================================================================
# This config provides the CycloneDDS::ddsc imported target pointing to the
# standalone CDR serialization library (liblwrcl_cdr.a).
# ===========================================================================
if(TARGET CycloneDDS::ddsc)
    return()
endif()

get_filename_component(_CDR_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

add_library(CycloneDDS::ddsc STATIC IMPORTED GLOBAL)
set_target_properties(CycloneDDS::ddsc PROPERTIES
    IMPORTED_LOCATION "${_CDR_PREFIX}/lib/liblwrcl_cdr.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_CDR_PREFIX}/include"
)

# Handle COMPONENTS (lifecycle_msgs etc. request COMPONENTS CXX)
foreach(_comp ${CycloneDDS_FIND_COMPONENTS})
    set(CycloneDDS_${_comp}_FOUND TRUE)
endforeach()

set(CycloneDDS_FOUND TRUE)
unset(_CDR_PREFIX)
CYCLONE_CFG

cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS/CycloneDDSConfigVersion.cmake" << 'CYCLONE_VER'
set(PACKAGE_VERSION "0.10.5")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
CYCLONE_VER

mkdir -p "${INSTALL_PREFIX}/lib/cmake/CycloneDDS-CXX"
cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfig.cmake" << 'CYCLONECXX_CFG'
# ===========================================================================
# Stub CycloneDDS-CXXConfig.cmake – CDR-only build (no DDS runtime)
# ===========================================================================
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

cat > "${INSTALL_PREFIX}/lib/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfigVersion.cmake" << 'CYCLONECXX_VER'
set(PACKAGE_VERSION "0.10.5")
set(PACKAGE_VERSION_COMPATIBLE TRUE)
set(PACKAGE_VERSION_EXACT FALSE)
CYCLONECXX_VER

# ---------------------------------------------------------------------------
# 6. Verify installation
# ---------------------------------------------------------------------------
echo ""
echo "=== vsomeip ${VSOMEIP_TAG} + CDR installation complete ==="
echo "  Install prefix: ${INSTALL_PREFIX}"
echo "  Libraries:      ${INSTALL_PREFIX}/lib/"
echo "  Headers:        ${INSTALL_PREFIX}/include/"
echo "  CDR library:    ${INSTALL_PREFIX}/lib/liblwrcl_cdr.a"
echo "  CMake configs:  ${INSTALL_PREFIX}/lib/cmake/"
echo "  Config:         ${INSTALL_PREFIX}/etc/vsomeip-lwrcl.json"
echo ""
echo "Set the following environment variables before using:"
echo "  export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
echo "  export VSOMEIP_CONFIGURATION=${INSTALL_PREFIX}/etc/vsomeip-lwrcl.json"
echo ""
echo "NOTE: CycloneDDS is NO LONGER needed at runtime."
echo "      idlc from CycloneDDS is still needed at build time for data type generation."
echo ""

ls -la "${INSTALL_PREFIX}/lib/libvsomeip"* 2>/dev/null || echo "[INFO] Check lib directory"
ls -la "${INSTALL_PREFIX}/lib/liblwrcl_cdr"* 2>/dev/null || echo "[INFO] Check CDR library"
