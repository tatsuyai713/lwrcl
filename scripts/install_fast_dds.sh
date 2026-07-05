#!/usr/bin/env bash
set -euo pipefail

FAST_DDS_TAG="2.11.2"
FOONATHAN_MEMORY_TAG="1.3.1"
GOOGLETEST_TAG="1.13.0"
FAST_DDS_GEN_TAG="3.3.1"
INSTALL_PREFIX="/opt/fast-dds"
BUILD_DIR="${HOME}/build-fast-dds"
if command -v nproc >/dev/null 2>&1; then
  _nproc="$(nproc)"
elif [[ "$(uname -s)" == "Darwin" ]]; then
  _nproc="$(sysctl -n hw.ncpu)"
else
  _nproc=4
fi
if [[ -r /proc/meminfo ]]; then
  _mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
  _mem_jobs=$(( _mem_kb / 1572864 ))
  [[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
  JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
else
  JOBS="${_nproc}"
fi
[[ "${JOBS}" -lt 1 ]] && JOBS=1
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --prefix <path>       Install prefix (default: ${INSTALL_PREFIX})
  --tag <version>       Fast-DDS version (default: ${FAST_DDS_TAG})
  --build-dir <path>    Build directory (default: \${HOME}/build-fast-dds)
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
      FAST_DDS_TAG="$2"
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

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]]; then
  if command -v apt-get >/dev/null 2>&1; then
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | ${SUDO} tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
    ${SUDO} apt-add-repository -y "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
    ${SUDO} apt-get update -qq
    ${SUDO} apt-get install -y --allow-downgrades gcc g++ make automake autoconf unzip git vim openssl gcc make cmake=3.31.11* cmake-data=3.31.11* curl tar wget p11-kit libasio-dev
    ${SUDO} apt-mark hold cmake cmake-data
  elif command -v brew >/dev/null 2>&1; then
    brew update
    brew install git automake autoconf libtool openssl@3 wget || true
    "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_cmake_macos.sh"
  else
    echo "[WARN] Unknown package manager. Ensure cmake, git, automake, autoconf, libtool, and OpenSSL are installed."
  fi
fi

BREW_PREFIX="$(brew --prefix 2>/dev/null || true)"
OPENSSL_PREFIX=""
if [[ -n "${BREW_PREFIX}" ]] && [[ -d "${BREW_PREFIX}/opt/openssl@3" ]]; then
  OPENSSL_PREFIX="${BREW_PREFIX}/opt/openssl@3"
  export PKG_CONFIG_PATH="${OPENSSL_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
fi

# CMake 4 removed compatibility with projects declaring cmake_minimum_required()
# older than 3.5. Some Fast-DDS 2.11 third-party dependencies still do that.
CMAKE_POLICY_ARG=""
if cmake --version | awk 'NR==1 {split($3, v, "."); exit !(v[1] >= 4)}'; then
  CMAKE_POLICY_ARG="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
fi

patch_cmake4_removed_old_policies() {
  local cmake_file="$1"
  [[ -f "${cmake_file}" ]] || return 0
  if [[ "$(uname -s)" == "Darwin" ]]; then
    sed -i '' -E 's/cmake_policy[[:space:]]*\([[:space:]]*SET[[:space:]]+(CMP0063)[[:space:]]+OLD[[:space:]]*\)/cmake_policy(SET \1 NEW)/g' "${cmake_file}"
  else
    sed -i -E 's/cmake_policy[[:space:]]*\([[:space:]]*SET[[:space:]]+(CMP0063)[[:space:]]+OLD[[:space:]]*\)/cmake_policy(SET \1 NEW)/g' "${cmake_file}"
  fi
}

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

git clone https://github.com/eProsima/Fast-DDS.git -b "v${FAST_DDS_TAG}" --depth 1
cd Fast-DDS
WORKSPACE="$PWD"
git submodule update --init "${WORKSPACE}/thirdparty/asio" "${WORKSPACE}/thirdparty/fastcdr" "${WORKSPACE}/thirdparty/tinyxml2"

git clone https://github.com/eProsima/foonathan_memory_vendor.git -b "v${FOONATHAN_MEMORY_TAG}" "${WORKSPACE}/foonathan_memory_vendor"
git clone https://github.com/google/googletest.git -b "v${GOOGLETEST_TAG}" --depth 1 "${WORKSPACE}/googletest"

${SUDO} rm -rf "${INSTALL_PREFIX}"
${SUDO} mkdir -p "${INSTALL_PREFIX}"

# foonathan_memory_vendor
cmake -S "${WORKSPACE}/foonathan_memory_vendor" -B "${WORKSPACE}/foonathan_memory_vendor/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3" \
  ${CMAKE_POLICY_ARG:+"${CMAKE_POLICY_ARG}"}
cmake --build "${WORKSPACE}/foonathan_memory_vendor/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/foonathan_memory_vendor/build"

# googletest
cmake -S "${WORKSPACE}/googletest" -B "${WORKSPACE}/googletest/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3" \
  ${CMAKE_POLICY_ARG:+"${CMAKE_POLICY_ARG}"}
cmake --build "${WORKSPACE}/googletest/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/googletest/build"

# Fast-CDR
cmake -S "${WORKSPACE}/thirdparty/fastcdr" -B "${WORKSPACE}/thirdparty/fastcdr/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3" \
  ${CMAKE_POLICY_ARG:+"${CMAKE_POLICY_ARG}"}
cmake --build "${WORKSPACE}/thirdparty/fastcdr/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/thirdparty/fastcdr/build"

# TinyXML2
patch_cmake4_removed_old_policies "${WORKSPACE}/thirdparty/tinyxml2/CMakeLists.txt"
cmake -S "${WORKSPACE}/thirdparty/tinyxml2" -B "${WORKSPACE}/thirdparty/tinyxml2/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3 -fPIC" \
  ${CMAKE_POLICY_ARG:+"${CMAKE_POLICY_ARG}"}
cmake --build "${WORKSPACE}/thirdparty/tinyxml2/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/thirdparty/tinyxml2/build"

# ASIO
cd "${WORKSPACE}/thirdparty/asio/asio"
./autogen.sh
./configure CXXFLAGS="-O3 -g -DASIO_HAS_PTHREADS -D_GLIBCXX_HAS_GTHREADS -std=c++11" --prefix="${INSTALL_PREFIX}"
make -j"${JOBS}"
${SUDO} make install

# Fast-DDS
cmake -S "${WORKSPACE}" -B "${WORKSPACE}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -Dfastcdr_DIR="${INSTALL_PREFIX}/lib/cmake/fastcdr/" \
  -Dfoonathan_memory_DIR="${INSTALL_PREFIX}/lib/foonathan_memory/cmake/" \
  -DCMAKE_SYSTEM_PREFIX_PATH="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
  ${OPENSSL_PREFIX:+-DOPENSSL_ROOT_DIR="${OPENSSL_PREFIX}"} \
  -DCMAKE_CXX_FLAGS="-DASIO_HAS_PTHREADS=1" \
  ${CMAKE_POLICY_ARG:+"${CMAKE_POLICY_ARG}"}
cmake --build "${WORKSPACE}/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/build"

# Update shell startup file with a hint for manually-run tools.
if [[ "$(uname -s)" == "Darwin" ]]; then
  SHELL_RC="${HOME}/.zshrc"
  sed -i '' -e "/export PATH=.*fast-dds\/bin/d" "${SHELL_RC}" 2>/dev/null || true
  echo "export PATH=\"\$PATH:${INSTALL_PREFIX}/bin\"" >> "${SHELL_RC}"
else
  SHELL_RC="${HOME}/.bashrc"
  sed -i -e "/export PATH=.*fast-dds\/bin/d" "${SHELL_RC}" 2>/dev/null || true
  echo "export PATH=\$PATH:${INSTALL_PREFIX}/bin" >> "${SHELL_RC}"
  if ! grep -q "export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib" "${SHELL_RC}" 2>/dev/null; then
    echo "export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:\$LD_LIBRARY_PATH" >> "${SHELL_RC}"
  fi
fi
if command -v ldconfig >/dev/null 2>&1; then
  ${SUDO} ldconfig
fi

echo "[OK] Installed Fast-DDS v${FAST_DDS_TAG} to ${INSTALL_PREFIX}"
