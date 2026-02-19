#!/usr/bin/env bash
set -euo pipefail

FAST_DDS_TAG="2.11.2"
FOONATHAN_MEMORY_TAG="1.3.1"
GOOGLETEST_TAG="1.13.0"
FAST_DDS_GEN_TAG="2.4.0"
GRADLE_TAG="7.5.1"
INSTALL_PREFIX="/opt/fast-dds"
GEN_INSTALL_PREFIX="/opt/fast-dds-gen"
GRADLE_PREFIX="/opt/gradle"
BUILD_DIR="${HOME}/build-fast-dds"
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
  --tag <version>       Fast-DDS version (default: ${FAST_DDS_TAG})
  --gen-tag <version>   Fast-DDS-Gen version (default: ${FAST_DDS_GEN_TAG})
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
    --gen-tag)
      FAST_DDS_GEN_TAG="$2"
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
  ${SUDO} pacman -Sy --noconfirm
  ${SUDO} pacman -S --noconfirm gcc make cmake automake autoconf unzip git openssl curl tar jre11-openjdk jdk11-openjdk wget
  ${SUDO} pacman -S --noconfirm community/opensc community/libp11 || true
  ${SUDO} archlinux-java set java-11-openjdk || true
fi

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
  -DCMAKE_CXX_FLAGS="-O3"
cmake --build "${WORKSPACE}/foonathan_memory_vendor/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/foonathan_memory_vendor/build"

# googletest
cmake -S "${WORKSPACE}/googletest" -B "${WORKSPACE}/googletest/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3"
cmake --build "${WORKSPACE}/googletest/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/googletest/build"

# Fast-CDR
cmake -S "${WORKSPACE}/thirdparty/fastcdr" -B "${WORKSPACE}/thirdparty/fastcdr/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3"
cmake --build "${WORKSPACE}/thirdparty/fastcdr/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/thirdparty/fastcdr/build"

# TinyXML2
cmake -S "${WORKSPACE}/thirdparty/tinyxml2" -B "${WORKSPACE}/thirdparty/tinyxml2/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_CXX_FLAGS="-O3 -fPIC"
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
  -DCMAKE_CXX_FLAGS="-DASIO_HAS_PTHREADS=1"
cmake --build "${WORKSPACE}/build" -j"${JOBS}"
${SUDO} cmake --install "${WORKSPACE}/build"

# Gradle
${SUDO} mkdir -p /usr/share/man/man1
${SUDO} rm -rf "${GRADLE_PREFIX}"
${SUDO} mkdir -p "${GRADLE_PREFIX}"
cd "${GRADLE_PREFIX}"
${SUDO} wget "https://services.gradle.org/distributions/gradle-${GRADLE_TAG}-bin.zip"
${SUDO} unzip "gradle-${GRADLE_TAG}-bin.zip"
${SUDO} rm -f "gradle-${GRADLE_TAG}-bin.zip"

export JAVA_HOME=/usr/lib/jvm/default
export PATH="${PATH}:${GRADLE_PREFIX}/gradle-${GRADLE_TAG}/bin"

# Fast-DDS-Gen
cd "${BUILD_DIR}"
${SUDO} rm -rf "${GEN_INSTALL_PREFIX}"
${SUDO} mkdir -p "${GEN_INSTALL_PREFIX}"
git clone --recursive -b "v${FAST_DDS_GEN_TAG}" https://github.com/eProsima/Fast-DDS-Gen.git fast-dds-gen
cd fast-dds-gen
gradle assemble
${SUDO} "${GRADLE_PREFIX}/gradle-${GRADLE_TAG}/bin/gradle" install --install_path="${GEN_INSTALL_PREFIX}"

# Update bashrc
sed -i -e "/export PATH=.*fast-dds\/bin/d" ~/.bashrc
echo "export PATH=\$PATH:${INSTALL_PREFIX}/bin" >> ~/.bashrc

sed -i -e "/export PATH=.*fast-dds-gen\/bin/d" ~/.bashrc
echo "export PATH=\$PATH:${GEN_INSTALL_PREFIX}/bin" >> ~/.bashrc

sed -i -e "/export PATH=.*gradle.*\/bin/d" ~/.bashrc
echo "export PATH=\$PATH:${GRADLE_PREFIX}/gradle-${GRADLE_TAG}/bin" >> ~/.bashrc

sed -i -e '/export JAVA_HOME=\/usr\/lib\/jvm\/default/d' ~/.bashrc
echo 'export JAVA_HOME=/usr/lib/jvm/default' >> ~/.bashrc

if ! grep -q "export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib" ~/.bashrc 2>/dev/null; then
  echo "export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
fi
${SUDO} ldconfig

echo "[OK] Installed Fast-DDS v${FAST_DDS_TAG} + Fast-DDS-Gen v${FAST_DDS_GEN_TAG} to ${INSTALL_PREFIX} (Arch Linux)"
