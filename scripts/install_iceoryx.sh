#!/usr/bin/env bash
# ============================================================================
# Install Eclipse iceoryx for CycloneDDS shared-memory (zero-copy) transport.
#
# iceoryx provides inter-process shared memory communication.
# When CycloneDDS is built with -DENABLE_SHM=ON it uses iceoryx
# for zero-copy data transfer between processes on the same host.
#
# Prerequisites: cmake (>=3.16), g++ (>=7), libacl1-dev (Debian/Ubuntu)
#
# Usage:
#   ./install_iceoryx.sh [INSTALL_PREFIX]
#   Default prefix: /opt/iceoryx
# ============================================================================
set -euo pipefail

ICEORYX_TAG="v2.0.5"
INSTALL_PREFIX="${1:-/opt/iceoryx}"
BUILD_DIR="$HOME/build-iceoryx"

echo "=== Installing Eclipse iceoryx ${ICEORYX_TAG} ==="
echo "  Install prefix : ${INSTALL_PREFIX}"
echo "  Build directory: ${BUILD_DIR}"
echo ""

# Install OS-level dependencies (Debian / Ubuntu)
if command -v apt-get &>/dev/null; then
    sudo apt-get update -qq
    sudo apt-get install -y -qq libacl1-dev libncurses5-dev
fi

sudo mkdir -p "${INSTALL_PREFIX}"
sudo chmod 777 -R "${INSTALL_PREFIX}"

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Clone iceoryx
git clone https://github.com/eclipse-iceoryx/iceoryx.git
cd iceoryx
git checkout "${ICEORYX_TAG}"

# Build using iceoryx_meta (top-level CMake project)
cmake -B build -S iceoryx_meta \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TEST=OFF \
    -DINTROSPECTION=OFF \
    -DBUILD_DOC=OFF \
    -DEXAMPLES=OFF

cmake --build build -j"$(nproc)"
sudo cmake --install build

echo ""
echo "=== iceoryx ${ICEORYX_TAG} installed to ${INSTALL_PREFIX} ==="
echo ""
echo "To use iceoryx, ensure iox-roudi is running before launching CycloneDDS apps:"
echo "  ${INSTALL_PREFIX}/bin/iox-roudi &"
echo ""
echo "Runtime library path:"
echo "  export LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:\${LD_LIBRARY_PATH:-}"
