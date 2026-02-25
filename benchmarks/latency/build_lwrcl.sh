#!/bin/bash
# Build lwrcl latency binaries for CycloneDDS and FastDDS backends.
# Requires both backends to be installed (build_lwrcl.sh <backend> install).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/src/lwrcl_latency.cpp"
OUT_DIR="${SCRIPT_DIR}/bin"
mkdir -p "${OUT_DIR}"

# ── CycloneDDS ──────────────────────────────────────────────────────────────
echo "=== Building lwrcl + CycloneDDS ==="
g++ -O2 -std=c++17 \
  -I/opt/cyclonedds-libs/include \
  -I/opt/cyclonedds/include \
  -I/opt/cyclonedds/include/ddscxx \
  -I/opt/iceoryx/include/iceoryx/v2.0.6 \
  "${SRC}" \
  -L/opt/cyclonedds-libs/lib -L/opt/cyclonedds/lib -L/opt/iceoryx/lib \
  -llwrcl -lstd_msgs -lddscxx -lddsc \
  -Wl,-rpath,/opt/cyclonedds-libs/lib:/opt/cyclonedds/lib:/opt/iceoryx/lib \
  -o "${OUT_DIR}/lat_lwrcl_cyclone"
echo "  -> ${OUT_DIR}/lat_lwrcl_cyclone"

# ── FastDDS ──────────────────────────────────────────────────────────────────
echo "=== Building lwrcl + FastDDS ==="
g++ -O2 -std=c++17 \
  -I/opt/fast-dds-libs/include \
  -I/opt/fast-dds/include \
  "${SRC}" \
  -L/opt/fast-dds-libs/lib -L/opt/fast-dds/lib \
  -llwrcl -lstd_msgs -lfastrtps -lfastcdr \
  -Wl,-rpath,/opt/fast-dds-libs/lib:/opt/fast-dds/lib \
  -o "${OUT_DIR}/lat_lwrcl_fastdds"
echo "  -> ${OUT_DIR}/lat_lwrcl_fastdds"

echo ""
echo "Done. Binaries in ${OUT_DIR}/"
