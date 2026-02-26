#!/bin/bash
# Build lwrcl latency binaries for CycloneDDS, FastDDS, and Adaptive AUTOSAR backends.
# Requires each backend to be installed (build_all.sh <backend> install).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${SCRIPT_DIR}/src/lwrcl_latency.cpp"
OUT_DIR="${SCRIPT_DIR}/bin"
mkdir -p "${OUT_DIR}"

ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar_ap}"
AUTOSAR_AP_LIBS_PREFIX="${AUTOSAR_AP_LIBS_PREFIX:-/opt/autosar-ap-libs}"
VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"

# ── CycloneDDS ──────────────────────────────────────────────────────────────
echo "=== Building lwrcl + CycloneDDS ==="
g++ -O2 -std=c++17 \
  -I/opt/cyclonedds-libs/include \
  -I/opt/cyclonedds/include \
  -I/opt/cyclonedds/include/ddscxx \
  -I${ICEORYX_PREFIX}/include/iceoryx/v2.0.6 \
  "${SRC}" \
  -L/opt/cyclonedds-libs/lib -L/opt/cyclonedds/lib -L${ICEORYX_PREFIX}/lib \
  -llwrcl -lstd_msgs -lddscxx -lddsc \
  -Wl,-rpath,/opt/cyclonedds-libs/lib:/opt/cyclonedds/lib:${ICEORYX_PREFIX}/lib \
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

# ── Adaptive AUTOSAR ─────────────────────────────────────────────────────────
if [ -d "${AUTOSAR_AP_PREFIX}" ] && [ -d "${AUTOSAR_AP_LIBS_PREFIX}" ]; then
    echo "=== Building lwrcl + Adaptive AUTOSAR ==="

    # Step 1: Generate topic mapping & proxy/skeleton header from benchmark sources
    #         (same approach as build_apps.sh for adaptive-autosar)
    export PATH="${AUTOSAR_AP_PREFIX}/bin:${DDS_PREFIX}/bin:${PATH}"
    AUTOSAR_GEN_DIR="${SCRIPT_DIR}/autosar-gen"
    AUTOSAR_GEN_MAPPING="${AUTOSAR_GEN_DIR}/lwrcl_autosar_topic_mapping.yaml"
    AUTOSAR_GEN_MANIFEST="${AUTOSAR_GEN_DIR}/lwrcl_autosar_manifest.yaml"
    AUTOSAR_GEN_PROXY_SKELETON="${AUTOSAR_GEN_DIR}/generated/lwrcl_autosar_proxy_skeleton.hpp"
    AUTOSAR_EVENT_BINDING="${AUTOSAR_EVENT_BINDING:-auto}"

    mkdir -p "${AUTOSAR_GEN_DIR}/generated"

    echo "  Generating topic mapping from benchmark sources..."
    autosar-generate-comm-manifest \
      --apps-root "${SCRIPT_DIR}" \
      --output-mapping "${AUTOSAR_GEN_MAPPING}" \
      --output-manifest "${AUTOSAR_GEN_MANIFEST}" \
      --event-binding "${AUTOSAR_EVENT_BINDING}" \
      --print-summary

    echo "  Generating proxy/skeleton header..."
    autosar-generate-proxy-skeleton \
      --mapping "${AUTOSAR_GEN_MAPPING}" \
      --output "${AUTOSAR_GEN_PROXY_SKELETON}" \
      --namespace "autosar_generated" \
      --print-summary

    # Step 2: Compile with installed headers + app-specific generated proxy/skeleton.
    #         The generated header provides topic bindings compiled into the binary;
    #         it must appear BEFORE the library include dir so the app's version is found.
    g++ -O2 -std=c++17 \
      -DARA_COM_USE_CYCLONEDDS=1 \
      -I${AUTOSAR_GEN_DIR}/generated \
      -I${AUTOSAR_AP_LIBS_PREFIX}/include \
      -I${AUTOSAR_AP_PREFIX}/include \
      -I${DDS_PREFIX}/include \
      -I${DDS_PREFIX}/include/ddscxx \
      -I${ICEORYX_PREFIX}/include/iceoryx/v2.0.6 \
      "${SRC}" \
      -L${AUTOSAR_AP_LIBS_PREFIX}/lib -L${AUTOSAR_AP_PREFIX}/lib \
      -L${DDS_PREFIX}/lib -L${ICEORYX_PREFIX}/lib -L${VSOMEIP_PREFIX}/lib \
      -llwrcl -lstd_msgs -lara_com -lara_core \
      -lddscxx -lddsc \
      -liceoryx_posh -liceoryx_hoofs -liceoryx_platform \
      -lvsomeip3 -lboost_thread -lboost_system -lpthread \
      -Wl,-rpath,${AUTOSAR_AP_LIBS_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${DDS_PREFIX}/lib:${ICEORYX_PREFIX}/lib:${VSOMEIP_PREFIX}/lib \
      -o "${OUT_DIR}/lat_lwrcl_autosar"
    echo "  -> ${OUT_DIR}/lat_lwrcl_autosar"
else
    echo "=== Skipping Adaptive AUTOSAR (${AUTOSAR_AP_PREFIX} or ${AUTOSAR_AP_LIBS_PREFIX} not found) ==="
fi

echo ""
echo "Done. Binaries in ${OUT_DIR}/"
