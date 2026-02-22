#!/bin/bash
set -euo pipefail

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/fast-dds"
    LWRCL_PREFIX="/opt/fast-dds-libs"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/cyclonedds-libs"
elif [ "$BACKEND" = "vsomeip" ]; then
    DDS_PREFIX="/opt/cyclonedds"
    VSOMEIP_PREFIX="/opt/vsomeip"
    LWRCL_PREFIX="/opt/vsomeip-libs"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    AUTOSAR_AP_PREFIX="/opt/autosar_ap"
    DDS_PREFIX="/opt/cyclonedds"
    VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"
    LWRCL_PREFIX="/opt/autosar-ap-libs"
    AUTOSAR_APP_SOURCE_ROOT="${AUTOSAR_APP_SOURCE_ROOT:-${SCRIPT_DIR}/apps}"
else
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/apps/build-${BACKEND}"
INSTALL_DIR="${SCRIPT_DIR}/apps/install-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

mkdir -p "$INSTALL_DIR"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

# Add iceoryx libraries if present (used by CycloneDDS SHM/zero-copy)
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/apps"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
)

if [ "$BACKEND" = "fastdds" ]; then
    CMAKE_ARGS+=(
        -DCMAKE_SYSTEM_PREFIX_PATH="$LWRCL_PREFIX"
        -DCMAKE_PREFIX_PATH="$LWRCL_PREFIX"
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"
    )
elif [ "$BACKEND" = "cyclonedds" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    CMAKE_ARGS+=(
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
    )
elif [ "$BACKEND" = "vsomeip" ]; then
    export PATH="${DDS_PREFIX}/bin:${PATH}"
    export LD_LIBRARY_PATH="${VSOMEIP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    CMAKE_ARGS+=(
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}"
    )
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    export PATH="${AUTOSAR_AP_PREFIX}/bin:${DDS_PREFIX}/bin:${PATH}"
    export LD_LIBRARY_PATH="${AUTOSAR_AP_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    AUTOSAR_GEN_DIR="${BUILD_DIR}/autosar"
    AUTOSAR_GEN_MAPPING="${AUTOSAR_GEN_DIR}/lwrcl_autosar_topic_mapping.yaml"
    AUTOSAR_GEN_MANIFEST_YAML="${AUTOSAR_GEN_DIR}/lwrcl_autosar_manifest.yaml"
    AUTOSAR_GEN_ARXML="${AUTOSAR_GEN_DIR}/lwrcl_autosar_manifest.arxml"
    AUTOSAR_GEN_PROXY_SKELETON_DIR="${AUTOSAR_GEN_DIR}/generated"
    AUTOSAR_GEN_PROXY_SKELETON_HEADER="${AUTOSAR_GEN_PROXY_SKELETON_DIR}/lwrcl_autosar_proxy_skeleton.hpp"
    AUTOSAR_MAPPING_GENERATOR_CMD="${AUTOSAR_COMM_MANIFEST_GENERATOR:-autosar-generate-comm-manifest}"
    AUTOSAR_PROXY_SKELETON_GENERATOR_CMD="${AUTOSAR_PROXY_SKELETON_GENERATOR:-autosar-generate-proxy-skeleton}"

    mkdir -p "${AUTOSAR_GEN_DIR}"
    if ! command -v "${AUTOSAR_MAPPING_GENERATOR_CMD}" >/dev/null 2>&1; then
        echo "Adaptive AUTOSAR mapping generator command not found: ${AUTOSAR_MAPPING_GENERATOR_CMD}"
        echo "Install codegen tools from Adaptive-AUTOSAR and ensure PATH contains /opt/autosar_ap/bin."
        exit 1
    fi
    "${AUTOSAR_MAPPING_GENERATOR_CMD}" \
      --apps-root "${AUTOSAR_APP_SOURCE_ROOT}" \
      --output-mapping "${AUTOSAR_GEN_MAPPING}" \
      --output-manifest "${AUTOSAR_GEN_MANIFEST_YAML}" \
      --print-summary
    if ! command -v "${AUTOSAR_PROXY_SKELETON_GENERATOR_CMD}" >/dev/null 2>&1; then
        echo "Adaptive AUTOSAR proxy/skeleton generator command not found: ${AUTOSAR_PROXY_SKELETON_GENERATOR_CMD}"
        echo "Install codegen tools from Adaptive-AUTOSAR and ensure PATH contains /opt/autosar_ap/bin."
        exit 1
    fi
    "${AUTOSAR_PROXY_SKELETON_GENERATOR_CMD}" \
      --mapping "${AUTOSAR_GEN_MAPPING}" \
      --output "${AUTOSAR_GEN_PROXY_SKELETON_HEADER}" \
      --print-summary

    AUTOSAR_ARXML_GENERATOR="${AUTOSAR_ARXML_GENERATOR:-${AUTOSAR_AP_PREFIX}/tools/arxml_generator/generate_arxml.py}"
    if [ -f "${AUTOSAR_ARXML_GENERATOR}" ]; then
        if ! python3 -c "import yaml" >/dev/null 2>&1; then
            echo "python3 module 'yaml' is required for ARXML generation."
            echo "Install it with: python3 -m pip install pyyaml"
            exit 1
        fi
        python3 "${AUTOSAR_ARXML_GENERATOR}" \
          --input "${AUTOSAR_GEN_MANIFEST_YAML}" \
          --output "${AUTOSAR_GEN_ARXML}" \
          --overwrite \
          --print-summary
    else
        echo "Warning: ARXML generator not found at ${AUTOSAR_ARXML_GENERATOR}; mapping/manifest only."
    fi

    CMAKE_ARGS+=(
        -DAUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX}"
        -DDDS_PREFIX="${DDS_PREFIX}"
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}"
        -DAUTOSAR_GENERATED_PROXY_SKELETON_DIR="${AUTOSAR_GEN_PROXY_SKELETON_DIR}"
        -DCMAKE_PREFIX_PATH="${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP;${DDS_PREFIX}/lib/cmake"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$INSTALL_DIR"
    if [ "$BACKEND" = "adaptive-autosar" ]; then
        AUTOSAR_INSTALL_DIR="${LWRCL_PREFIX}/share/lwrcl/autosar"
        sudo mkdir -p "${AUTOSAR_INSTALL_DIR}"
        sudo cp "${AUTOSAR_GEN_MAPPING}" "${AUTOSAR_INSTALL_DIR}/lwrcl_autosar_topic_mapping.yaml"
        sudo cp "${AUTOSAR_GEN_MANIFEST_YAML}" "${AUTOSAR_INSTALL_DIR}/lwrcl_autosar_manifest.yaml"
        if [ -f "${AUTOSAR_GEN_ARXML}" ]; then
            sudo cp "${AUTOSAR_GEN_ARXML}" "${AUTOSAR_INSTALL_DIR}/lwrcl_autosar_manifest.arxml"
        fi
    fi
fi
