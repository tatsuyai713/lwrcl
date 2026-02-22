#!/bin/bash
set -euo pipefail

# Input sudo password here to avoid prompt after build is completed.
sudo echo "OK"

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

generate_adaptive_autosar_artifacts_qnx() {
    local manifest_input mapping_input output_dir output_arxml output_mapping output_manifest
    local apps_root mapping_generator_cmd
    local generator=""
    manifest_input="${AUTOSAR_ARXML_MANIFEST_YAML:-}"
    mapping_input="${AUTOSAR_TOPIC_MAPPING_YAML:-}"
    apps_root="${AUTOSAR_APP_SOURCE_ROOT:-${SCRIPT_DIR}/apps}"
    mapping_generator_cmd="${AUTOSAR_COMM_MANIFEST_GENERATOR:-autosar-generate-comm-manifest}"
    output_dir="${BUILD_DIR}/autosar"
    output_arxml="${output_dir}/lwrcl_autosar_manifest.arxml"
    output_mapping="${output_dir}/lwrcl_autosar_topic_mapping.yaml"
    output_manifest="${output_dir}/lwrcl_autosar_manifest.yaml"

    mkdir -p "${output_dir}"

    if [ -n "${manifest_input}" ] && [ -f "${manifest_input}" ] && [ -n "${mapping_input}" ] && [ -f "${mapping_input}" ]; then
        cp "${mapping_input}" "${output_mapping}"
        cp "${manifest_input}" "${output_manifest}"
    else
        if ! command -v "${mapping_generator_cmd}" >/dev/null 2>&1; then
            echo "Adaptive AUTOSAR mapping generator command not found: ${mapping_generator_cmd}"
            echo "Install codegen tools from Adaptive-AUTOSAR and ensure PATH contains /opt/autosar_ap/bin."
            exit 1
        fi
        "${mapping_generator_cmd}" \
          --apps-root "${apps_root}" \
          --output-mapping "${output_mapping}" \
          --output-manifest "${output_manifest}" \
          --print-summary
    fi

    if [ "${AUTOSAR_SKIP_ARXML_GEN:-0}" = "1" ]; then
        echo "Skipping ARXML generation (AUTOSAR_SKIP_ARXML_GEN=1)."
        return
    fi

    if [ -n "${AUTOSAR_ARXML_GENERATOR:-}" ] && [ -f "${AUTOSAR_ARXML_GENERATOR}" ]; then
        generator="${AUTOSAR_ARXML_GENERATOR}"
    elif [ -f "${AUTOSAR_AP_PREFIX}/tools/arxml_generator/generate_arxml.py" ]; then
        generator="${AUTOSAR_AP_PREFIX}/tools/arxml_generator/generate_arxml.py"
    elif [ -f "/opt/autosar_ap/tools/arxml_generator/generate_arxml.py" ]; then
        generator="/opt/autosar_ap/tools/arxml_generator/generate_arxml.py"
    fi

    if [ -z "${generator}" ]; then
        echo "ARXML generator not found."
        echo "Set AUTOSAR_ARXML_GENERATOR or install Adaptive AUTOSAR tools under ${AUTOSAR_AP_PREFIX}/tools/arxml_generator."
        exit 1
    fi

    if ! python3 -c "import yaml" >/dev/null 2>&1; then
        echo "python3 module 'yaml' is required for ARXML generation."
        echo "Install it with: python3 -m pip install pyyaml"
        exit 1
    fi

    python3 "${generator}" \
      --input "${output_manifest}" \
      --output "${output_arxml}" \
      --overwrite \
      --print-summary
}

install_adaptive_autosar_artifacts_qnx() {
    local output_dir install_dir
    output_dir="${BUILD_DIR}/autosar"
    install_dir="${LWRCL_PREFIX}/share/lwrcl/autosar"
    sudo mkdir -p "${install_dir}"

    if [ -f "${output_dir}/lwrcl_autosar_manifest.arxml" ]; then
        sudo cp "${output_dir}/lwrcl_autosar_manifest.arxml" "${install_dir}/"
    fi
    if [ -f "${output_dir}/lwrcl_autosar_topic_mapping.yaml" ]; then
        sudo cp "${output_dir}/lwrcl_autosar_topic_mapping.yaml" "${install_dir}/"
    fi
    if [ -f "${output_dir}/lwrcl_autosar_manifest.yaml" ]; then
        sudo cp "${output_dir}/lwrcl_autosar_manifest.yaml" "${install_dir}/"
    fi
}

if [ -z "${QNX_TARGET:-}" ] || [ -z "${QNX_HOST:-}" ]; then
    echo "Please source QNX SDP environment (QNX_HOST/QNX_TARGET)."
    exit 1
fi

QNX_ARCH="${AUTOSAR_QNX_ARCH:-aarch64le}"
ICEORYX_PREFIX="/opt/qnx/iceoryx"
IDLC_PREFIX="${IDLC_PREFIX:-/opt/cyclonedds}"
HOST_ICEORYX_PREFIX="${HOST_ICEORYX_PREFIX:-/opt/iceoryx}"

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/qnx/fast-dds/aarch64le/usr"
    LWRCL_PREFIX="/opt/qnx/fast-dds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_aarch64le.cmake"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    LWRCL_PREFIX="/opt/qnx/cyclonedds-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    DDS_PREFIX="/opt/qnx/cyclonedds"
    AUTOSAR_AP_PREFIX="/opt/qnx/autosar_ap/${QNX_ARCH}"
    LWRCL_PREFIX="/opt/qnx/autosar-ap-libs"
    TOOLCHAIN_FILE="${SCRIPT_DIR}/scripts/cmake/qnx_toolchain.cmake"
else
    echo "Usage: $0 <fastdds|cyclonedds|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/data_types/build_qnx-${BACKEND}"

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

sudo mkdir -p "$LWRCL_PREFIX"

export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
if [ -d "${ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ -d "${IDLC_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${IDLC_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ -d "${HOST_ICEORYX_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${HOST_ICEORYX_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ "$BACKEND" = "adaptive-autosar" ] && [ -d "${AUTOSAR_AP_PREFIX}/lib" ]; then
    export LD_LIBRARY_PATH="${AUTOSAR_AP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
if [ "$BACKEND" = "adaptive-autosar" ] && [ -d "${AUTOSAR_AP_PREFIX}/bin" ]; then
    export PATH="${AUTOSAR_AP_PREFIX}/bin:${PATH}"
fi

if [ "$BACKEND" = "adaptive-autosar" ]; then
    generate_adaptive_autosar_artifacts_qnx
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/data_types"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
)

if [ "$BACKEND" = "fastdds" ]; then
    CMAKE_ARGS+=(
        -DOPENSSL_ROOT_DIR="${QNX_TARGET}/aarch64le/usr"
        -DOPENSSL_INCLUDE_DIR="${QNX_TARGET}/usr/include"
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
        -DCMAKE_SYSTEM_PREFIX_PATH="${QNX_TARGET}/aarch64le/usr/"
        -DCMAKE_PREFIX_PATH="${QNX_TARGET}/aarch64le/usr/"
    )
elif [ "$BACKEND" = "cyclonedds" ]; then
    IDLC_BIN_DIR=""
    if [ -x "${DDS_PREFIX}/bin/idlc" ]; then
        IDLC_BIN_DIR="${DDS_PREFIX}/bin"
    elif [ -x "${IDLC_PREFIX}/bin/idlc" ]; then
        IDLC_BIN_DIR="${IDLC_PREFIX}/bin"
    fi
    if [ -n "${IDLC_BIN_DIR}" ]; then
        export PATH="${IDLC_BIN_DIR}:${PATH}"
    else
        echo "Warning: idlc not found under ${DDS_PREFIX}/bin or ${IDLC_PREFIX}/bin"
    fi
    CMAKE_ARGS+=(
        -DICEORYX_PREFIX="${ICEORYX_PREFIX}"
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
    )
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    IDLC_BIN_DIR=""
    if [ -x "${DDS_PREFIX}/bin/idlc" ]; then
        IDLC_BIN_DIR="${DDS_PREFIX}/bin"
    elif [ -x "${IDLC_PREFIX}/bin/idlc" ]; then
        IDLC_BIN_DIR="${IDLC_PREFIX}/bin"
    fi
    if [ -n "${IDLC_BIN_DIR}" ]; then
        export PATH="${IDLC_BIN_DIR}:${PATH}"
    else
        echo "Warning: idlc not found under ${DDS_PREFIX}/bin or ${IDLC_PREFIX}/bin"
    fi
    CMAKE_ARGS+=(
        -DAUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX}"
        -DDDS_PREFIX="${DDS_PREFIX}"
        -DICEORYX_PREFIX="${ICEORYX_PREFIX}"
        -DCMAKE_PREFIX_PATH="${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP;${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake"
    )
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$LWRCL_PREFIX"
    if [ "$BACKEND" = "adaptive-autosar" ]; then
        install_adaptive_autosar_artifacts_qnx
    fi
fi
