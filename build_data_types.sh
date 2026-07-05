#!/bin/bash
set -euo pipefail

BACKEND="${1:-}"
ACTION="${2:-}"
ORIGINAL_BACKEND="${BACKEND}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif [ "$(uname -s)" = "Darwin" ]; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi
BREW_PREFIX="$(brew --prefix 2>/dev/null || true)"

ensure_dir() {
    local dir="$1"
    if mkdir -p "$dir" 2>/dev/null; then
        return
    fi
    sudo mkdir -p "$dir"
}

copy_file() {
    local src="$1"
    local dst="$2"
    if cp "$src" "$dst" 2>/dev/null; then
        return
    fi
    sudo cp "$src" "$dst"
}

cmake_install() {
    local build_dir="$1"
    local prefix="$2"
    if [ -w "$prefix" ]; then
        cmake --install "$build_dir" --prefix "$prefix"
    else
        sudo cmake --install "$build_dir" --prefix "$prefix"
    fi
}

resolve_fastddsgen() {
    local configured="${FASTDDS_GEN_BIN:-${FASTDDS_GEN_PREFIX:-}}"
    local candidate

    if [ -n "${configured}" ]; then
        if [ -x "${configured}" ] && [ ! -d "${configured}" ]; then
            printf '%s\n' "${configured}"
            return 0
        fi
        if [ -x "${configured}/bin/fastddsgen" ]; then
            printf '%s\n' "${configured}/bin/fastddsgen"
            return 0
        fi
    fi

    for candidate in /opt/fast-dds-gen/bin/fastddsgen
    do
        if [ -x "${candidate}" ]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    if command -v fastddsgen >/dev/null 2>&1; then
        command -v fastddsgen
        return 0
    fi

    return 1
}

generate_adaptive_autosar_artifacts() {
    local manifest_input mapping_input output_dir output_arxml output_mapping output_manifest
    local output_proxy_dir output_proxy_header apps_root
    local mapping_generator_cmd proxy_skeleton_generator_cmd proxy_skeleton_patcher
    local generator=""
    local event_binding
    manifest_input="${AUTOSAR_ARXML_MANIFEST_YAML:-}"
    mapping_input="${AUTOSAR_TOPIC_MAPPING_YAML:-}"
    apps_root="${AUTOSAR_APP_SOURCE_ROOT:-${SCRIPT_DIR}/apps}"
    event_binding="${AUTOSAR_EVENT_BINDING:-auto}"
    mapping_generator_cmd="${AUTOSAR_COMM_MANIFEST_GENERATOR:-autosar-generate-comm-manifest}"
    proxy_skeleton_generator_cmd="${AUTOSAR_PROXY_SKELETON_GENERATOR:-autosar-generate-proxy-skeleton}"
    output_dir="${BUILD_DIR}/autosar"
    output_arxml="${output_dir}/lwrcl_autosar_manifest.arxml"
    output_mapping="${output_dir}/lwrcl_autosar_topic_mapping.yaml"
    output_manifest="${output_dir}/lwrcl_autosar_manifest.yaml"
    output_proxy_dir="${output_dir}/generated"
    output_proxy_header="${output_proxy_dir}/lwrcl_autosar_proxy_skeleton.hpp"

    mkdir -p "${output_dir}"

    if [ -n "${manifest_input}" ] && [ -f "${manifest_input}" ] && [ -n "${mapping_input}" ] && [ -f "${mapping_input}" ]; then
        cp "${mapping_input}" "${output_mapping}"
        cp "${manifest_input}" "${output_manifest}"
    else
        if ! command -v "${mapping_generator_cmd}" >/dev/null 2>&1; then
            echo "Adaptive AUTOSAR mapping generator command not found: ${mapping_generator_cmd}"
            echo "Install codegen tools from Adaptive-AUTOSAR and ensure PATH contains /opt/autosar-ap/bin."
            exit 1
        fi
        "${mapping_generator_cmd}" \
          --apps-root "${apps_root}" \
          --output-mapping "${output_mapping}" \
          --output-manifest "${output_manifest}" \
          --event-binding "${event_binding}" \
          --print-summary
    fi

    if ! command -v "${proxy_skeleton_generator_cmd}" >/dev/null 2>&1; then
        echo "Adaptive AUTOSAR proxy/skeleton generator command not found: ${proxy_skeleton_generator_cmd}"
        echo "Install codegen tools from Adaptive-AUTOSAR and ensure PATH contains /opt/autosar-ap/bin."
        exit 1
    fi
    mkdir -p "${output_proxy_dir}"
    "${proxy_skeleton_generator_cmd}" \
      --mapping "${output_mapping}" \
      --output "${output_proxy_header}" \
      --namespace "autosar_generated" \
      --print-summary
    # No post-generation patch required; modern generator includes runtime-name handling.

    if [ "${AUTOSAR_SKIP_ARXML_GEN:-0}" = "1" ]; then
        echo "Skipping ARXML generation (AUTOSAR_SKIP_ARXML_GEN=1)."
        return
    fi

    if [ -n "${AUTOSAR_ARXML_GENERATOR:-}" ] && [ -f "${AUTOSAR_ARXML_GENERATOR}" ]; then
        generator="${AUTOSAR_ARXML_GENERATOR}"
    elif [ -f "${AUTOSAR_AP_PREFIX}/tools/arxml_generator/generate_arxml.py" ]; then
        generator="${AUTOSAR_AP_PREFIX}/tools/arxml_generator/generate_arxml.py"
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

install_adaptive_autosar_artifacts() {
    local output_dir install_dir
    output_dir="${BUILD_DIR}/autosar"
    install_dir="${LWRCL_PREFIX}/share/lwrcl/autosar"
    ensure_dir "${install_dir}"

    if [ -f "${output_dir}/lwrcl_autosar_manifest.arxml" ]; then
        copy_file "${output_dir}/lwrcl_autosar_manifest.arxml" "${install_dir}/"
    fi
    if [ -f "${output_dir}/lwrcl_autosar_topic_mapping.yaml" ]; then
        copy_file "${output_dir}/lwrcl_autosar_topic_mapping.yaml" "${install_dir}/"
    fi
    if [ -f "${output_dir}/lwrcl_autosar_manifest.yaml" ]; then
        copy_file "${output_dir}/lwrcl_autosar_manifest.yaml" "${install_dir}/"
    fi
    # Install the generated proxy/skeleton header to share/ (NOT include/).
    # The lwrcl library needs it for template definitions, but apps override it
    # with their own generated header via -I<app-gen-dir> at compile time.
    local gen_install_dir="${install_dir}/generated"
    ensure_dir "${gen_install_dir}"
    if [ -f "${output_dir}/generated/lwrcl_autosar_proxy_skeleton.hpp" ]; then
        copy_file "${output_dir}/generated/lwrcl_autosar_proxy_skeleton.hpp" "${gen_install_dir}/"
    fi
}

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="${DDS_PREFIX:-/opt/fast-dds}"
    FASTDDS_GEN_PREFIX="${FASTDDS_GEN_PREFIX:-/opt/fast-dds-gen}"
    LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/fast-dds-libs}"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
    LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/cyclonedds-libs}"
elif [ "$BACKEND" = "vsomeip" ]; then
    # vsomeip compiles data types against the standalone CDR library.
    # CycloneDDS is needed ONLY for the idlc code generator tool.
    # The stub CMake configs at VSOMEIP_PREFIX provide CycloneDDS::ddsc
    # and CycloneDDS-CXX::ddscxx targets pointing to liblwrcl_cdr.a.
    IDLC_PREFIX="${IDLC_PREFIX:-/opt/cyclonedds}"
    VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"
    LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/vsomeip-libs}"
    # Override backend to cyclonedds for data type generation pipeline
    BACKEND="cyclonedds"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    # Adaptive AUTOSAR wraps CycloneDDS internally, so data types are
    # identical to CycloneDDS-generated types linked against real CycloneDDS.
    DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
    LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/autosar-ap-libs}"
    AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar-ap}"
    # Override backend to cyclonedds for data type generation pipeline
    BACKEND="cyclonedds"
else
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/data_types/build-${BACKEND}"

# For vsomeip/adaptive-autosar, override build dir to use backend-specific path
if [ "${ORIGINAL_BACKEND}" = "vsomeip" ]; then
    BUILD_DIR="${SCRIPT_DIR}/data_types/build-vsomeip"
elif [ "${ORIGINAL_BACKEND}" = "adaptive-autosar" ]; then
    BUILD_DIR="${SCRIPT_DIR}/data_types/build-adaptive-autosar"
fi

if [ "$ACTION" = "clean" ]; then
    rm -rf "$BUILD_DIR"
    echo "Cleaned $BUILD_DIR"
    exit 0
fi

ensure_dir "$LWRCL_PREFIX"

if [ -n "${DDS_PREFIX:-}" ]; then
    export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${DDS_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
fi

if [ "${ORIGINAL_BACKEND}" = "adaptive-autosar" ] && [ -d "${AUTOSAR_AP_PREFIX}/bin" ]; then
    export PATH="${AUTOSAR_AP_PREFIX}/bin:${PATH}"
fi

# Add iceoryx libraries if present (used by CycloneDDS SHM/zero-copy)
ICEORYX_PREFIX_RT="${ICEORYX_PREFIX:-/opt/iceoryx}"
if [ -d "${ICEORYX_PREFIX_RT}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX_RT}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${ICEORYX_PREFIX_RT}/lib:${DYLD_LIBRARY_PATH:-}"
fi
if [ -n "${BREW_PREFIX}" ]; then
    export PATH="${BREW_PREFIX}/opt/bison/bin:${BREW_PREFIX}/opt/flex/bin:${PATH}"
fi

if [ "${ORIGINAL_BACKEND}" = "adaptive-autosar" ]; then
    generate_adaptive_autosar_artifacts
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/data_types"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
)

if [ "$(uname -s)" = "Darwin" ]; then
    CMAKE_ARGS+=(
        -DCMAKE_MACOSX_RPATH=ON
        -DCMAKE_INSTALL_RPATH="${LWRCL_PREFIX}/lib;${DDS_PREFIX:-}/lib;${VSOMEIP_PREFIX:-}/lib"
    )
fi

if [ "$BACKEND" = "fastdds" ]; then
    FASTDDSGEN_EXECUTABLE="$(resolve_fastddsgen || true)"
    if [ -z "${FASTDDSGEN_EXECUTABLE}" ]; then
        echo "fastddsgen not found. Install Fast DDS Gen or set FASTDDS_GEN_PREFIX to its install prefix, or FASTDDS_GEN_BIN to the fastddsgen executable."
        echo "Checked: ${FASTDDS_GEN_PREFIX:-/opt/fast-dds-gen}/bin/fastddsgen, /opt/fast-dds-gen/bin/fastddsgen, and PATH."
        exit 1
    fi
    export PATH="$(dirname "${FASTDDSGEN_EXECUTABLE}"):${PATH}"
    echo "Using fastddsgen: ${FASTDDSGEN_EXECUTABLE}"
    CMAKE_ARGS+=(
        -DCMAKE_SYSTEM_PREFIX_PATH="$DDS_PREFIX"
        -DCMAKE_PREFIX_PATH="$DDS_PREFIX"
        -Dfastcdr_DIR="${DDS_PREFIX}/lib/cmake/fastcdr/"
        -Dfastrtps_DIR="${DDS_PREFIX}/share/fastrtps/cmake/"
        -Dfoonathan_memory_DIR="${DDS_PREFIX}/lib/foonathan_memory/cmake/"
        -Dtinyxml2_DIR="${DDS_PREFIX}/lib/cmake/tinyxml2/"
    )
elif [ "$BACKEND" = "cyclonedds" ]; then
    if [ "${ORIGINAL_BACKEND}" = "vsomeip" ]; then
        # vsomeip: use CDR stub configs for CycloneDDS targets,
        # but still need idlc from CycloneDDS for code generation
        export PATH="${IDLC_PREFIX}/bin:${PATH}"
        CMAKE_ARGS+=(
            -DCMAKE_PREFIX_PATH="${VSOMEIP_PREFIX}/lib/cmake"
            -DCycloneDDS_DIR="${VSOMEIP_PREFIX}/lib/cmake/CycloneDDS"
            -DCycloneDDS-CXX_DIR="${VSOMEIP_PREFIX}/lib/cmake/CycloneDDS-CXX"
        )
    else
        export PATH="${DDS_PREFIX}/bin:${PATH}"
        # Include iceoryx cmake configs if present (CycloneDDS depends on it)
        ICEORYX_CMAKE="${ICEORYX_PREFIX_RT}/lib/cmake"
        if [ -d "${ICEORYX_CMAKE}" ]; then
            CMAKE_ARGS+=(
                -DICEORYX_PREFIX="${ICEORYX_PREFIX_RT}"
                -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${ICEORYX_CMAKE}"
            )
        else
            CMAKE_ARGS+=(
                -DICEORYX_PREFIX="${ICEORYX_PREFIX_RT}"
                -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
            )
        fi
    fi
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    cmake_install "$BUILD_DIR" "$LWRCL_PREFIX"
    if [ "${ORIGINAL_BACKEND}" = "adaptive-autosar" ]; then
        install_adaptive_autosar_artifacts
    fi
fi
