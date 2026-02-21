#!/bin/bash
set -euo pipefail

BACKEND="${1:-}"
ACTION="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || echo 4)

if [ "$BACKEND" = "fastdds" ]; then
    DDS_PREFIX="/opt/fast-dds"
    FASTDDS_GEN_PREFIX="${FASTDDS_GEN_PREFIX:-/opt/fast-dds-gen}"
    LWRCL_PREFIX="/opt/fast-dds-libs"
elif [ "$BACKEND" = "cyclonedds" ]; then
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/cyclonedds-libs"
elif [ "$BACKEND" = "vsomeip" ]; then
    # vsomeip compiles data types against the standalone CDR library.
    # CycloneDDS is needed ONLY for the idlc code generator tool.
    # The stub CMake configs at VSOMEIP_PREFIX provide CycloneDDS::ddsc
    # and CycloneDDS-CXX::ddscxx targets pointing to liblwrcl_cdr.a.
    IDLC_PREFIX="${IDLC_PREFIX:-/opt/cyclonedds}"
    VSOMEIP_PREFIX="/opt/vsomeip"
    LWRCL_PREFIX="/opt/vsomeip-libs"
    # Override backend to cyclonedds for data type generation pipeline
    BACKEND="cyclonedds"
elif [ "$BACKEND" = "adaptive-autosar" ]; then
    # Adaptive AUTOSAR wraps CycloneDDS internally, so data types are
    # identical to CycloneDDS-generated types linked against real CycloneDDS.
    DDS_PREFIX="/opt/cyclonedds"
    LWRCL_PREFIX="/opt/autosar-ap-libs"
    # Override backend to cyclonedds for data type generation pipeline
    BACKEND="cyclonedds"
else
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]"
    exit 1
fi

BUILD_DIR="${SCRIPT_DIR}/data_types/build-${BACKEND}"

# For vsomeip/adaptive-autosar, override build dir to use backend-specific path
ORIGINAL_BACKEND="${1:-}"
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

sudo mkdir -p "$LWRCL_PREFIX"

if [ -n "${DDS_PREFIX:-}" ]; then
    export LD_LIBRARY_PATH="${DDS_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi

# Add iceoryx libraries if present (used by CycloneDDS SHM/zero-copy)
ICEORYX_PREFIX_RT="${ICEORYX_PREFIX:-/opt/iceoryx}"
if [ -d "${ICEORYX_PREFIX_RT}/lib" ]; then
    export LD_LIBRARY_PATH="${ICEORYX_PREFIX_RT}/lib:${LD_LIBRARY_PATH:-}"
fi

CMAKE_ARGS=(
    -S "${SCRIPT_DIR}/data_types"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Debug
    -DDDS_BACKEND="$BACKEND"
    -DCMAKE_INSTALL_PREFIX="$LWRCL_PREFIX"
)

if [ "$BACKEND" = "fastdds" ]; then
    export PATH="${FASTDDS_GEN_PREFIX}/bin:${PATH}"
    if ! command -v fastddsgen >/dev/null 2>&1; then
        echo "fastddsgen not found. Set FASTDDS_GEN_PREFIX or install Fast DDS Gen (expected: ${FASTDDS_GEN_PREFIX}/bin/fastddsgen)."
        exit 1
    fi
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
                -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${ICEORYX_CMAKE}"
            )
        else
            CMAKE_ARGS+=(
                -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake"
            )
        fi
    fi
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j "$JOBS"

if [ "$ACTION" = "install" ]; then
    sudo cmake --install "$BUILD_DIR" --prefix "$LWRCL_PREFIX"
fi
