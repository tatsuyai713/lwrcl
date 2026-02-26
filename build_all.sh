#!/bin/bash
set -euo pipefail

##############################################################################
# build_all.sh — Build libraries, data_types, and lwrcl in one shot
#
# Usage:
#   ./build_all.sh <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]
#
# Examples:
#   ./build_all.sh fastdds install              # Build & install everything (FastDDS)
#   ./build_all.sh cyclonedds install             # Build & install everything (CycloneDDS)
#   ./build_all.sh vsomeip install                # Build & install everything (vsomeip)
#   ./build_all.sh adaptive-autosar install       # Build & install everything (Adaptive AUTOSAR)
#   ./build_all.sh fastdds                        # Build only (no install)
#   ./build_all.sh cyclonedds clean               # Clean all build directories
##############################################################################

BACKEND="${1:-}"
ACTION="${2:-}"

if [ "$BACKEND" != "fastdds" ] && [ "$BACKEND" != "cyclonedds" ] && [ "$BACKEND" != "vsomeip" ] && [ "$BACKEND" != "adaptive-autosar" ]; then
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip|adaptive-autosar> [install|clean]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build order: libraries → data_types → lwrcl → apps
# Each step must be installed before the next step can find its outputs,
# so always use "install" unless the user requested "clean".
if [ "$ACTION" = "clean" ]; then
    STEP_ACTION="clean"
else
    STEP_ACTION="install"
fi

STEPS=("build_libraries.sh" "build_data_types.sh" "build_lwrcl.sh" "build_apps.sh")
LABELS=("libraries" "data_types" "lwrcl" "apps")

FAILED=0
for i in "${!STEPS[@]}"; do
    LABEL="${LABELS[$i]}"
    SCRIPT="${SCRIPT_DIR}/${STEPS[$i]}"

    echo "========================================"
    echo "  [$(( i + 1 ))/${#STEPS[@]}] ${LABEL}  (${BACKEND}${ACTION:+ $ACTION})"
    echo "========================================"

    if ! bash "$SCRIPT" "$BACKEND" "$STEP_ACTION"; then
        echo "ERROR: ${LABEL} failed."
        FAILED=1
        break
    fi

    echo ""
done

if [ "$FAILED" -eq 0 ]; then
    echo "========================================"
    echo "  All steps completed successfully (${BACKEND}${ACTION:+ $ACTION})"
    echo "========================================"
else
    echo "========================================"
    echo "  Build FAILED at ${LABEL}"
    echo "========================================"
    exit 1
fi
