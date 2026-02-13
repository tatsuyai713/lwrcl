#!/bin/bash
set -euo pipefail

##############################################################################
# build_all.sh — Build libraries, data_types, and lwrcl in one shot
#
# Usage:
#   ./build_all.sh <fastdds|cyclonedds|vsomeip> [install|clean]
#
# Examples:
#   ./build_all.sh fastdds install      # Build & install everything (FastDDS)
#   ./build_all.sh cyclonedds install   # Build & install everything (CycloneDDS)
#   ./build_all.sh vsomeip install      # Build & install everything (vsomeip)
#   ./build_all.sh fastdds              # Build only (no install)
#   ./build_all.sh cyclonedds clean     # Clean all build directories
##############################################################################

BACKEND="${1:-}"
ACTION="${2:-}"

if [ "$BACKEND" != "fastdds" ] && [ "$BACKEND" != "cyclonedds" ] && [ "$BACKEND" != "vsomeip" ]; then
    echo "Usage: $0 <fastdds|cyclonedds|vsomeip> [install|clean]"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build order: libraries → data_types → lwrcl
STEPS=("build_libraries.sh" "build_data_types.sh" "build_lwrcl.sh")
LABELS=("libraries" "data_types" "lwrcl")

FAILED=0
for i in "${!STEPS[@]}"; do
    LABEL="${LABELS[$i]}"
    SCRIPT="${SCRIPT_DIR}/${STEPS[$i]}"

    echo "========================================"
    echo "  [$(( i + 1 ))/${#STEPS[@]}] ${LABEL}  (${BACKEND}${ACTION:+ $ACTION})"
    echo "========================================"

    if ! bash "$SCRIPT" "$BACKEND" "$ACTION"; then
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
