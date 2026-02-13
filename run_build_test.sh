#!/bin/bash
# ===========================================================================
# run_build_test.sh – Build all lwrcl components and verify compilation
# ===========================================================================
# Usage:
#   ./run_build_test.sh                  # Build all backends (full)
#   ./run_build_test.sh fastdds          # Build only FastDDS backend
#   ./run_build_test.sh cyclonedds       # Build only CycloneDDS backend
#   ./run_build_test.sh vsomeip          # Build only vsomeip backend
#   ./run_build_test.sh --clean          # Clean all build dirs first
#   ./run_build_test.sh fastdds --clean  # Clean then build FastDDS only
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

BACKENDS=()
CLEAN=false

# Parse args
for arg in "$@"; do
    case "$arg" in
        fastdds)     BACKENDS+=(fastdds) ;;
        cyclonedds)  BACKENDS+=(cyclonedds) ;;
        vsomeip)     BACKENDS+=(vsomeip) ;;
        --clean)     CLEAN=true ;;
        *)           echo "Unknown arg: $arg"; exit 1 ;;
    esac
done

# Default: both backends
if [ ${#BACKENDS[@]} -eq 0 ]; then
    BACKENDS=(fastdds cyclonedds vsomeip)
fi

PASS=0
FAIL=0
SKIP=0
RESULTS=()

# ---------------------------------------------------------------------------
log_step() {
    echo -e "\n${CYAN}========================================${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

log_ok() {
    echo -e "  ${GREEN}✓ $1${NC}"
    RESULTS+=("${GREEN}✓${NC} $1")
    ((PASS++))
}

log_fail() {
    echo -e "  ${RED}✗ $1${NC}"
    RESULTS+=("${RED}✗${NC} $1")
    ((FAIL++))
}

log_skip() {
    echo -e "  ${YELLOW}⊘ $1 (skipped)${NC}"
    RESULTS+=("${YELLOW}⊘${NC} $1 (skipped)")
    ((SKIP++))
}

# ---------------------------------------------------------------------------
check_dds_installed() {
    local backend="$1"
    if [ "$backend" = "fastdds" ]; then
        [ -d "/opt/fast-dds" ]
    elif [ "$backend" = "cyclonedds" ]; then
        [ -d "/opt/cyclonedds" ]
    elif [ "$backend" = "vsomeip" ]; then
        [ -d "/opt/vsomeip" ] && [ -d "/opt/cyclonedds" ]
    fi
}

# ---------------------------------------------------------------------------
run_build_step() {
    local label="$1"
    shift
    local logfile
    logfile=$(mktemp /tmp/lwrcl_build_XXXXXX.log)

    if "$@" > "$logfile" 2>&1; then
        log_ok "$label"
        rm -f "$logfile"
        return 0
    else
        log_fail "$label"
        echo -e "    ${RED}Build log (last 30 lines):${NC}"
        tail -30 "$logfile" | sed 's/^/    /'
        rm -f "$logfile"
        return 1
    fi
}

# ---------------------------------------------------------------------------
build_backend() {
    local backend="$1"

    log_step "Building backend: ${backend}"

    if ! check_dds_installed "$backend"; then
        log_skip "[${backend}] DDS libraries not installed"
        return
    fi

    # Clean if requested
    if [ "$CLEAN" = true ]; then
        echo "  Cleaning build directories..."
        ./build_libraries.sh "$backend" clean 2>/dev/null || true
        ./build_data_types.sh "$backend" clean 2>/dev/null || true
        ./build_lwrcl.sh "$backend" clean 2>/dev/null || true
        ./build_apps.sh "$backend" clean 2>/dev/null || true
        rm -rf "apps/build-advanced-${backend}" 2>/dev/null || true
    fi

    # Step 1: Libraries (yaml-cpp etc.)
    run_build_step "[${backend}] libraries" \
        ./build_libraries.sh "$backend" install || true

    # Step 2: Data types (IDL generated message types)
    run_build_step "[${backend}] data_types" \
        ./build_data_types.sh "$backend" install || true

    # Step 3: lwrcl core
    run_build_step "[${backend}] lwrcl core" \
        ./build_lwrcl.sh "$backend" install || true

    # Step 4: Standard apps
    run_build_step "[${backend}] apps (standard)" \
        ./build_apps.sh "$backend" || true

    # Step 5: Advanced apps
    if [ -f "apps/advanced/build_and_run_advanced.sh" ]; then
        run_build_step "[${backend}] apps (advanced)" \
            ./apps/advanced/build_and_run_advanced.sh "$backend" build || true
    else
        log_skip "[${backend}] apps (advanced) - script not found"
    fi
}

# ===========================================================================
# Main
# ===========================================================================
echo -e "${CYAN}"
echo "  ╔════════════════════════════════════════╗"
echo "  ║     lwrcl Build Verification Test      ║"
echo "  ╚════════════════════════════════════════╝"
echo -e "${NC}"
echo "  Backends: ${BACKENDS[*]}"
echo "  Clean:    ${CLEAN}"
echo ""

START_TIME=$(date +%s)

for backend in "${BACKENDS[@]}"; do
    build_backend "$backend"
done

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

# ===========================================================================
# Summary
# ===========================================================================
echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Build Results Summary${NC}"
echo -e "${CYAN}========================================${NC}"
for r in "${RESULTS[@]}"; do
    echo -e "  $r"
done
echo ""
echo -e "  Passed:  ${GREEN}${PASS}${NC}"
echo -e "  Failed:  ${RED}${FAIL}${NC}"
echo -e "  Skipped: ${YELLOW}${SKIP}${NC}"
echo -e "  Time:    ${ELAPSED}s"
echo ""

if [ "$FAIL" -gt 0 ]; then
    echo -e "${RED}  BUILD FAILED${NC}"
    exit 1
else
    echo -e "${GREEN}  ALL BUILDS PASSED${NC}"
    exit 0
fi
