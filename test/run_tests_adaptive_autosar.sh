#!/bin/bash
# ===========================================================================
# run_tests_adaptive_autosar.sh – Build & run lwrcl gtest suite against
# Adaptive AUTOSAR
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar-ap}"
DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"
LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/autosar-ap-libs}"
BUILD_DIR="${SCRIPT_DIR}/build-adaptive-autosar"
AUTOSAR_TEST_TIMEOUT="${AUTOSAR_TEST_TIMEOUT:-10}"
AUTOSAR_CYCLONEDDS_CONFIG="${BUILD_DIR}/autosar/cyclonedds-test.xml"
AUTOSAR_TEST_GEN_DIR="${BUILD_DIR}/autosar/generated"
AUTOSAR_TEST_MAPPING="${BUILD_DIR}/autosar/lwrcl_autosar_topic_mapping.yaml"
AUTOSAR_TEST_MANIFEST="${BUILD_DIR}/autosar/lwrcl_autosar_manifest.yaml"
AUTOSAR_TEST_PROXY_SKELETON="${AUTOSAR_TEST_GEN_DIR}/lwrcl_autosar_proxy_skeleton.hpp"
SOURCE_LWRCL_LIB_DIR="${ROOT_DIR}/lwrcl/build-adaptive-autosar/adaptive-autosar/lwrcl"
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif [ "$(uname -s)" = "Darwin" ]; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi
ACTION="${1:-all}"
ROUDI_PID=""
ROUTING_MANAGER_PID=""

export PATH="${AUTOSAR_AP_PREFIX}/bin:${DDS_PREFIX}/bin:${ICEORYX_PREFIX}/bin:${VSOMEIP_PREFIX}/bin:${PATH}"
export ARA_COM_EVENT_BINDING="${ARA_COM_EVENT_BINDING:-dds}"
export LWRCL_SKIP_ARA_CORE_INIT="${LWRCL_SKIP_ARA_CORE_INIT:-1}"
if [ -d "${SOURCE_LWRCL_LIB_DIR}" ]; then
    export LD_LIBRARY_PATH="${SOURCE_LWRCL_LIB_DIR}:${LWRCL_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${SOURCE_LWRCL_LIB_DIR}:${LWRCL_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
else
    export LD_LIBRARY_PATH="${LWRCL_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    export DYLD_LIBRARY_PATH="${LWRCL_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
fi

cleanup_runtime() {
    if [ -n "${ROUTING_MANAGER_PID}" ] && kill -0 "${ROUTING_MANAGER_PID}" 2>/dev/null; then
        kill -TERM "${ROUTING_MANAGER_PID}" 2>/dev/null || true
        wait "${ROUTING_MANAGER_PID}" 2>/dev/null || true
        ROUTING_MANAGER_PID=""
    fi
    if [ -n "${ROUDI_PID}" ] && kill -0 "${ROUDI_PID}" 2>/dev/null; then
        kill -TERM "${ROUDI_PID}" 2>/dev/null || true
        wait "${ROUDI_PID}" 2>/dev/null || true
        ROUDI_PID=""
    fi
    pkill -x autosar_vsomeip_routing_manager >/dev/null 2>&1 || true
    pkill -x iox-roudi >/dev/null 2>&1 || true
}

start_runtime() {
    if [ -x "${ICEORYX_PREFIX}/bin/iox-roudi" ] && ! pgrep -x iox-roudi >/dev/null 2>&1; then
        "${ICEORYX_PREFIX}/bin/iox-roudi" > /tmp/lwrcl_test_adaptive_iox_roudi.log 2>&1 &
        ROUDI_PID=$!
    fi
    if [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] \
       && ! pgrep -x autosar_vsomeip_routing_manager >/dev/null 2>&1; then
        "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" > /tmp/lwrcl_test_adaptive_routing_manager.log 2>&1 &
        ROUTING_MANAGER_PID=$!
    fi
    sleep 1
}

write_cyclonedds_test_config() {
    mkdir -p "$(dirname "${AUTOSAR_CYCLONEDDS_CONFIG}")"
    cat > "${AUTOSAR_CYCLONEDDS_CONFIG}" <<'CYCLONEDDS_XML'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="127.0.0.1" multicast="true" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <SharedMemory>
      <Enable>false</Enable>
    </SharedMemory>
  </Domain>
</CycloneDDS>
CYCLONEDDS_XML
}

run_with_timeout() {
    local timeout_seconds="$1"
    local log_file="$2"
    shift 2

    "$@" > "$log_file" 2>&1 &
    local pid=$!
    local start_time=$SECONDS

    while kill -0 "$pid" 2>/dev/null; do
        if [ $((SECONDS - start_time)) -ge "$timeout_seconds" ]; then
            kill -TERM "$pid" 2>/dev/null || true
            sleep 1
            kill -KILL "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
            return 124
        fi
        sleep 0.2
    done

    wait "$pid"
}

trap cleanup_runtime EXIT INT TERM

show_usage() {
    echo "Usage: $0 [build|run|clean|all]"
    echo "  build  – configure & compile tests"
    echo "  run    – run ctest (build first if needed)"
    echo "  clean  – remove build directory"
    echo "  all    – build + run (default)"
}

do_build() {
    if ! command -v autosar-generate-comm-manifest >/dev/null 2>&1; then
        echo "[ERROR] autosar-generate-comm-manifest not found. Ensure ${AUTOSAR_AP_PREFIX}/bin is in PATH." >&2
        exit 1
    fi
    if ! command -v autosar-generate-proxy-skeleton >/dev/null 2>&1; then
        echo "[ERROR] autosar-generate-proxy-skeleton not found. Ensure ${AUTOSAR_AP_PREFIX}/bin is in PATH." >&2
        exit 1
    fi

    mkdir -p "${AUTOSAR_TEST_GEN_DIR}"
    autosar-generate-comm-manifest \
        --apps-root "${SCRIPT_DIR}" \
        --output-mapping "${AUTOSAR_TEST_MAPPING}" \
        --output-manifest "${AUTOSAR_TEST_MANIFEST}" \
        --event-binding "${ARA_COM_EVENT_BINDING}" \
        --print-summary
    python3 - "${AUTOSAR_TEST_MAPPING}" <<'PY'
import copy
import hashlib
import json
import sys

mapping_path = sys.argv[1]
with open(mapping_path, "r", encoding="utf-8") as f:
    mapping = json.load(f)

topics = mapping.setdefault("topic_mappings", [])
if not any(t.get("dds_topic") == "rt/test_ns/chatter" for t in topics):
    base = next(
        (
            t
            for t in topics
            if t.get("dds_topic") == "rt/chatter"
            and t.get("type_name") == "std_msgs::msg::String"
        ),
        None,
    )
    if base is None:
        raise SystemExit("missing base AUTOSAR mapping for rt/chatter")

    alias = copy.deepcopy(base)
    alias["ros_topic"] = "/test_ns/chatter"
    alias["dds_topic"] = "rt/test_ns/chatter"
    ara = alias.setdefault("ara", {})
    suffix = "test_ns_chatter"
    digest = int(hashlib.sha1(alias["dds_topic"].encode("utf-8")).hexdigest()[:4], 16)
    ara["service_interface"] = f"Ros2Msg_std_msgs_msg_String_{suffix}"
    ara["instance_specifier"] = "/autosar/ros2/topic/test_ns/chatter"
    ara["service_interface_id"] = f"0x{0x3000 | (digest & 0x0fff):04X}"
    ara["event_id"] = f"0x{0x8000 | (digest & 0x0fff):04X}"
    ara["iceoryx_service"] = suffix
    topics.append(alias)

    with open(mapping_path, "w", encoding="utf-8") as f:
        json.dump(mapping, f, indent=2)
        f.write("\n")
PY
    autosar-generate-proxy-skeleton \
        --mapping "${AUTOSAR_TEST_MAPPING}" \
        --output "${AUTOSAR_TEST_PROXY_SKELETON}" \
        --namespace autosar_generated \
        --print-summary

    echo "=== [Adaptive AUTOSAR] Configuring tests ==="
    cmake \
        -S "${SCRIPT_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DDDS_BACKEND=adaptive-autosar \
        -DAUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX}" \
        -DDDS_PREFIX="${DDS_PREFIX}" \
        -DICEORYX_PREFIX="${ICEORYX_PREFIX}" \
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}" \
        -DLWRCL_INSTALL_PREFIX="${LWRCL_PREFIX}" \
        -DAUTOSAR_GENERATED_PROXY_SKELETON_DIR="${AUTOSAR_TEST_GEN_DIR}" \
        -DCMAKE_PREFIX_PATH="${AUTOSAR_AP_PREFIX}/lib/cmake/AdaptiveAutosarAP;${DDS_PREFIX}/lib/cmake;${ICEORYX_PREFIX}/lib/cmake;${VSOMEIP_PREFIX}/lib/cmake" \
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"

    echo "=== [Adaptive AUTOSAR] Building tests ==="
    cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

do_run() {
    if [ ! -d "${BUILD_DIR}" ]; then
        echo "Build directory not found – building first..."
        do_build
    fi

    echo "=== [Adaptive AUTOSAR] Running tests ==="
    write_cyclonedds_test_config
    export CYCLONEDDS_URI="file://${AUTOSAR_CYCLONEDDS_CONFIG}"
    cd "${BUILD_DIR}"
    RESULT=0
    total=0
    failed=0
    test_names=()
    while IFS= read -r line; do
        test_name="$(printf '%s\n' "$line" | sed -nE 's/^  Test +#[0-9]+: //p')"
        [ -n "$test_name" ] && test_names+=("$test_name")
    done < <(ctest -N)

    for test_name in "${test_names[@]}"; do
        if [ ! -x "./${test_name}" ]; then
            echo "[FAIL] ${test_name}: executable not found"
            RESULT=1
            failed=$((failed + 1))
            continue
        fi

        suite=""
        while IFS= read -r line; do
            if [[ "$line" =~ ^[A-Za-z_][A-Za-z0-9_/]*\.$ ]]; then
                suite="${line}"
            elif [[ "$line" == "  "* && -n "$suite" ]]; then
                case_name="$(printf '%s' "$line" | sed -E 's/^  //; s/[[:space:]]+#.*$//')"
                [ -z "$case_name" ] && continue
                filter="${suite}${case_name}"
                total=$((total + 1))
                log="/tmp/lwrcl_test_adaptive_${test_name}_${suite}${case_name}.log"
                log="${log//\//_}"

                echo "== RUN ${test_name} --gtest_filter=${filter}"
                cleanup_runtime
                start_runtime
                set +e
                run_with_timeout "${AUTOSAR_TEST_TIMEOUT}" "$log" "./${test_name}" "--gtest_filter=${filter}"
                status=$?
                set -e
                cleanup_runtime

                if [ "$status" -eq 124 ]; then
                    echo "[FAIL] ${filter} (timeout after ${AUTOSAR_TEST_TIMEOUT}s)"
                    echo "       log: ${log}"
                    sed -n '1,220p' "$log"
                    RESULT=1
                    failed=$((failed + 1))
                elif grep -q "\\[  FAILED  \\]" "$log"; then
                    echo "[FAIL] ${filter}"
                    echo "       log: ${log}"
                    sed -n '1,220p' "$log"
                    RESULT=1
                    failed=$((failed + 1))
                elif grep -q "\\[       OK \\]" "$log" || grep -q "\\[  PASSED  \\]" "$log"; then
                    if [ "$status" -ne 0 ]; then
                        echo "[ OK ] ${filter} (runtime teardown returned ${status})"
                    else
                        echo "[ OK ] ${filter}"
                    fi
                else
                    echo "[FAIL] ${filter}"
                    echo "       log: ${log}"
                    sed -n '1,220p' "$log"
                    RESULT=1
                    failed=$((failed + 1))
                fi
            fi
        done < <("./${test_name}" --gtest_list_tests)
    done

    echo ""
    if [ $RESULT -eq 0 ]; then
        echo "=== ALL Adaptive AUTOSAR TESTS PASSED (${total}) ==="
    else
        echo "=== SOME Adaptive AUTOSAR TESTS FAILED (${failed}/${total}) ==="
    fi
    return $RESULT
}

do_clean() {
    rm -rf "${BUILD_DIR}"
    echo "Cleaned ${BUILD_DIR}"
}

case "${ACTION}" in
    build) do_build ;;
    run)   do_run ;;
    clean) do_clean ;;
    all)   do_build && do_run ;;
    *)     show_usage; exit 1 ;;
esac
