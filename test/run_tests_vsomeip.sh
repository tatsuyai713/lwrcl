#!/bin/bash
# ===========================================================================
# run_tests_vsomeip.sh – Build & run lwrcl gtest suite against vsomeip
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"
DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/vsomeip-libs}"
BUILD_DIR="${SCRIPT_DIR}/build-vsomeip"
if command -v nproc >/dev/null 2>&1; then
    JOBS=$(nproc)
elif [ "$(uname -s)" = "Darwin" ]; then
    JOBS=$(sysctl -n hw.ncpu)
else
    JOBS=4
fi
ACTION="${1:-all}"
CTEST_JOBS="${CTEST_JOBS:-1}"
ROUTING_MANAGER_PID=""
if [ "$(uname -s)" = "Darwin" ]; then
    VSOMEIP_RESTART_DELAY="${VSOMEIP_RESTART_DELAY:-0.2}"
else
    VSOMEIP_RESTART_DELAY="${VSOMEIP_RESTART_DELAY:-0.2}"
fi

export LD_LIBRARY_PATH="${VSOMEIP_PREFIX}/lib:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export DYLD_LIBRARY_PATH="${VSOMEIP_PREFIX}/lib:${DDS_PREFIX}/lib:${LWRCL_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
export PATH="${VSOMEIP_PREFIX}/bin:${DDS_PREFIX}/bin:${PATH}"
export VSOMEIP_CONFIGURATION="${SCRIPT_DIR}/vsomeip-test.json"

# ---------------------------------------------------------------------------
cleanup_runtime() {
    if [ -n "${ROUTING_MANAGER_PID}" ] && kill -0 "${ROUTING_MANAGER_PID}" 2>/dev/null; then
        kill -TERM "${ROUTING_MANAGER_PID}" 2>/dev/null || true
        wait "${ROUTING_MANAGER_PID}" 2>/dev/null || true
        ROUTING_MANAGER_PID=""
    fi
    pkill -x routingmanagerd >/dev/null 2>&1 || true
    rm -f /tmp/vsomeip-* >/dev/null 2>&1 || true
    rm -f /tmp/lwrcl_vsomeip_test_*.json >/dev/null 2>&1 || true
    sleep "${VSOMEIP_RESTART_DELAY}"
}

write_runtime_config() {
    local routing_port="$1"
    local sd_port="$2"
    local guest_first_port="$3"
    local guest_last_port="$4"
    local config_file="$5"

    cat > "${config_file}" <<EOF
{
    "unicast": "127.0.0.1",
    "logging": {
        "level": "info",
        "console": "true",
        "file": { "enable": "false" },
        "dlt": "false"
    },
    "applications": [
        { "name": "routingmanagerd", "id": "0x0001" }
    ],
    "services": [],
    "routing": {
        "host": {
            "name": "routingmanagerd",
            "unicast": "127.0.0.1",
            "port": "${routing_port}"
        },
        "guests": {
            "unicast": "127.0.0.1",
            "ports": {
                "lwrcl": {
                    "range": {
                        "first": "${guest_first_port}",
                        "last": "${guest_last_port}"
                    }
                }
            }
        }
    },
    "service-discovery": {
        "enable": "true",
        "multicast": "224.244.224.245",
        "port": "${sd_port}",
        "protocol": "udp",
        "initial_delay_min": "10",
        "initial_delay_max": "100",
        "repetitions_base_delay": "200",
        "repetitions_max": "3",
        "ttl": "3",
        "cyclic_offer_delay": "2000",
        "request_response_delay": "1500"
    }
}
EOF
}

start_routing_manager() {
    local case_index="$1"
    local routing_port=$((31490 + case_index * 20))
    local sd_port=$((30490 + case_index))
    local guest_first_port=$((routing_port + 2))
    local guest_last_port=$((routing_port + 19))
    local config_file="/tmp/lwrcl_vsomeip_test_${routing_port}.json"

    if [ ! -x "${VSOMEIP_PREFIX}/bin/routingmanagerd" ]; then
        echo "[ERROR] Missing routing manager: ${VSOMEIP_PREFIX}/bin/routingmanagerd" >&2
        echo "        Reinstall vsomeip with scripts/install_vsomeip.sh --force" >&2
        return 1
    fi

    cleanup_runtime
    write_runtime_config "${routing_port}" "${sd_port}" "${guest_first_port}" "${guest_last_port}" "${config_file}"
    export VSOMEIP_CONFIGURATION="${config_file}"
    "${VSOMEIP_PREFIX}/bin/routingmanagerd" > /tmp/lwrcl_test_vsomeip_routingmanagerd.log 2>&1 &
    ROUTING_MANAGER_PID=$!

    for _ in {1..100}; do
        if ! kill -0 "${ROUTING_MANAGER_PID}" 2>/dev/null; then
            echo "[ERROR] routingmanagerd failed to start. See /tmp/lwrcl_test_vsomeip_routingmanagerd.log" >&2
            return 1
        fi
        if grep -q "SOME/IP routing ready" /tmp/lwrcl_test_vsomeip_routingmanagerd.log 2>/dev/null; then
            return 0
        fi
        sleep 0.1
    done

    echo "[ERROR] routingmanagerd did not become ready. See /tmp/lwrcl_test_vsomeip_routingmanagerd.log" >&2
    return 1
}

trap cleanup_runtime EXIT INT TERM

# ---------------------------------------------------------------------------
show_usage() {
    echo "Usage: $0 [build|run|clean|all]"
    echo "  build  – configure & compile tests"
    echo "  run    – run ctest (build first if needed)"
    echo "  clean  – remove build directory"
    echo "  all    – build + run (default)"
}

do_build() {
    echo "=== [vsomeip] Configuring tests ==="
    cmake \
        -S "${SCRIPT_DIR}" \
        -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DDDS_BACKEND=vsomeip \
        -DCMAKE_PREFIX_PATH="${DDS_PREFIX}/lib/cmake;${VSOMEIP_PREFIX}/lib/cmake" \
        -DVSOMEIP_PREFIX="${VSOMEIP_PREFIX}" \
        -Dyaml-cpp_DIR="${LWRCL_PREFIX}/lib/cmake/yaml-cpp/"

    echo "=== [vsomeip] Building tests ==="
    cmake --build "${BUILD_DIR}" -j "${JOBS}"
}

do_run() {
    if [ ! -d "${BUILD_DIR}" ]; then
        echo "Build directory not found – building first..."
        do_build
    fi

    echo "=== [vsomeip] Running tests ==="
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
                log="/tmp/lwrcl_test_vsomeip_${test_name}_${suite}${case_name}.log"
                log="${log//\//_}"

                echo "== RUN ${test_name} --gtest_filter=${filter}"
                if start_routing_manager "${total}" && "./${test_name}" "--gtest_filter=${filter}" > "$log" 2>&1; then
                    echo "[ OK ] ${filter}"
                else
                    echo "[FAIL] ${filter}"
                    echo "       log: ${log}"
                    sed -n '1,220p' "$log"
                    RESULT=1
                    failed=$((failed + 1))
                fi
                cleanup_runtime
            fi
        done < <("./${test_name}" --gtest_list_tests)
    done

    echo ""
    if [ $RESULT -eq 0 ]; then
        echo "=== ALL vsomeip TESTS PASSED (${total}) ==="
    else
        echo "=== SOME vsomeip TESTS FAILED (${failed}/${total}) ==="
    fi
    return $RESULT
}

do_clean() {
    rm -rf "${BUILD_DIR}"
    echo "Cleaned ${BUILD_DIR}"
}

# ---------------------------------------------------------------------------
case "${ACTION}" in
    build) do_build ;;
    run)   do_run ;;
    clean) do_clean ;;
    all)   do_build && do_run ;;
    *)     show_usage; exit 1 ;;
esac
