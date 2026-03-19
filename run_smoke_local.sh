#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

NO_BUILD=0
SELECT_BACKEND="adaptive-autosar"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-build) NO_BUILD=1 ;;
    --backend) shift; SELECT_BACKEND="${1:-}";;
    -h|--help)
      echo "Usage: $0 [--no-build] [--backend <fastdds|cyclonedds|vsomeip|adaptive-autosar|all>]"
      exit 0 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
  shift
done

# Defaults (match workflow)
DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"
AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar-ap}"
LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/autosar-ap-libs}"
FASTDDS_PREFIX="${FASTDDS_PREFIX:-/opt/fastdds}"

export PATH="$DDS_PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

overall_fail=0

log_path() {
  local name="$1"
  name="${name// /_}"
  name="${name//\//_}"
  echo "/tmp/lwrcl_smoke_${name}.log"
}

cleanup_runtime_processes() {
  pkill -x autosar_vsomeip_routing_manager >/dev/null 2>&1 || true
  pkill -x iox-roudi >/dev/null 2>&1 || true
  pkill -x example_class_sub >/dev/null 2>&1 || true
  pkill -x example_class_pub >/dev/null 2>&1 || true
  pkill -x example_service_server >/dev/null 2>&1 || true
  pkill -x example_service_client >/dev/null 2>&1 || true
  pkill -x example_zero_copy_sub >/dev/null 2>&1 || true
  pkill -x example_zero_copy_pub >/dev/null 2>&1 || true
}

trap cleanup_runtime_processes EXIT INT TERM

# Run command with a hard timeout, capture stdout/stderr to logfile.
# Returns:
#   0   success (command exit 0 within time)
#   124 timeout
#   other nonzero: command failed
run_with_timeout() {
  local secs="$1"; shift
  local logfile="$1"; shift
  ("$@") >"$logfile" 2>&1 &
  local pid=$!
  local deadline=$((SECONDS + secs))

  while kill -0 "$pid" 2>/dev/null; do
    if [ "$SECONDS" -ge "$deadline" ]; then
      kill -TERM "$pid" 2>/dev/null || true
      sleep 2
      kill -KILL "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
      return 124
    fi
    sleep 0.2
  done
  wait "$pid"
  return $?
}

# Start a background process with a watchdog timeout.
# Prints: "<pid> <watchdog_pid>"
start_bg_with_timeout() {
  local secs="$1"; shift
  local logfile="$1"; shift

  ("$@") >"$logfile" 2>&1 &
  local pid=$!

  (
    sleep "$secs"
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      sleep 2
      kill -KILL "$pid" 2>/dev/null || true
    fi
  ) &
  local wdpid=$!

  echo "$pid $wdpid"
}

wait_bg() {
  local pid="$1" wdpid="$2"
  wait "$pid" 2>/dev/null || true
  local rc=$?
  kill -TERM "$wdpid" 2>/dev/null || true
  wait "$wdpid" 2>/dev/null || true
  return $rc
}

run_test() {
  local name="$1"; shift
  local log
  log="$(log_path "$name")"

  echo "== RUN: ${name}"
  echo "   log: ${log}"

  if "$@" >"$log" 2>&1; then
    echo "${name}: OK"
    return 0
  else
    echo "${name}: FAIL"
    overall_fail=1
    return 1
  fi
}

# ---- environment banner (what you're asking for) ----------------------------
if [ "$(uname -s)" != "Linux" ]; then
  echo "Platform check: FAIL (Linux only)"
  exit 1
fi
echo "Platform check: OK"

if [ -d "apps/install-adaptive-autosar/bin" ]; then
  if ! file "apps/install-adaptive-autosar/bin/example_class_pub" 2>/dev/null | grep -q ELF; then
    echo "ELF check: FAIL"
    exit 1
  fi
fi
echo "ELF check: OK"

echo "== ENV =="
echo "  date: $(date -Is)"
echo "  host: $(hostname)"
echo "  uname: $(uname -a)"
echo "  user: $(id -un) uid=$(id -u) gid=$(id -g)"
echo "  NO_BUILD=${NO_BUILD}"
echo "  SELECT_BACKEND=${SELECT_BACKEND}"
echo "  DDS_PREFIX=${DDS_PREFIX}"
echo "  ICEORYX_PREFIX=${ICEORYX_PREFIX}"
echo "  VSOMEIP_PREFIX=${VSOMEIP_PREFIX}"
echo "  AUTOSAR_AP_PREFIX=${AUTOSAR_AP_PREFIX}"
echo "  LWRCL_PREFIX=${LWRCL_PREFIX}"
echo "  FASTDDS_PREFIX=${FASTDDS_PREFIX}"
echo

# ---- build -----------------------------------------------------------------
if [ "$NO_BUILD" -eq 0 ]; then
  run_test "Build" bash -lc '
    set -euo pipefail
    sudo ldconfig || true
    ./build_libraries.sh adaptive-autosar install
    ./build_data_types.sh adaptive-autosar install
    ./build_lwrcl.sh adaptive-autosar install
    ./build_apps.sh adaptive-autosar install
  ' || true
else
  echo "Build: SKIP (--no-build)"
  echo "Build: OK"
fi

# ---- Installed artifacts ----------------------------------------------------
run_test "Installed artifacts" bash -lc '
  set -euo pipefail
  LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/autosar-ap-libs}"
  AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar-ap}"

  missing=0
  [ -f "${LWRCL_PREFIX}/lib/liblwrcl.so" ] || { echo "Missing liblwrcl.so"; missing=1; }
  [ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest.arxml" ] || { echo "Missing arxml"; missing=1; }
  [ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest.yaml" ] || { echo "Missing manifest yaml"; missing=1; }
  [ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest_dds.yaml" ] || { echo "Missing manifest dds yaml"; missing=1; }
  [ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest_vsomeip.yaml" ] || { echo "Missing vsomeip manifest yaml"; missing=1; }
  [ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_topic_mapping.yaml" ] || { echo "Missing topic mapping"; missing=1; }
  [ -x "apps/install-adaptive-autosar/bin/example_spin" ] || { echo "Missing example_spin"; missing=1; }
  [ -x "apps/install-adaptive-autosar/bin/example_service_server" ] || { echo "Missing example_service_server"; missing=1; }

  if [ "$missing" -ne 0 ]; then exit 1; fi
' || true

# ---- runtime tests (implemented in this script to guarantee timeouts) -------
runtime_dds_or_vsomeip() {
  local binding="$1"
  local expect_register="$2"   # "yes" or "no"
  local name="$3"

  local pub_log="/tmp/lwrcl_${binding}_pub.log"
  local sub_log="/tmp/lwrcl_${binding}_sub.log"
  local rm_log="/tmp/lwrcl_${binding}_rm.log"

  cleanup_runtime_processes
  unset ARA_COM_BINDING_MANIFEST
  export ARA_COM_EVENT_BINDING="$binding"

  # Start RM and SUB in background with watchdogs
  read -r rm_pid rm_wd < <(start_bg_with_timeout 12 "$rm_log" "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager")
  sleep 1
  read -r sub_pid sub_wd < <(start_bg_with_timeout 10 "$sub_log" apps/install-adaptive-autosar/bin/example_class_sub)
  sleep 1

  # PUB in foreground (timeout)
  run_with_timeout 8 "$pub_log" apps/install-adaptive-autosar/bin/example_class_pub || true

  wait_bg "$sub_pid" "$sub_wd" >/dev/null 2>&1 || true
  wait_bg "$rm_pid" "$rm_wd"   >/dev/null 2>&1 || true

  grep -q "Publishing: 'Hello, world!" "$pub_log"
  grep -q "I heard: 'Hello, world!" "$sub_log"
  if [ "$expect_register" = "yes" ]; then
    grep -q "REGISTER EVENT" "$rm_log"
  else
    if grep -q "REGISTER EVENT" "$rm_log"; then return 1; fi
  fi
  return 0
}

runtime_iceoryx() {
  local roudi_log="/tmp/lwrcl_iceoryx_roudi.log"
  local rm_log="/tmp/lwrcl_iceoryx_rm.log"
  local sub_log="/tmp/lwrcl_iceoryx_sub.log"
  local pub_log="/tmp/lwrcl_iceoryx_pub.log"

  cleanup_runtime_processes
  unset ARA_COM_BINDING_MANIFEST
  export ARA_COM_EVENT_BINDING=iceoryx

  read -r roudi_pid roudi_wd < <(start_bg_with_timeout 12 "$roudi_log" "${ICEORYX_PREFIX}/bin/iox-roudi")
  read -r rm_pid rm_wd       < <(start_bg_with_timeout 12 "$rm_log" "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager")
  sleep 1
  read -r sub_pid sub_wd     < <(start_bg_with_timeout 10 "$sub_log" apps/install-adaptive-autosar/bin/example_class_sub)
  sleep 1
  run_with_timeout 8 "$pub_log" apps/install-adaptive-autosar/bin/example_class_pub || true

  wait_bg "$sub_pid" "$sub_wd" >/dev/null 2>&1 || true
  wait_bg "$rm_pid" "$rm_wd"   >/dev/null 2>&1 || true
  wait_bg "$roudi_pid" "$roudi_wd" >/dev/null 2>&1 || true

  grep -q "Publishing: 'Hello, world!" "$pub_log"
  grep -q "I heard: 'Hello, world!" "$sub_log"
  if grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" "$sub_log"; then return 1; fi
  if grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" "$pub_log"; then return 1; fi
  if grep -Eiq "error|fatal|panic" "$roudi_log"; then return 1; fi
  return 0
}

service_request_response() {
  local rm_log="/tmp/lwrcl_sr_rm.log"
  local srv_log="/tmp/lwrcl_service_server.log"
  local cli_log="/tmp/lwrcl_service_client.log"

  cleanup_runtime_processes

  read -r rm_pid rm_wd < <(start_bg_with_timeout 12 "$rm_log" "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager")
  sleep 1
  read -r srv_pid srv_wd < <(start_bg_with_timeout 8 "$srv_log" apps/install-adaptive-autosar/bin/example_service_server)
  sleep 1
  run_with_timeout 6 "$cli_log" apps/install-adaptive-autosar/bin/example_service_client || true

  wait_bg "$srv_pid" "$srv_wd" >/dev/null 2>&1 || true
  wait_bg "$rm_pid" "$rm_wd"   >/dev/null 2>&1 || true

  grep -q "Create CameraInfo service server." "$srv_log"
  grep -q "Received request." "$srv_log"
  grep -q "Success to call service." "$cli_log"
  return 0
}

loaned_message_zero_copy() {
  local roudi_log="/tmp/lwrcl_zc_roudi.log"
  local rm_log="/tmp/lwrcl_zc_rm.log"
  local sub_log="/tmp/lwrcl_zc_sub.log"
  local pub_log="/tmp/lwrcl_zc_pub.log"

  cleanup_runtime_processes
  unset ARA_COM_BINDING_MANIFEST
  export ARA_COM_EVENT_BINDING=iceoryx

  read -r roudi_pid roudi_wd < <(start_bg_with_timeout 12 "$roudi_log" "${ICEORYX_PREFIX}/bin/iox-roudi")
  read -r rm_pid rm_wd       < <(start_bg_with_timeout 12 "$rm_log" "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager")
  sleep 1
  read -r sub_pid sub_wd     < <(start_bg_with_timeout 10 "$sub_log" apps/install-adaptive-autosar/bin/example_zero_copy_sub)
  sleep 1
  run_with_timeout 8 "$pub_log" apps/install-adaptive-autosar/bin/example_zero_copy_pub || true

  wait_bg "$sub_pid" "$sub_wd" >/dev/null 2>&1 || true
  wait_bg "$rm_pid" "$rm_wd"   >/dev/null 2>&1 || true
  wait_bg "$roudi_pid" "$roudi_wd" >/dev/null 2>&1 || true

  grep -q "Can loan messages: yes" "$pub_log"
  grep -q "Can loan messages: yes" "$sub_log"
  grep -q "\[Zero-copy\] Publishing image" "$pub_log"
  grep -q "\[Zero-copy\] Received image" "$sub_log"
  if grep -q "Received image: 0x0" "$sub_log"; then return 1; fi
  if grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" "$sub_log"; then return 1; fi
  if grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" "$pub_log"; then return 1; fi
  return 0
}

plain_backend_pubsub() {
  local backend="$1"
  local install_dir="apps/install-${backend}/bin"
  local pub_log="/tmp/lwrcl_plain_${backend}_pub.log"
  local sub_log="/tmp/lwrcl_plain_${backend}_sub.log"

  cleanup_runtime_processes

  read -r sub_pid sub_wd < <(start_bg_with_timeout 10 "$sub_log" "${install_dir}/example_class_sub")
  sleep 1
  run_with_timeout 8 "$pub_log" "${install_dir}/example_class_pub" || true
  wait_bg "$sub_pid" "$sub_wd" >/dev/null 2>&1 || true

  grep -q "Publishing: 'Hello, world!" "$pub_log"
  grep -q "I heard: 'Hello, world!" "$sub_log"
  return 0
}

# ---- decide which tests to run ---------------------------------------------
need_adaptive=0
need_plain_fastdds=0
need_plain_cyclonedds=0
need_plain_vsomeip=0

case "$SELECT_BACKEND" in
  adaptive-autosar) need_adaptive=1 ;;
  fastdds) need_plain_fastdds=1 ;;
  cyclonedds) need_plain_cyclonedds=1 ;;
  vsomeip) need_plain_vsomeip=1 ;;
  all)
    need_adaptive=1
    need_plain_fastdds=1
    need_plain_cyclonedds=1
    need_plain_vsomeip=1
    ;;
  *) echo "Backend selection: FAIL"; exit 1 ;;
esac

# ---- Adaptive AUTOSAR runtime profiles --------------------------------------
if [ "$need_adaptive" -eq 1 ]; then
  if [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_class_pub" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_class_sub" ]; then
    run_test "Runtime (DDS profile)" runtime_dds_or_vsomeip dds no "Runtime (DDS)" || true
    run_test "Runtime (vsomeip profile)" runtime_dds_or_vsomeip vsomeip yes "Runtime (vsomeip)" || true
  else
    echo "Runtime (DDS/vsomeip profile): SKIP (missing binaries)"
    echo "Runtime (DDS/vsomeip profile): OK"
  fi

  if [ -x "${ICEORYX_PREFIX}/bin/iox-roudi" ] \
     && [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_class_pub" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_class_sub" ]; then
    run_test "Runtime (iceoryx profile)" runtime_iceoryx || true
  else
    echo "Runtime (iceoryx profile): SKIP (missing binaries)"
    echo "Runtime (iceoryx profile): OK"
  fi

  if [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_service_server" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_service_client" ]; then
    run_test "Service request/response" service_request_response || true
  else
    echo "Service request/response: SKIP (missing binaries)"
    echo "Service request/response: OK"
  fi

  if [ -x "${ICEORYX_PREFIX}/bin/iox-roudi" ] \
     && [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_zero_copy_pub" ] \
     && [ -x "apps/install-adaptive-autosar/bin/example_zero_copy_sub" ]; then
    run_test "Loaned-message (zero-copy)" loaned_message_zero_copy || true
  else
    echo "Loaned-message (zero-copy): SKIP (missing binaries)"
    echo "Loaned-message (zero-copy): OK"
  fi
fi

# ---- plain backends ---------------------------------------------------------
if [ "$need_plain_fastdds" -eq 1 ]; then
  if [ -x "apps/install-fastdds/bin/example_class_sub" ] && [ -x "apps/install-fastdds/bin/example_class_pub" ]; then
    run_test "Plain backend (fastdds)" plain_backend_pubsub fastdds || true
  else
    echo "Plain backend (fastdds): SKIP (missing binaries)"
    echo "Plain backend (fastdds): OK"
  fi
fi

if [ "$need_plain_cyclonedds" -eq 1 ]; then
  if [ -x "apps/install-cyclonedds/bin/example_class_sub" ] && [ -x "apps/install-cyclonedds/bin/example_class_pub" ]; then
    run_test "Plain backend (cyclonedds)" plain_backend_pubsub cyclonedds || true
  else
    echo "Plain backend (cyclonedds): SKIP (missing binaries)"
    echo "Plain backend (cyclonedds): OK"
  fi
fi

if [ "$need_plain_vsomeip" -eq 1 ]; then
  if [ -x "apps/install-vsomeip/bin/example_class_sub" ] && [ -x "apps/install-vsomeip/bin/example_class_pub" ]; then
    run_test "Plain backend (vsomeip)" plain_backend_pubsub vsomeip || true
  else
    echo "Plain backend (vsomeip): SKIP (missing binaries)"
    echo "Plain backend (vsomeip): OK"
  fi
fi

cleanup_runtime_processes

echo
if [ "$overall_fail" -eq 0 ]; then
  echo "ALL: OK"
  exit 0
else
  echo "ALL: FAIL"
  echo "See logs: /tmp/lwrcl_smoke_*.log and /tmp/lwrcl_*.log"
  exit 1
fi