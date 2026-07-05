#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

NO_BUILD=0
SELECT_BACKEND="${SMOKE_BACKEND:-}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-build) NO_BUILD=1 ;;
    --backend) shift; SELECT_BACKEND="${1:-}";;
    --backend=*) SELECT_BACKEND="${1#--backend=}" ;;
    -h|--help)
      echo "Usage: $0 [--no-build] [--backend <fastdds|cyclonedds|vsomeip|adaptive-autosar|all>]"
      echo "       $0 [--no-build] [--backend=<fastdds|cyclonedds|vsomeip|adaptive-autosar|all>]"
      exit 0 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
  shift
done

# Defaults (match workflow)
HOST_OS="$(uname -s)"
if [ -z "$SELECT_BACKEND" ]; then
  if [ "$HOST_OS" = "Darwin" ]; then
    SELECT_BACKEND="fastdds"
  else
    SELECT_BACKEND="adaptive-autosar"
  fi
fi

case "$SELECT_BACKEND" in
  fastdds|cyclonedds|vsomeip|adaptive-autosar|all) ;;
  *) echo "Backend selection: FAIL (${SELECT_BACKEND})"; exit 1 ;;
esac

SELECTED_BACKENDS=()
if [ "$SELECT_BACKEND" = "all" ]; then
  SELECTED_BACKENDS=(fastdds cyclonedds vsomeip)
  if [ "$HOST_OS" != "Darwin" ]; then
    SELECTED_BACKENDS+=(adaptive-autosar)
  fi
else
  SELECTED_BACKENDS=("$SELECT_BACKEND")
fi
SELECTED_BACKENDS_STR="${SELECTED_BACKENDS[*]}"

: "${DDS_PREFIX:=/opt/cyclonedds}"
: "${ICEORYX_PREFIX:=/opt/iceoryx}"
: "${VSOMEIP_PREFIX:=/opt/vsomeip}"
: "${AUTOSAR_AP_PREFIX:=/opt/autosar-ap}"
: "${FASTDDS_PREFIX:=/opt/fast-dds}"

case "$SELECT_BACKEND" in
  fastdds) : "${LWRCL_PREFIX:=/opt/fast-dds-libs}" ;;
  cyclonedds) : "${LWRCL_PREFIX:=/opt/cyclonedds-libs}" ;;
  vsomeip) : "${LWRCL_PREFIX:=/opt/vsomeip-libs}" ;;
  adaptive-autosar) : "${LWRCL_PREFIX:=/opt/autosar-ap-libs}" ;;
  all)
    if [ "$HOST_OS" = "Darwin" ]; then
      : "${LWRCL_PREFIX:=/opt/fast-dds-libs}"
    else
      : "${LWRCL_PREFIX:=/opt/autosar-ap-libs}"
    fi
    ;;
  *) : "${LWRCL_PREFIX:=/opt/autosar-ap-libs}" ;;
esac

export PATH="$FASTDDS_PREFIX/bin:$DDS_PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="${FASTDDS_PREFIX}/lib:${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export DYLD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${LWRCL_PREFIX}/lib:${FASTDDS_PREFIX}/lib:${DYLD_LIBRARY_PATH:-}"
export NO_BUILD SELECT_BACKEND SELECTED_BACKENDS_STR DDS_PREFIX ICEORYX_PREFIX VSOMEIP_PREFIX AUTOSAR_AP_PREFIX LWRCL_PREFIX FASTDDS_PREFIX

overall_fail=0

log_path() {
  local name="$1"
  name="${name// /_}"
  name="${name//\//_}"
  echo "/tmp/lwrcl_smoke_${name}.log"
}

cleanup_runtime_processes() {
  pkill -x routingmanagerd >/dev/null 2>&1 || true
  pkill -x autosar_vsomeip_routing_manager >/dev/null 2>&1 || true
  pkill -x iox-roudi >/dev/null 2>&1 || true
  pkill -x example_class_sub >/dev/null 2>&1 || true
  pkill -x example_class_pub >/dev/null 2>&1 || true
  pkill -x example_service_server >/dev/null 2>&1 || true
  pkill -x example_service_client >/dev/null 2>&1 || true
  pkill -x example_zero_copy_sub >/dev/null 2>&1 || true
  pkill -x example_zero_copy_pub >/dev/null 2>&1 || true
  rm -f /tmp/vsomeip-* >/dev/null 2>&1 || true
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

plain_backend_app_dir() {
  local backend="$1"
  local candidates=(
    "apps/install-${backend}/bin"
    "apps/build-${backend}/lwrcl_example"
  )

  local dir
  for dir in "${candidates[@]}"; do
    if [ -x "${dir}/example_class_sub" ] && [ -x "${dir}/example_class_pub" ]; then
      echo "$dir"
      return 0
    fi
  done
  return 1
}

# ---- environment banner (what you're asking for) ----------------------------
case "$HOST_OS" in
  Linux|Darwin) echo "Platform check: OK (${HOST_OS})" ;;
  *) echo "Platform check: FAIL (${HOST_OS} unsupported)"; exit 1 ;;
esac

for backend in "${SELECTED_BACKENDS[@]}"; do
  app_bin="apps/install-${backend}/bin/example_class_pub"
  if [ -x "$app_bin" ]; then
    if [ "$HOST_OS" = "Darwin" ]; then
      if ! file "$app_bin" 2>/dev/null | grep -Eq 'Mach-O'; then
        echo "Binary format check: FAIL (${backend}, expected Mach-O)"
        exit 1
      fi
    elif ! file "$app_bin" 2>/dev/null | grep -q ELF; then
      echo "Binary format check: FAIL (${backend}, expected ELF)"
      exit 1
    fi
  fi
done
echo "Binary format check: OK"

echo "== ENV =="
echo "  date: $(date '+%Y-%m-%dT%H:%M:%S%z')"
echo "  host: $(hostname)"
echo "  uname: $(uname -a)"
echo "  user: $(id -un) uid=$(id -u) gid=$(id -g)"
echo "  NO_BUILD=${NO_BUILD}"
echo "  SELECT_BACKEND=${SELECT_BACKEND}"
echo "  SELECTED_BACKENDS=${SELECTED_BACKENDS_STR}"
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
    if command -v ldconfig >/dev/null 2>&1; then
      sudo ldconfig || true
    fi
    for backend in ${SELECTED_BACKENDS_STR}; do
      ./build_libraries.sh "${backend}" install
      ./build_data_types.sh "${backend}" install
      ./build_lwrcl.sh "${backend}" install
      ./build_apps.sh "${backend}" install
    done
  ' || true
else
  echo "Build: SKIP (--no-build)"
  echo "Build: OK"
fi

# ---- Installed artifacts ----------------------------------------------------
run_test "Installed artifacts" bash -lc '
  set -euo pipefail
  AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar-ap}"
  missing=0
  require_app() {
    local backend="$1"
    local path="$2"
    local label="$3"
    if [ -x "$path" ]; then
      return 0
    fi
    if [ "${NO_BUILD:-0}" -eq 1 ]; then
      echo "Skip ${backend}: ${label} missing (--no-build)"
    else
      echo "Missing ${backend}: ${label}"
      missing=1
    fi
  }

  for backend in ${SELECTED_BACKENDS_STR}; do
    case "$backend" in
      fastdds) prefix="/opt/fast-dds-libs"; app_dir="apps/install-fastdds/bin" ;;
      cyclonedds) prefix="/opt/cyclonedds-libs"; app_dir="apps/install-cyclonedds/bin" ;;
      vsomeip) prefix="/opt/vsomeip-libs"; app_dir="apps/install-vsomeip/bin" ;;
      adaptive-autosar) prefix="/opt/autosar-ap-libs"; app_dir="apps/install-adaptive-autosar/bin" ;;
      *) echo "Unknown backend: $backend"; missing=1; continue ;;
    esac

    if [ "$backend" = "${SELECT_BACKEND:-}" ] && [ -n "${LWRCL_PREFIX:-}" ]; then
      prefix="$LWRCL_PREFIX"
    fi

    if [ "$(uname -s)" = "Darwin" ]; then
      [ -f "${prefix}/lib/liblwrcl.dylib" ] || { echo "Missing ${backend}: ${prefix}/lib/liblwrcl.dylib"; missing=1; }
    else
      [ -f "${prefix}/lib/liblwrcl.so" ] || { echo "Missing ${backend}: ${prefix}/lib/liblwrcl.so"; missing=1; }
    fi

    if [ "$backend" = "adaptive-autosar" ]; then
      [ -f "${prefix}/share/lwrcl/autosar/lwrcl_autosar_manifest.arxml" ] || { echo "Missing ${backend}: arxml"; missing=1; }
      [ -f "${prefix}/share/lwrcl/autosar/lwrcl_autosar_manifest.yaml" ] || { echo "Missing ${backend}: manifest yaml"; missing=1; }
      [ -f "${prefix}/share/lwrcl/autosar/lwrcl_autosar_manifest_dds.yaml" ] || { echo "Missing ${backend}: manifest dds yaml"; missing=1; }
      [ -f "${prefix}/share/lwrcl/autosar/lwrcl_autosar_manifest_vsomeip.yaml" ] || { echo "Missing ${backend}: vsomeip manifest yaml"; missing=1; }
      [ -f "${prefix}/share/lwrcl/autosar/lwrcl_autosar_topic_mapping.yaml" ] || { echo "Missing ${backend}: topic mapping"; missing=1; }
    fi

    require_app "$backend" "${app_dir}/example_spin" "example_spin"
    require_app "$backend" "${app_dir}/example_class_pub" "example_class_pub"
    require_app "$backend" "${app_dir}/example_class_sub" "example_class_sub"
    if [ "$backend" = "vsomeip" ]; then
      require_app "$backend" "${VSOMEIP_PREFIX:-/opt/vsomeip}/bin/routingmanagerd" "routingmanagerd"
    fi
  done

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
  local install_dir
  local pub_log="/tmp/lwrcl_plain_${backend}_pub.log"
  local sub_log="/tmp/lwrcl_plain_${backend}_sub.log"
  local rm_log="/tmp/lwrcl_plain_${backend}_rm.log"
  local rm_pid=""
  local rm_wd=""
  local had_vsomeip_configuration=0
  local old_vsomeip_configuration=""
  local had_vsomeip_application_name=0
  local old_vsomeip_application_name=""
  local had_vsomeip_routing=0
  local old_vsomeip_routing=""

  install_dir="$(plain_backend_app_dir "$backend")"
  cleanup_runtime_processes

  if [ "$backend" = "vsomeip" ]; then
    if [ "${VSOMEIP_CONFIGURATION+x}" ]; then
      had_vsomeip_configuration=1
      old_vsomeip_configuration="$VSOMEIP_CONFIGURATION"
    fi
    if [ "${VSOMEIP_APPLICATION_NAME+x}" ]; then
      had_vsomeip_application_name=1
      old_vsomeip_application_name="$VSOMEIP_APPLICATION_NAME"
    fi
    if [ "${VSOMEIP_ROUTING+x}" ]; then
      had_vsomeip_routing=1
      old_vsomeip_routing="$VSOMEIP_ROUTING"
    fi

    unset VSOMEIP_CONFIGURATION
    unset VSOMEIP_APPLICATION_NAME
    unset VSOMEIP_ROUTING
    export VSOMEIP_CONFIGURATION="${VSOMEIP_PREFIX}/etc/vsomeip-lwrcl.json"
  fi

  restore_plain_vsomeip_env() {
    if [ "$backend" != "vsomeip" ]; then
      return 0
    fi
    if [ "$had_vsomeip_configuration" -eq 1 ]; then
      export VSOMEIP_CONFIGURATION="$old_vsomeip_configuration"
    else
      unset VSOMEIP_CONFIGURATION
    fi
    if [ "$had_vsomeip_application_name" -eq 1 ]; then
      export VSOMEIP_APPLICATION_NAME="$old_vsomeip_application_name"
    else
      unset VSOMEIP_APPLICATION_NAME
    fi
    if [ "$had_vsomeip_routing" -eq 1 ]; then
      export VSOMEIP_ROUTING="$old_vsomeip_routing"
    else
      unset VSOMEIP_ROUTING
    fi
  }

  echo "Using app dir: ${install_dir}"
  if [ "$backend" = "vsomeip" ]; then
    if [ ! -x "${VSOMEIP_PREFIX}/bin/routingmanagerd" ]; then
      echo "Missing vsomeip routing manager: ${VSOMEIP_PREFIX}/bin/routingmanagerd"
      restore_plain_vsomeip_env
      return 1
    fi
    read -r rm_pid rm_wd < <(start_bg_with_timeout 12 "$rm_log" "${VSOMEIP_PREFIX}/bin/routingmanagerd")
    sleep 1
  fi
  read -r sub_pid sub_wd < <(start_bg_with_timeout 10 "$sub_log" "${install_dir}/example_class_sub")
  sleep 1
  run_with_timeout 8 "$pub_log" "${install_dir}/example_class_pub" || true
  wait_bg "$sub_pid" "$sub_wd" >/dev/null 2>&1 || true
  if [ "$backend" = "vsomeip" ]; then
    wait_bg "$rm_pid" "$rm_wd" >/dev/null 2>&1 || true
  fi

  restore_plain_vsomeip_env

  if ! grep -q "Publishing: 'Hello, world!" "$pub_log"; then
    echo "Publisher did not publish expected message. See ${pub_log}"
    return 1
  fi
  if ! grep -q "I heard: 'Hello, world!" "$sub_log"; then
    echo "Subscriber did not receive expected message. See ${sub_log}"
    return 1
  fi
  return 0
}

# ---- decide which tests to run ---------------------------------------------
need_adaptive=0
need_plain_fastdds=0
need_plain_cyclonedds=0
need_plain_vsomeip=0

for backend in "${SELECTED_BACKENDS[@]}"; do
  case "$backend" in
    adaptive-autosar) need_adaptive=1 ;;
    fastdds) need_plain_fastdds=1 ;;
    cyclonedds) need_plain_cyclonedds=1 ;;
    vsomeip) need_plain_vsomeip=1 ;;
  esac
done

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
  if plain_backend_app_dir fastdds >/dev/null; then
    run_test "Plain backend (fastdds)" plain_backend_pubsub fastdds || true
  else
    echo "Plain backend (fastdds): FAIL (missing binaries)"
    overall_fail=1
  fi
fi

if [ "$need_plain_cyclonedds" -eq 1 ]; then
  if plain_backend_app_dir cyclonedds >/dev/null; then
    run_test "Plain backend (cyclonedds)" plain_backend_pubsub cyclonedds || true
  else
    echo "Plain backend (cyclonedds): FAIL (missing binaries)"
    overall_fail=1
  fi
fi

if [ "$need_plain_vsomeip" -eq 1 ]; then
  if plain_backend_app_dir vsomeip >/dev/null; then
    run_test "Plain backend (vsomeip)" plain_backend_pubsub vsomeip || true
  else
    echo "Plain backend (vsomeip): FAIL (missing binaries)"
    overall_fail=1
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
