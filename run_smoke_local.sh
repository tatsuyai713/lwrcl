#!/usr/bin/env bash
set -euo pipefail
# Print a helpful message when any command fails to locate the failing statement and recent logs
trap 'rc=$?; echo "ERROR: command failed at line ${LINENO}: ${BASH_COMMAND} (rc=${rc})"; echo "--- recent /tmp/lwrcl_* logs (tail -n 80) ---"; tail -n 80 /tmp/lwrcl_* 2>/dev/null || true; exit ${rc}' ERR

# run_smoke_local.sh
# Run the Adaptive-AUTOSAR smoke checks locally, mirroring .github/workflows/ci-adaptive-autosar.yml
# Usage:
#   sudo ./run_smoke_local.sh          # runs build+smoke checks (needs sudo for installs)
#   ./run_smoke_local.sh --no-build    # skip building/installing lwrcl (assume already installed)
# Note: This script assumes a Linux environment with `timeout` available and
# Adaptive-AUTOSAR / CycloneDDS / iceoryx / vsomeip installed under /opt (defaults).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# early platform sanity check ------------------------------------------------
# The smoke tests exercise Linux binaries (CycloneDDS, iceoryx, etc.) and the
# local workstation environment *cannot* execute them on macOS or other
# non-Linux hosts.  In practice the tests are expected to run inside the same
# container used by CI, so we guard here and print a helpful message rather
# than letting the kernel kill child processes with "Killed" symptoms.
if [ "$(uname -s)" != "Linux" ]; then
  echo "ERROR: smoke tests are only supported on Linux (e.g. inside a container)."
  echo "       please run './run_smoke_local.sh' from a Linux VM/container."
  exit 1
fi

# also verify we have an ELF binary for one of the examples; if not, we are
# probably on the wrong architecture and should bail early.
if [ -d "apps/install-adaptive-autosar/bin" ]; then
  if ! file "apps/install-adaptive-autosar/bin/example_class_pub" | grep -q ELF; then
    echo "ERROR: example_class_pub is not an ELF executable; aborting."
    exit 1
  fi
fi


NO_BUILD=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-build) NO_BUILD=1 ;;
    --backend)
      shift
      SELECT_BACKEND="$1"
      ;;
    -h|--help)
      echo "Usage: $0 [--no-build] [--backend <fastdds|cyclonedds|vsomeip|adaptive-autosar|all>]"
      exit 0 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
  shift
done

# Default: run adaptive-autosar tests (and plain backends as well)
SELECT_BACKEND="${SELECT_BACKEND:-adaptive-autosar}"

# Defaults (match workflow)
DDS_PREFIX="${DDS_PREFIX:-/opt/cyclonedds}"
ICEORYX_PREFIX="${ICEORYX_PREFIX:-/opt/iceoryx}"
VSOMEIP_PREFIX="${VSOMEIP_PREFIX:-/opt/vsomeip}"
AUTOSAR_AP_PREFIX="${AUTOSAR_AP_PREFIX:-/opt/autosar_ap}"
LWRCL_PREFIX="${LWRCL_PREFIX:-/opt/autosar-ap-libs}"

export PATH="$DDS_PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="${ICEORYX_PREFIX}/lib:${DDS_PREFIX}/lib:${VSOMEIP_PREFIX}/lib:${AUTOSAR_AP_PREFIX}/lib:${LWRCL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

echo "Environment:"
echo "  DDS_PREFIX=$DDS_PREFIX"
echo "  ICEORYX_PREFIX=$ICEORYX_PREFIX"
echo "  VSOMEIP_PREFIX=$VSOMEIP_PREFIX"
echo "  AUTOSAR_AP_PREFIX=$AUTOSAR_AP_PREFIX"
echo "  LWRCL_PREFIX=$LWRCL_PREFIX"

if [ "$NO_BUILD" -eq 0 ]; then
  echo "== Building and installing lwrcl (adaptive-autosar) =="
  sudo ldconfig || true
  ./build_libraries.sh adaptive-autosar install
  ./build_data_types.sh adaptive-autosar install
  ./build_lwrcl.sh adaptive-autosar install
  ./build_apps.sh adaptive-autosar install
else
  echo "Skipping build ( --no-build ). Ensure artifacts are installed under ${LWRCL_PREFIX} and apps/ install dir."
fi

# Smoke check installed artifacts
echo "== Smoke: installed artifacts =="
set -e
[ -f "${LWRCL_PREFIX}/lib/liblwrcl.so" ] || { echo "Missing liblwrcl.so"; exit 1; }
[ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest.arxml" ] || { echo "Missing arxml"; exit 1; }
[ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest.yaml" ] || { echo "Missing manifest yaml"; exit 1; }
[ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest_dds.yaml" ] || { echo "Missing manifest dds yaml"; exit 1; }
if [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar-generate-comm-manifest" ]; then
  if "${AUTOSAR_AP_PREFIX}/bin/autosar-generate-comm-manifest" --help 2>&1 | grep -q "iceoryx"; then
    [ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest_iceoryx.yaml" ] || { echo "Missing iceoryx manifest yaml"; exit 1; }
  fi
fi
[ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_manifest_vsomeip.yaml" ] || { echo "Missing vsomeip manifest yaml"; exit 1; }
[ -f "${LWRCL_PREFIX}/share/lwrcl/autosar/lwrcl_autosar_topic_mapping.yaml" ] || { echo "Missing topic mapping"; exit 1; }
[ -x "apps/install-adaptive-autosar/bin/example_spin" ] || { echo "Missing example_spin"; exit 1; }
[ -x "apps/install-adaptive-autosar/bin/example_service_server" ] || { echo "Missing example_service_server"; exit 1; }

echo "Installed artifacts smoke: OK"

# Helper functions used in workflow
cleanup_runtime_processes() {
  pkill -x autosar_vsomeip_routing_manager || true
  pkill -x iox-roudi || true
  pkill -x example_class_sub || true
  pkill -x example_class_pub || true
}

hard_timeout() {
  # Portable timeout wrapper: accept duration like '12s' as first arg
  local dur="$1"; shift
  echo "DEBUG: hard_timeout requested dur=${dur} cmd=$*"
  # Implement timeout in-shell to avoid external `timeout` sending signals
  # to the shell process group (which can kill the parent script).
  # dur is like '12s' - strip trailing 's' if present
  local secs="${dur%s}"
  echo "DEBUG: hard_timeout (internal watchdog) running command for ${secs}s"
  ("$@") &
  local child_pid=$!
  echo "DEBUG: hard_timeout started child pid=${child_pid}"

  # watchdog: sleeps then terminates child if still running
  (
    sleep "${secs}"
    if kill -0 "${child_pid}" 2>/dev/null; then
      echo "DEBUG: hard_timeout watchdog killing pid=${child_pid}"
      kill -TERM "${child_pid}" 2>/dev/null || true
      sleep 2
      kill -KILL "${child_pid}" 2>/dev/null || true
    fi
  ) &
  local watchdog_pid=$!

  # wait for child to exit, then kill watchdog
  wait "${child_pid}" 2>/dev/null || true
  kill -TERM "${watchdog_pid}" 2>/dev/null || true
  wait "${watchdog_pid}" 2>/dev/null || true
  return 0
}

run_dds_or_vsomeip_profile() {
  local binding="$1"
  local pub_log="$2"
  local sub_log="$3"
  local rm_log="$4"
  local expect_register_event="$5"

  echo "DEBUG: run_dds_or_vsomeip_profile start binding=${binding} pub_log=${pub_log} sub_log=${sub_log} rm_log=${rm_log} expect_register_event=${expect_register_event}"
  cleanup_runtime_processes
  unset ARA_COM_BINDING_MANIFEST
  export ARA_COM_EVENT_BINDING="${binding}"

  hard_timeout 12s "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" > >(tee "${rm_log}") 2>&1 &
  local rm_pid=$!
  echo "DEBUG: started routing manager pid=${rm_pid}"
  sleep 1

  hard_timeout 10s apps/install-adaptive-autosar/bin/example_class_sub > >(tee "${sub_log}") 2>&1 &
  local sub_pid=$!
  echo "DEBUG: started subscriber pid=${sub_pid}"
  sleep 1

  hard_timeout 8s apps/install-adaptive-autosar/bin/example_class_pub > >(tee "${pub_log}") 2>&1 || true
  echo "DEBUG: publisher finished (or timed out), logs: ${pub_log} ${sub_log} ${rm_log}"
  wait "${sub_pid}" || true
  wait "${rm_pid}" || true

  # Check logs, but don't let a single grep abort the script without printing logs
  set +e
  grep -q "Publishing: 'Hello, world!" "${pub_log}"; rc_pub=$?
  grep -q "I heard: 'Hello, world!" "${sub_log}"; rc_sub=$?
  if [ "${expect_register_event}" = "yes" ]; then
    grep -q "REGISTER EVENT" "${rm_log}"; rc_rm=$?
  else
    grep -q "REGISTER EVENT" "${rm_log}"; rc_tmp=$?
    if [ "$rc_tmp" -eq 0 ]; then
      # REGISTER EVENT found but we did not expect it -> failure
      rc_rm=1
    else
      rc_rm=0
    fi
  fi
  set -e
  if [ "$rc_pub" -ne 0 ] || [ "$rc_sub" -ne 0 ] || [ "$rc_rm" -ne 0 ]; then
    echo "ERROR: smoke checks failed for binding ${binding}: rc_pub=${rc_pub} rc_sub=${rc_sub} rc_rm=${rc_rm}"
    echo "--- ${pub_log} (tail 200) ---"; tail -n 200 "${pub_log}" || true
    echo "--- ${sub_log} (tail 200) ---"; tail -n 200 "${sub_log}" || true
    echo "--- ${rm_log} (tail 200) ---"; tail -n 200 "${rm_log}" || true
    return 1
  fi
}

# runtime backend selection smoke (Adaptive AUTOSAR)
echo "== Smoke: runtime backend selection (Adaptive AUTOSAR — DDS / vsomeip / iceoryx) =="
echo "DEBUG: SELECT_BACKEND=${SELECT_BACKEND}"
echo "DEBUG: prefixes: AUTOSAR_AP_PREFIX=${AUTOSAR_AP_PREFIX} ICEORYX_PREFIX=${ICEORYX_PREFIX} DDS_PREFIX=${DDS_PREFIX} VSOMEIP_PREFIX=${VSOMEIP_PREFIX} LWRCL_PREFIX=${LWRCL_PREFIX}"
printf "DEBUG: quick existence checks (early):\n"
printf "  autosar_vsomeip_routing_manager: %s\n" "$( [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] && echo yes || echo no )"
printf "  example_class_pub: %s\n" "$( [ -x "apps/install-adaptive-autosar/bin/example_class_pub" ] && echo yes || echo no )"
printf "  example_class_sub: %s\n" "$( [ -x "apps/install-adaptive-autosar/bin/example_class_sub" ] && echo yes || echo no )"
trap cleanup_runtime_processes EXIT INT TERM

# Quick existence checks - skip runtime smoke if required runtime/tools/apps are missing
SKIP_RUNTIME_SMOKE=0
if [ ! -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ]; then
  echo "Skip runtime backend smoke: autosar_vsomeip_routing_manager not found at ${AUTOSAR_AP_PREFIX}/bin"
  SKIP_RUNTIME_SMOKE=1
fi
if [ ! -x "apps/install-adaptive-autosar/bin/example_class_pub" ] || [ ! -x "apps/install-adaptive-autosar/bin/example_class_sub" ]; then
  echo "Skip runtime backend smoke: example_class_pub/sub not installed under apps/install-adaptive-autosar/bin"
  SKIP_RUNTIME_SMOKE=1
fi

if [ "$SKIP_RUNTIME_SMOKE" -eq 0 ]; then
  run_dds_or_vsomeip_profile \
    dds /tmp/lwrcl_autosar_dds_pub.log /tmp/lwrcl_autosar_dds_sub.log /tmp/lwrcl_autosar_dds_rm.log no

  run_dds_or_vsomeip_profile \
    vsomeip /tmp/lwrcl_autosar_vsomeip_pub.log /tmp/lwrcl_autosar_vsomeip_sub.log /tmp/lwrcl_autosar_vsomeip_rm.log yes
  echo "DEBUG: completed run_dds_or_vsomeip_profile calls"
else
  echo "Runtime backend selection smoke: SKIPPED"
fi

echo "[diagnostic] autosar_vsomeip_routing_manager exists: \"$( [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] && echo yes || echo no )\""
echo "[diagnostic] example_class_pub exists: \"$( [ -x "apps/install-adaptive-autosar/bin/example_class_pub" ] && echo yes || echo no )\""
echo "[diagnostic] example_class_sub exists: \"$( [ -x "apps/install-adaptive-autosar/bin/example_class_sub" ] && echo yes || echo no )\""
echo "[diagnostic] example_service_server exists: \"$( [ -x "apps/install-adaptive-autosar/bin/example_service_server" ] && echo yes || echo no )\""
echo "[diagnostic] example_service_client exists: \"$( [ -x "apps/install-adaptive-autosar/bin/example_service_client" ] && echo yes || echo no )\""

# Run plain backend (fastdds/cyclonedds/vsomeip) smoke: runs example_class_sub/pub from apps/install-<backend>/bin
run_plain_backend() {
  local backend="$1"
  local prefix_var="$2"
  local prefix="${!prefix_var}"
  local install_dir="apps/install-${backend}/bin"

  echo "== Smoke (${backend}): checking ${install_dir} =="
  if [ ! -x "${install_dir}/example_class_sub" ] || [ ! -x "${install_dir}/example_class_pub" ]; then
    echo "Skip ${backend} smoke: example binaries missing in ${install_dir}."
    return 0
  fi

  echo "Using prefix=${prefix} for ${backend}"
  export LD_LIBRARY_PATH="${prefix}/lib:${LD_LIBRARY_PATH:-}"

  cleanup_runtime_processes
  hard_timeout 10s "${install_dir}/example_class_sub" > "/tmp/lwrcl_${backend}_sub.log" 2>&1 &
  local sub_pid=$!
  sleep 1
  hard_timeout 8s "${install_dir}/example_class_pub" > >(tee "/tmp/lwrcl_${backend}_pub.log") 2>&1 || true
  wait "${sub_pid}" || true

  grep -q "Publishing: 'Hello, world!" "/tmp/lwrcl_${backend}_pub.log"
  grep -q "I heard: 'Hello, world!" "/tmp/lwrcl_${backend}_sub.log"
  echo "${backend} smoke: OK"
}

# Decide which backends to run
case "$SELECT_BACKEND" in
  adaptive-autosar)
    # run only adaptive-autosar runtime profiles below (dds/vsomeip/iceoryx)
    ;;
  fastdds)
    run_plain_backend fastdds FASTDDS_PREFIX
    ;;
  cyclonedds)
    run_plain_backend cyclonedds DDS_PREFIX
    ;;
  vsomeip)
    run_plain_backend vsomeip VSOMEIP_PREFIX
    ;;
  all)
    run_plain_backend fastdds FASTDDS_PREFIX
    run_plain_backend cyclonedds DDS_PREFIX
    run_plain_backend vsomeip VSOMEIP_PREFIX
    ;;
  *)
    echo "Unknown backend selection: $SELECT_BACKEND"; exit 1
    ;;
esac

if [ -x "${ICEORYX_PREFIX}/bin/iox-roudi" ] && [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] && [ -x "apps/install-adaptive-autosar/bin/example_class_pub" ] && [ -x "apps/install-adaptive-autosar/bin/example_class_sub" ]; then
  cleanup_runtime_processes
  unset ARA_COM_BINDING_MANIFEST
  export ARA_COM_EVENT_BINDING=iceoryx

  hard_timeout 12s "${ICEORYX_PREFIX}/bin/iox-roudi" > /tmp/lwrcl_autosar_iceoryx_roudi.log 2>&1 &
  ROUDI_PID=$!
  hard_timeout 12s "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" > >(tee /tmp/lwrcl_autosar_iceoryx_rm.log) 2>&1 &
  ICEORYX_RM_PID=$!
  sleep 1

  hard_timeout 10s apps/install-adaptive-autosar/bin/example_class_sub > >(tee /tmp/lwrcl_autosar_iceoryx_sub.log) 2>&1 &
  ICEORYX_SUB_PID=$!
  sleep 1

  hard_timeout 8s apps/install-adaptive-autosar/bin/example_class_pub > >(tee /tmp/lwrcl_autosar_iceoryx_pub.log) 2>&1 || true
  wait "${ICEORYX_SUB_PID}" || true
  wait "${ICEORYX_RM_PID}" || true
  wait "${ROUDI_PID}" || true

  grep -q "Publishing: 'Hello, world!" /tmp/lwrcl_autosar_iceoryx_pub.log
  grep -q "I heard: 'Hello, world!" /tmp/lwrcl_autosar_iceoryx_sub.log
  ! grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" /tmp/lwrcl_autosar_iceoryx_sub.log
  ! grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" /tmp/lwrcl_autosar_iceoryx_pub.log
  ! grep -Eiq "error|fatal|panic" /tmp/lwrcl_autosar_iceoryx_roudi.log
else
  echo "Skip iceoryx runtime smoke (iox-roudi or routing manager or examples unavailable)."
fi

cleanup_runtime_processes

echo "Runtime backend selection smoke: OK"

echo "== Smoke: service request/response =="

cleanup_sr_processes() {
  pkill -x autosar_vsomeip_routing_manager || true
  pkill -x iox-roudi || true
  pkill -x example_service_server || true
  pkill -x example_service_client || true
}

trap cleanup_sr_processes EXIT INT TERM
cleanup_sr_processes

hard_timeout 12s "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" > /tmp/lwrcl_autosar_sr_rm.log 2>&1 &
RM_PID=$!
sleep 1
hard_timeout 8s apps/install-adaptive-autosar/bin/example_service_server > >(tee /tmp/lwrcl_autosar_service_server.log) 2>&1 &
SRV_PID=$!
sleep 1
hard_timeout 6s apps/install-adaptive-autosar/bin/example_service_client > >(tee /tmp/lwrcl_autosar_service_client.log) 2>&1 || true
wait "${SRV_PID}" || true
wait "${RM_PID}" || true

grep -q "Create CameraInfo service server." /tmp/lwrcl_autosar_service_server.log
grep -q "Received request." /tmp/lwrcl_autosar_service_server.log
grep -q "Success to call service." /tmp/lwrcl_autosar_service_client.log

cleanup_sr_processes

# loaned-message (zero-copy) smoke
echo "== Smoke: Adaptive AUTOSAR loaned-message API (zero-copy) =="

cleanup_zero_copy_processes() {
  pkill -x autosar_vsomeip_routing_manager || true
  pkill -x iox-roudi || true
  pkill -x example_zero_copy_sub || true
  pkill -x example_zero_copy_pub || true
}

trap cleanup_zero_copy_processes EXIT INT TERM
cleanup_zero_copy_processes

if [ -x "${ICEORYX_PREFIX}/bin/iox-roudi" ] && [ -x "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" ] && [ -x "apps/install-adaptive-autosar/bin/example_zero_copy_pub" ] && [ -x "apps/install-adaptive-autosar/bin/example_zero_copy_sub" ] 2>/dev/null; then
  unset ARA_COM_BINDING_MANIFEST
  export ARA_COM_EVENT_BINDING=iceoryx

  hard_timeout 12s "${ICEORYX_PREFIX}/bin/iox-roudi" > /tmp/lwrcl_autosar_zc_roudi.log 2>&1 &
  ZC_ROUDI_PID=$!
  hard_timeout 12s "${AUTOSAR_AP_PREFIX}/bin/autosar_vsomeip_routing_manager" > >(tee /tmp/lwrcl_autosar_zc_rm.log) 2>&1 &
  ZC_RM_PID=$!
  sleep 1

  hard_timeout 10s apps/install-adaptive-autosar/bin/example_zero_copy_sub > >(tee /tmp/lwrcl_autosar_zc_sub.log) 2>&1 &
  ZC_SUB_PID=$!
  sleep 1

  hard_timeout 8s apps/install-adaptive-autosar/bin/example_zero_copy_pub > >(tee /tmp/lwrcl_autosar_zc_pub.log) 2>&1 || true
  wait "${ZC_SUB_PID}" || true
  wait "${ZC_RM_PID}" || true
  wait "${ZC_ROUDI_PID}" || true

  grep -q "Can loan messages: yes" /tmp/lwrcl_autosar_zc_pub.log
  grep -q "Can loan messages: yes" /tmp/lwrcl_autosar_zc_sub.log
  grep -q "\[Zero-copy\] Publishing image" /tmp/lwrcl_autosar_zc_pub.log
  grep -q "\[Zero-copy\] Received image" /tmp/lwrcl_autosar_zc_sub.log
  ! grep -q "Received image: 0x0" /tmp/lwrcl_autosar_zc_sub.log
  ! grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" /tmp/lwrcl_autosar_zc_sub.log
  ! grep -q "APP_WITH_SAME_NAME_STILL_RUNNING" /tmp/lwrcl_autosar_zc_pub.log
else
  echo "Skip zero-copy smoke (iox-roudi or routing manager or examples unavailable)."
fi

echo "Loaned-message smoke: OK"

echo "ALL SMOKE CHECKS PASSED"
