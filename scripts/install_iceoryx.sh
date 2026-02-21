#!/usr/bin/env bash
set -euo pipefail

ICEORYX_TAG="v2.0.6"
INSTALL_PREFIX="/opt/iceoryx"
BUILD_DIR="${HOME}/build-iceoryx"
_nproc="$(nproc 2>/dev/null || echo 4)"
_mem_kb="$(grep -i MemAvailable /proc/meminfo 2>/dev/null | awk '{print $2}' || echo 0)"
_mem_jobs=$(( _mem_kb / 1572864 ))
[[ "${_mem_jobs}" -lt 1 ]] && _mem_jobs=1
JOBS=$(( _nproc < _mem_jobs ? _nproc : _mem_jobs ))
[[ "${JOBS}" -lt 1 ]] && JOBS=1
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"
ALLOW_NO_ACL="ON"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --tag)
      ICEORYX_TAG="$2"
      shift 2
      ;;
    --build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    --jobs)
      JOBS="$2"
      shift 2
      ;;
    --skip-system-deps)
      SKIP_SYSTEM_DEPS="ON"
      shift
      ;;
    --force)
      FORCE_REINSTALL="ON"
      shift
      ;;
    --strict-acl)
      ALLOW_NO_ACL="OFF"
      shift
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ -f "${INSTALL_PREFIX}/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]]; then
  echo "[INFO] iceoryx already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
  exit 0
fi

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
  else
    echo "[ERROR] Please run as root or install sudo." >&2
    exit 1
  fi
fi

if [[ "${SKIP_SYSTEM_DEPS}" != "ON" ]] && command -v apt-get >/dev/null 2>&1; then
  ${SUDO} apt-get update -qq
  ${SUDO} apt-get install -y --no-install-recommends libacl1-dev libncurses5-dev
fi

${SUDO} mkdir -p "${INSTALL_PREFIX}"

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

git clone --depth 1 --branch "${ICEORYX_TAG}" https://github.com/eclipse-iceoryx/iceoryx.git "${BUILD_DIR}/iceoryx"

if [[ "${ALLOW_NO_ACL}" == "ON" ]]; then
  ACCESS_CONTROL_CPP="${BUILD_DIR}/iceoryx/iceoryx_hoofs/source/posix_wrapper/access_control.cpp"
  if [[ -f "${ACCESS_CONTROL_CPP}" ]]; then
    # In some container kernels (/dev/shm without POSIX ACL support), acl_set_fd returns ENOTSUP/EOPNOTSUPP.
    # Fallback to continue startup with default shm permissions so RouDi can run.
    patch -d "${BUILD_DIR}/iceoryx" -p1 <<'PATCH_EOF'
diff --git a/iceoryx_hoofs/source/posix_wrapper/access_control.cpp b/iceoryx_hoofs/source/posix_wrapper/access_control.cpp
index 2647fcdf..6f145f31 100644
--- a/iceoryx_hoofs/source/posix_wrapper/access_control.cpp
+++ b/iceoryx_hoofs/source/posix_wrapper/access_control.cpp
@@ -70,6 +70,12 @@ bool AccessController::writePermissionsToFile(const int32_t f_fileDescriptor) c
     auto aclSetFdCall = posixCall(acl_set_fd)(f_fileDescriptor, workingACL.get()).successReturnValue(0).evaluate();
     if (aclSetFdCall.has_error())
     {
+        const auto aclErrno = aclSetFdCall.get_error().errnum;
+        if (aclErrno == ENOTSUP || aclErrno == EOPNOTSUPP)
+        {
+            std::cerr << "Warning: ACL is not supported in this environment. Continuing without ACL enforcement." << std::endl;
+            return true;
+        }
         std::cerr << "Error: Could not set file ACL." << std::endl;
         return false;
     }
PATCH_EOF
  fi
fi

cmake -S "${BUILD_DIR}/iceoryx/iceoryx_meta" -B "${BUILD_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TEST=OFF \
  -DINTROSPECTION=OFF \
  -DBUILD_DOC=OFF \
  -DEXAMPLES=OFF

cmake --build "${BUILD_DIR}/build" -j"${JOBS}"
${SUDO} cmake --install "${BUILD_DIR}/build"

echo "[OK] Installed iceoryx ${ICEORYX_TAG} to ${INSTALL_PREFIX}"
