#!/bin/bash

set -euo pipefail

# Build an aarch64-host Android NDK toolchain following SnowNF README:
# https://github.com/SnowNF/ndk-aarch64-linux

HOST_ARCH="$(uname -m)"
if [ "${HOST_ARCH}" != "aarch64" ] && [ "${HOST_ARCH}" != "arm64" ]; then
  echo "Warning: host arch is ${HOST_ARCH}. This script targets aarch64 Linux."
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LWRCL_ROOT="${LWRCL_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
LLVM_TOOLCHAIN_DIR="${LLVM_TOOLCHAIN_DIR:-$HOME/llvm-toolchain}"
AARCH64_NDK_REPO_DIR="${AARCH64_NDK_REPO_DIR:-$HOME/aarch64-ndk}"
SNOW_BRANCH="${SNOW_BRANCH:-}"
REPO_BRANCH="${REPO_BRANCH:-llvm-toolchain}"
REPO_SYNC="${REPO_SYNC:-1}"
FORCE_TOOLCHAIN_SYNC="${FORCE_TOOLCHAIN_SYNC:-1}"
INSTALL_DEPS="${INSTALL_DEPS:-1}"
SKIP_SOURCE_SETUP="${SKIP_SOURCE_SETUP:-0}"
RUN_BUILD="${RUN_BUILD:-1}"
COPY_TO_NDK="${COPY_TO_NDK:-auto}"
NDK_ROOT="${NDK_ROOT:-}"
NDK_HOST_TAG="${NDK_HOST_TAG:-linux-x86_64}"
ANDROID_ABI="${ANDROID_ABI:-arm64-v8a}"
ANDROID_API="${ANDROID_API:-31}"

find_toolchain_dir() {
  local base found
  for base in "${LWRCL_ROOT}" "${HOME}" "${HOME}/host_home" "${HOME}/host_home/repos"; do
    if [ -d "${base}" ]; then
      found="$(find "${base}" -maxdepth 5 -path "*/toolchain/llvm_android/do_build.py" -print -quit 2>/dev/null || true)"
      if [ -n "${found}" ]; then
        echo "${found%/toolchain/llvm_android/do_build.py}"
        return 0
      fi
    fi
  done
  return 1
}

mkdir -p "${LLVM_TOOLCHAIN_DIR}" "${AARCH64_NDK_REPO_DIR}"

if [ "${INSTALL_DEPS}" = "1" ] && command -v apt >/dev/null 2>&1; then
  sudo apt update
  sudo apt install -y clang bison llvm llvm-dev python3 python3-dev lld ninja-build cmake crossbuild-essential-arm64 git repo curl rsync
fi

# Auto-detect Android SDK/NDK without requiring env vars.
ANDROID_HOME="${ANDROID_HOME:-${ANDROID_SDK_ROOT:-}}"
if [ -z "${ANDROID_HOME}" ]; then
  for p in "$HOME/android-sdk" "$HOME/Android/Sdk" "/usr/lib/android-sdk"; do
    if [ -d "${p}" ]; then
      ANDROID_HOME="${p}"
      break
    fi
  done
fi
if [ -n "${ANDROID_HOME}" ]; then
  ANDROID_SDK_ROOT="${ANDROID_HOME}"
fi

if [ -z "${NDK_ROOT}" ]; then
  if [ -n "${ANDROID_NDK_HOME:-}" ] && [ -d "${ANDROID_NDK_HOME}" ]; then
    NDK_ROOT="${ANDROID_NDK_HOME}"
  elif [ -n "${ANDROID_HOME:-}" ] && [ -d "${ANDROID_HOME}/ndk" ]; then
    NDK_ROOT="$(ls -d "${ANDROID_HOME}/ndk/"* 2>/dev/null | sort -V | tail -n 1 || true)"
  fi
fi

if [ "${COPY_TO_NDK}" = "auto" ]; then
  if [ -n "${NDK_ROOT}" ] && [ -d "${NDK_ROOT}" ]; then
    COPY_TO_NDK="1"
  else
    COPY_TO_NDK="0"
  fi
fi

# Ensure repo tool exists
if ! command -v repo >/dev/null 2>&1; then
  echo "repo tool not found. Installing to ~/.local/bin/repo"
  mkdir -p "$HOME/.local/bin"
  curl -fsSL https://storage.googleapis.com/git-repo-downloads/repo -o "$HOME/.local/bin/repo"
  chmod +x "$HOME/.local/bin/repo"
  export PATH="$HOME/.local/bin:$PATH"
fi

# Ensure git identity for repo (use env only; do not touch global config)
if ! git config --global user.name >/dev/null 2>&1 || ! git config --global user.email >/dev/null 2>&1; then
  GIT_USER_NAME="${GIT_USER_NAME:-lwrcl-builder}"
  GIT_USER_EMAIL="${GIT_USER_EMAIL:-lwrcl-builder@local}"
  export GIT_AUTHOR_NAME="${GIT_USER_NAME}"
  export GIT_AUTHOR_EMAIL="${GIT_USER_EMAIL}"
  export GIT_COMMITTER_NAME="${GIT_USER_NAME}"
  export GIT_COMMITTER_EMAIL="${GIT_USER_EMAIL}"
fi

# Step 1: Download source code (llvm-toolchain)
cd "${LLVM_TOOLCHAIN_DIR}"
if [ ! -d ".repo" ]; then
  repo init -u https://android.googlesource.com/platform/manifest -b "${REPO_BRANCH}"
fi
if [ "${REPO_SYNC}" = "1" ]; then
  repo sync -c
fi
if [ "${FORCE_TOOLCHAIN_SYNC}" = "1" ]; then
  repo sync -c --force-sync toolchain/llvm_android
  if [ -d "${LLVM_TOOLCHAIN_DIR}/toolchain/llvm_android" ]; then
    repo forall toolchain/llvm_android -c "git reset --hard HEAD && git clean -fdx"
  fi
fi

# Step 1 (cont): Clone SnowNF repo
cd "${AARCH64_NDK_REPO_DIR}"
if [ ! -d "ndk-aarch64-linux" ]; then
  if [ -n "${SNOW_BRANCH}" ]; then
    git clone --branch "${SNOW_BRANCH}" https://github.com/SnowNF/ndk-aarch64-linux
  else
    git clone https://github.com/SnowNF/ndk-aarch64-linux
  fi
fi
git -C "${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux" fetch --all
if [ -z "${SNOW_BRANCH}" ]; then
  SNOW_BRANCH="$(git -C "${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux" symbolic-ref -q --short refs/remotes/origin/HEAD | sed 's#^origin/##')"
fi
if [ -z "${SNOW_BRANCH}" ]; then
  SNOW_BRANCH="$(git -C "${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux" remote show origin 2>/dev/null | sed -n 's/ *HEAD branch: //p')"
fi
if [ -z "${SNOW_BRANCH}" ]; then
  echo "Could not determine SnowNF default branch."
  echo "Please run: SNOW_BRANCH=<branch> ./scripts/build_ndk_aarch64_linux.sh"
  exit 1
fi
echo "Using SnowNF branch: ${SNOW_BRANCH}"
git -C "${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux" checkout "${SNOW_BRANCH}" || git -C "${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux" checkout -b "${SNOW_BRANCH}" "origin/${SNOW_BRANCH}"
git -C "${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux" pull --ff-only || true

# Step 2: Apply SnowNF changes
TOOLCHAIN_DIR="${LLVM_TOOLCHAIN_DIR}/toolchain/llvm_android"
REPO_DIR="${AARCH64_NDK_REPO_DIR}/ndk-aarch64-linux"
if [ ! -d "${TOOLCHAIN_DIR}" ]; then
  echo "Missing: ${TOOLCHAIN_DIR}"
  echo "repo sync did not produce toolchain/llvm_android. Check REPO_BRANCH or set LLVM_TOOLCHAIN_DIR explicitly."
  exit 1
fi

# Ensure toolchain checkout is complete (do_build.py must exist).
if [ ! -f "${TOOLCHAIN_DIR}/do_build.py" ]; then
  echo "Repairing toolchain checkout (do_build.py missing)..."
  rm -rf "${TOOLCHAIN_DIR}"
  repo sync -c --force-sync toolchain/llvm_android
fi
if [ ! -f "${TOOLCHAIN_DIR}/do_build.py" ]; then
  echo "do_build.py still missing in ${TOOLCHAIN_DIR} after repair."
  exit 1
fi

if [ -f "${REPO_DIR}/build.py" ]; then
  # Full replacement mode (SnowNF README, when full sources are present)
  cd "${TOOLCHAIN_DIR}"
  if [ -d patches ]; then
    mv patches ../patches.bak
  fi
  rsync -a --delete --exclude .git "${REPO_DIR}/" "${TOOLCHAIN_DIR}/"
  if [ -d ../patches.bak ]; then
    mv ../patches.bak patches
  fi
elif [ -f "${REPO_DIR}/diff.txt" ]; then
  # Patch mode (r29 diff.txt)
  cd "${TOOLCHAIN_DIR}"
  if [ ! -f "${TOOLCHAIN_DIR}/do_build.py" ]; then
    echo "do_build.py missing in ${TOOLCHAIN_DIR}. repo sync may be incomplete."
    exit 1
  fi
  # Align toolchain/llvm_android to the exact base revision expected by diff.txt.
  python3 - "${REPO_DIR}/diff.txt" "${TOOLCHAIN_DIR}" <<'PY'
import re
import subprocess
import sys
from pathlib import Path

diff_path = Path(sys.argv[1])
repo_dir = Path(sys.argv[2])

def run(cmd):
    return subprocess.check_output(cmd, cwd=repo_dir, text=True).strip()

text = diff_path.read_text(encoding="utf-8", errors="replace")
base_blobs = {}
current = None
for line in text.splitlines():
    if line.startswith("diff --git "):
        parts = line.split()
        if len(parts) >= 4:
            a = parts[2]
            path = a[2:] if a.startswith("a/") else a
            if path.startswith("toolchain/llvm_android/"):
                path = path[len("toolchain/llvm_android/"):]
            current = path
        continue
    if current and line.startswith("index "):
        m = re.match(r"index ([0-9a-f]+)\.\.([0-9a-f]+)", line)
        if m:
            base_blobs[current] = m.group(1)
        continue

target_files = [
    "do_build.py",
    "py3_utils.py",
    "src/llvm_android/builders.py",
    "src/llvm_android/configs.py",
    "src/llvm_android/hosts.py",
    "src/llvm_android/paths.py",
]
targets = [(f, base_blobs.get(f)) for f in target_files if base_blobs.get(f)]
if not targets:
    sys.exit("No base blob hashes found in diff.txt")

def blob_at(commit, path):
    try:
        out = run(["git", "ls-tree", commit, path])
    except subprocess.CalledProcessError:
        return None
    if not out:
        return None
    parts = out.split()
    return parts[2] if len(parts) >= 3 else None

def commits_for(path):
    return run(["git", "rev-list", "--all", "--", path]).splitlines()

# Choose file with shortest history to reduce search.
best = None
best_list = None
for path, blob_prefix in targets:
    try:
        count = int(run(["git", "rev-list", "--count", "--all", "--", path]))
    except Exception:
        count = 10**9
    if best is None or count < best:
        best = count
        best_list = (path, blob_prefix)

path0, blob0 = best_list
commits = commits_for(path0)
matches = []
for c in commits:
    b = blob_at(c, path0)
    if b and b.startswith(blob0):
        matches.append(c)

def filter_matches(commits_list, path, blob_prefix):
    out = []
    for c in commits_list:
        b = blob_at(c, path)
        if b and b.startswith(blob_prefix):
            out.append(c)
    return out

for path, blob_prefix in targets:
    if path == path0:
        continue
    matches = filter_matches(matches, path, blob_prefix)
    if not matches:
        break

if not matches:
    sys.exit("Could not find a commit matching diff.txt base blobs")

target_commit = matches[0]
subprocess.check_call(["git", "checkout", target_commit], cwd=repo_dir)
print(f"Checked out toolchain/llvm_android at {target_commit} to match diff.txt")
PY

  TMP_DIFF="$(mktemp)"
  # Normalize diff paths to apply cleanly inside toolchain/llvm_android.
  python3 - "${REPO_DIR}/diff.txt" <<'PY' > "${TMP_DIFF}"
import sys

src = sys.argv[1] if len(sys.argv) > 1 else None
if not src:
    sys.exit("Missing diff source path")

with open(src, "r", encoding="utf-8", errors="replace") as f:
    lines = f.readlines()

out = []
skip = False
for line in lines:
    if line.startswith("project "):
        continue
    if line.startswith("diff --git "):
        parts = line.strip().split()
        if len(parts) >= 4:
            a = parts[2]
            b = parts[3]
            a_path = a[2:] if a.startswith("a/") else a
            b_path = b[2:] if b.startswith("b/") else b
            # Skip linux-arm64/bin symlink hunks (handled via symlinks below)
            if a_path.startswith("linux-arm64/bin/") or b_path.startswith("linux-arm64/bin/"):
                skip = True
                continue
            if a_path.startswith("toolchain/llvm_android/"):
                a_path = a_path[len("toolchain/llvm_android/"):]
            if b_path.startswith("toolchain/llvm_android/"):
                b_path = b_path[len("toolchain/llvm_android/"):]
            out.append(f"diff --git a/{a_path} b/{b_path}\n")
            skip = False
            continue
    if skip:
        continue
    if line.startswith("--- ") or line.startswith("+++ "):
        prefix = line[:4]
        path = line[4:].strip()
        lead = path[:2] if path.startswith(("a/", "b/")) else ""
        p = path[2:] if lead else path
        if p.startswith("toolchain/llvm_android/"):
            p = p[len("toolchain/llvm_android/"):]
        if p.startswith("linux-arm64/bin/"):
            skip = True
            continue
        out.append(f"{prefix}{lead}{p}\n")
        continue
    out.append(line)

sys.stdout.write("".join(out))
PY
  APPLIED=0
  for STRIP in 1 0; do
    if git apply --check -p"${STRIP}" "${TMP_DIFF}" >/dev/null 2>&1; then
      git apply -p"${STRIP}" "${TMP_DIFF}"
      APPLIED=1
      break
    fi
    if git apply --check --reverse -p"${STRIP}" "${TMP_DIFF}" >/dev/null 2>&1; then
      echo "Patch already applied (git apply reverse check passed)."
      APPLIED=1
      break
    fi
  done
  if [ "${APPLIED}" = "0" ]; then
    for STRIP in 1 0; do
      if patch -p"${STRIP}" --dry-run --forward < "${TMP_DIFF}" >/dev/null 2>&1; then
        patch -p"${STRIP}" --forward --batch < "${TMP_DIFF}"
        APPLIED=1
        break
      fi
      if patch -p"${STRIP}" --dry-run -R < "${TMP_DIFF}" >/dev/null 2>&1; then
        echo "Patch already applied (patch reverse dry-run passed)."
        APPLIED=1
        break
      fi
    done
  fi
  rm -f "${TMP_DIFF}"
  if [ "${APPLIED}" = "0" ]; then
    echo "Failed to apply diff.txt cleanly. Aborting to avoid partial patch."
    exit 1
  fi
else
  echo "SnowNF repo does not contain build.py or diff.txt. Cannot apply."
  exit 1
fi

# Step 2 (cont): Create symlinks required by SnowNF README
mkdir -p "${LLVM_TOOLCHAIN_DIR}/prebuilts/build-tools/linux-arm64/bin"
ln -sf /bin/bison "${LLVM_TOOLCHAIN_DIR}/prebuilts/build-tools/linux-arm64/bin/bison"
ln -sf /bin/ninja "${LLVM_TOOLCHAIN_DIR}/prebuilts/build-tools/linux-arm64/bin/ninja"
mkdir -p "${LLVM_TOOLCHAIN_DIR}/prebuilts/clang/host"
ln -sfn /usr "${LLVM_TOOLCHAIN_DIR}/prebuilts/clang/host/linux-arm64"
mkdir -p "${LLVM_TOOLCHAIN_DIR}/prebuilts/cmake"
ln -sfn /usr "${LLVM_TOOLCHAIN_DIR}/prebuilts/cmake/linux-arm64"

# Python prebuilt: create a single-version layout to avoid multiple include dirs.
PY_ROOT="${LLVM_TOOLCHAIN_DIR}/prebuilts/python/linux-arm64"
rm -rf "${PY_ROOT}"
mkdir -p "${PY_ROOT}/bin" "${PY_ROOT}/include" "${PY_ROOT}/lib"
PY_VER="$(python3 - <<'PY'
import sys
print(f"{sys.version_info.major}.{sys.version_info.minor}")
PY
)"
PY_INC="/usr/include/python${PY_VER}"
PY_LIB_DIR="$(python3 - <<'PY'
import sysconfig
print(sysconfig.get_config_var('LIBDIR') or '')
PY
)"
PY_LDLIB="$(python3 - <<'PY'
import sysconfig
print(sysconfig.get_config_var('LDLIBRARY') or '')
PY
)"
if [ ! -d "${PY_INC}" ]; then
  echo "Python include dir not found: ${PY_INC} (install python3-dev)"
  exit 1
fi
if [ -z "${PY_LIB_DIR}" ] || [ -z "${PY_LDLIB}" ] || [ ! -f "${PY_LIB_DIR}/${PY_LDLIB}" ]; then
  echo "Python library not found: ${PY_LIB_DIR}/${PY_LDLIB} (install python3-dev)"
  exit 1
fi
ln -sf /usr/bin/python3 "${PY_ROOT}/bin/python3"
ln -sfn "${PY_INC}" "${PY_ROOT}/include/python${PY_VER}"
ln -sf "${PY_LIB_DIR}/${PY_LDLIB}" "${PY_ROOT}/lib/${PY_LDLIB}"

# Step 3: Build
if [ "${RUN_BUILD}" = "1" ]; then
  cd "${LLVM_TOOLCHAIN_DIR}"
  BUILD_LOG="${LLVM_TOOLCHAIN_DIR}/build_aarch64_ndk.log"
  BUILD_CMD=(python3 toolchain/llvm_android/build.py --no-build windows --skip-tests --no-musl)
  if python3 toolchain/llvm_android/do_build.py --help 2>&1 | grep -q -- '--single-stage'; then
    BUILD_CMD+=(--single-stage)
  fi
  if [ "${SKIP_SOURCE_SETUP}" = "1" ]; then
    BUILD_CMD+=(--skip-source-setup)
  fi
  set +e
  "${BUILD_CMD[@]}" 2>&1 | tee "${BUILD_LOG}"
  BUILD_STATUS=${PIPESTATUS[0]}
  set -e
  if [ "${BUILD_STATUS}" -ne 0 ]; then
    if grep -q "bits/libc-header-start.h" "${BUILD_LOG}"; then
      echo "Build stopped at libc-header-start.h (per README, clang core is built)."
    else
      exit "${BUILD_STATUS}"
    fi
  fi
fi

# Step 4: Replace android-ndk-rXX-linux binary files (optional)
if [ "${COPY_TO_NDK}" = "1" ]; then
  if [ -z "${NDK_ROOT}" ] || [ ! -d "${NDK_ROOT}" ]; then
    echo "NDK not found. Install Android SDK/NDK first (scripts/install_android_sdk_ubuntu.sh)."
    exit 1
  fi

  NDK_PREBUILT="${NDK_ROOT}/toolchains/llvm/prebuilt/${NDK_HOST_TAG}"
  if [ ! -d "${NDK_PREBUILT}" ]; then
    echo "NDK prebuilt directory not found: ${NDK_PREBUILT}"
    exit 1
  fi

  cp -fr "${LLVM_TOOLCHAIN_DIR}/out/stage2/bin/clang"* "${NDK_PREBUILT}/bin/"
  cp -fr "${LLVM_TOOLCHAIN_DIR}/out/stage2/lib/llvm-"* "${NDK_PREBUILT}/bin/" || true
  cp -fr "${LLVM_TOOLCHAIN_DIR}/out/stage2/lib/lld"* "${NDK_PREBUILT}/bin/" || true
  cp -fr "${LLVM_TOOLCHAIN_DIR}/out/stage2/lib/clang" "${NDK_PREBUILT}/lib/"

  if [ -d "${NDK_ROOT}/prebuilt/${NDK_HOST_TAG}/bin" ]; then
    cp -fr /usr/bin/make "${NDK_ROOT}/prebuilt/${NDK_HOST_TAG}/bin/"
    cp -fr /usr/bin/yasm "${NDK_ROOT}/prebuilt/${NDK_HOST_TAG}/bin/" || true
    cp -fr /usr/bin/ytasm "${NDK_ROOT}/prebuilt/${NDK_HOST_TAG}/bin/" || true
  fi
fi

# Auto-write environment for Dart usage (no manual export needed)
ENV_FILE="${HOME}/.lwrcl_android_env"
ANDROID_PREFIX="${ANDROID_PREFIX:-${LWRCL_ROOT}/lwrcl/android/${ANDROID_ABI}}"
FAST_DDS_PREFIX="${FAST_DDS_PREFIX:-${ANDROID_PREFIX}}"
if [ -n "${NDK_ROOT}" ] && [ -d "${NDK_ROOT}" ]; then
  ANDROID_NDK_HOME="${NDK_ROOT}"
  ANDROID_NDK_VERSION="$(basename "${NDK_ROOT}")"
fi

mkdir -p "$(dirname "${ENV_FILE}")"
cat > "${ENV_FILE}" <<EOF
# Auto-generated by lwrcl scripts/build_ndk_aarch64_linux.sh
export LWRCL_ROOT="${LWRCL_ROOT}"
export ANDROID_HOME="${ANDROID_HOME:-}"
export ANDROID_SDK_ROOT="${ANDROID_SDK_ROOT:-}"
export ANDROID_NDK_HOME="${ANDROID_NDK_HOME:-}"
export ANDROID_NDK_VERSION="${ANDROID_NDK_VERSION:-}"
export ANDROID_ABI="${ANDROID_ABI}"
export ANDROID_API="${ANDROID_API}"
export ANDROID_PREFIX="${ANDROID_PREFIX}"
export FAST_DDS_PREFIX="${FAST_DDS_PREFIX}"
export PATH="\$PATH:\$ANDROID_HOME/cmdline-tools/latest/bin:\$ANDROID_HOME/platform-tools"
EOF

for RC in "${HOME}/.bashrc" "${HOME}/.zshrc"; do
  if [ -f "${RC}" ] && ! grep -q "lwrcl_android_env" "${RC}"; then
    echo "" >> "${RC}"
    echo "# lwrcl_android_env" >> "${RC}"
    echo "if [ -f \"${ENV_FILE}\" ]; then . \"${ENV_FILE}\"; fi" >> "${RC}"
  fi
done

echo "Done."
