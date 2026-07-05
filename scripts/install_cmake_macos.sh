#!/usr/bin/env bash
set -euo pipefail

CMAKE_VERSION="${CMAKE_VERSION:-3.31.6}"
CMAKE_FORMULA_COMMIT="${CMAKE_FORMULA_COMMIT:-b4e46db74e74a8c1650b38b1da222284ce1ec5ce}"
CMAKE_TAP="${CMAKE_TAP:-lwrcl/old-cmake}"
BUILD_DIR="${BUILD_DIR:-${HOME}/build-cmake-${CMAKE_VERSION}}"
FORCE_REINSTALL="OFF"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --version <version>   Expected CMake version (default: ${CMAKE_VERSION})
  --commit <sha>        homebrew-core commit containing that formula
                        (default: ${CMAKE_FORMULA_COMMIT})
  --tap <name>          Local Homebrew tap for the old formula
                        (default: ${CMAKE_TAP})
  --build-dir <path>    Formula download directory (default: ${BUILD_DIR})
  --force               Reinstall even if CMake ${CMAKE_VERSION} is already present
  --help                Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      CMAKE_VERSION="$2"
      BUILD_DIR="${HOME}/build-cmake-${CMAKE_VERSION}"
      shift 2
      ;;
    --commit)
      CMAKE_FORMULA_COMMIT="$2"
      shift 2
      ;;
    --tap)
      CMAKE_TAP="$2"
      shift 2
      ;;
    --build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    --force)
      FORCE_REINSTALL="ON"
      shift
      ;;
    --help)
      usage
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "$(uname -s)" != "Darwin" ]]; then
  echo "[ERROR] This installer is only for macOS." >&2
  exit 1
fi

if ! command -v brew >/dev/null 2>&1; then
  echo "[ERROR] Homebrew is required to install CMake ${CMAKE_VERSION}." >&2
  exit 1
fi

if command -v cmake >/dev/null 2>&1; then
  CURRENT_VERSION="$(cmake --version | awk 'NR==1 {print $3}')"
  if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ "${CURRENT_VERSION}" == "${CMAKE_VERSION}" ]]; then
    brew pin --formula cmake || true
    echo "[INFO] CMake ${CMAKE_VERSION} is already active."
    exit 0
  fi
fi

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
FORMULA_FILE="${BUILD_DIR}/cmake.rb"
FORMULA_URL="https://raw.githubusercontent.com/Homebrew/homebrew-core/${CMAKE_FORMULA_COMMIT}/Formula/c/cmake.rb"

echo "[INFO] Downloading Homebrew formula: ${FORMULA_URL}"
curl -L --fail -o "${FORMULA_FILE}" "${FORMULA_URL}"

if ! brew tap | grep -qx "${CMAKE_TAP}"; then
  echo "[INFO] Creating local Homebrew tap: ${CMAKE_TAP}"
  brew tap-new "${CMAKE_TAP}"
  brew developer off || true
fi

TAP_REPO="$(brew --repository "${CMAKE_TAP}")"
mkdir -p "${TAP_REPO}/Formula"
cp "${FORMULA_FILE}" "${TAP_REPO}/Formula/cmake.rb"

if brew list --versions cmake >/dev/null 2>&1; then
  echo "[INFO] Uninstalling current Homebrew cmake before downgrade."
  brew uninstall --ignore-dependencies cmake
fi

echo "[INFO] Installing CMake ${CMAKE_VERSION} with Homebrew."
HOMEBREW_NO_AUTO_UPDATE=1 brew install "${CMAKE_TAP}/cmake"
brew link --overwrite --force cmake
brew pin --formula cmake || true

INSTALLED_VERSION="$(cmake --version | awk 'NR==1 {print $3}')"
if [[ "${INSTALLED_VERSION}" != "${CMAKE_VERSION}" ]]; then
  echo "[ERROR] Expected CMake ${CMAKE_VERSION}, but active cmake is ${INSTALLED_VERSION}." >&2
  exit 1
fi

echo "[OK] CMake ${INSTALLED_VERSION} is installed and pinned with Homebrew."
