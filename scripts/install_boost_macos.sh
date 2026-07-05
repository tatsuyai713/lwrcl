#!/usr/bin/env bash
set -euo pipefail

BOOST_VERSION="${BOOST_VERSION:-1.85.0}"
BOOST_FORMULA="${BOOST_FORMULA:-boost@1.85}"
BOOST_FORMULA_COMMIT="${BOOST_FORMULA_COMMIT:-2d85d3c03e9a2f925a978621f5bae68a4c6c295f}"
BOOST_TAP="${BOOST_TAP:-lwrcl/old-boost}"
BUILD_DIR="${BUILD_DIR:-${HOME}/build-${BOOST_FORMULA}}"
FORCE_REINSTALL="OFF"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --version <version>   Expected Boost version (default: ${BOOST_VERSION})
  --formula <name>      Boost formula name (default: ${BOOST_FORMULA})
  --commit <sha>        homebrew-core commit containing that formula
                        (default: ${BOOST_FORMULA_COMMIT})
  --tap <name>          Local Homebrew tap for the old formula
                        (default: ${BOOST_TAP})
  --build-dir <path>    Formula download directory (default: ${BUILD_DIR})
  --force               Reinstall even if Boost ${BOOST_VERSION} is already present
  --help                Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      BOOST_VERSION="$2"
      shift 2
      ;;
    --formula)
      BOOST_FORMULA="$2"
      BUILD_DIR="${HOME}/build-${BOOST_FORMULA}"
      shift 2
      ;;
    --commit)
      BOOST_FORMULA_COMMIT="$2"
      shift 2
      ;;
    --tap)
      BOOST_TAP="$2"
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
  echo "[ERROR] Homebrew is required to install ${BOOST_FORMULA}." >&2
  exit 1
fi

if [[ "${FORCE_REINSTALL}" != "ON" ]] && brew list --versions "${BOOST_FORMULA}" >/dev/null 2>&1; then
  BOOST_PREFIX="$(brew --prefix "${BOOST_FORMULA}")"
  echo "[INFO] ${BOOST_FORMULA} is already installed at ${BOOST_PREFIX}."
  exit 0
fi

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
FORMULA_FILE="${BUILD_DIR}/${BOOST_FORMULA}.rb"
FORMULA_URL="https://raw.githubusercontent.com/Homebrew/homebrew-core/${BOOST_FORMULA_COMMIT}/Formula/b/${BOOST_FORMULA}.rb"

echo "[INFO] Downloading Homebrew formula: ${FORMULA_URL}"
curl -L --fail -o "${FORMULA_FILE}" "${FORMULA_URL}"

# Homebrew disables versioned formulae after a grace period. Keep the historical
# formula local to this project so vsomeip can continue using the older Boost.
perl -0pi -e 's/^\s*deprecate!.*\n//mg; s/^\s*disable!.*\n//mg;' "${FORMULA_FILE}"

if ! brew tap | grep -qx "${BOOST_TAP}"; then
  echo "[INFO] Creating local Homebrew tap: ${BOOST_TAP}"
  brew tap-new "${BOOST_TAP}"
  brew developer off || true
fi

TAP_REPO="$(brew --repository "${BOOST_TAP}")"
mkdir -p "${TAP_REPO}/Formula"
cp "${FORMULA_FILE}" "${TAP_REPO}/Formula/${BOOST_FORMULA}.rb"

if [[ "${FORCE_REINSTALL}" == "ON" ]] && brew list --versions "${BOOST_FORMULA}" >/dev/null 2>&1; then
  echo "[INFO] Uninstalling current ${BOOST_FORMULA} before reinstall."
  brew uninstall --ignore-dependencies "${BOOST_FORMULA}"
fi

echo "[INFO] Installing ${BOOST_FORMULA} ${BOOST_VERSION} with Homebrew."
HOMEBREW_NO_AUTO_UPDATE=1 brew install "${BOOST_TAP}/${BOOST_FORMULA}"

BOOST_PREFIX="$(brew --prefix "${BOOST_FORMULA}")"
BOOST_VERSION_HPP="${BOOST_PREFIX}/include/boost/version.hpp"
INSTALLED_VERSION="$(
  awk '/#define BOOST_LIB_VERSION/ {gsub(/"/, "", $3); gsub(/_/, ".", $3); print $3}' "${BOOST_VERSION_HPP}"
)"

if [[ "${INSTALLED_VERSION}" != "${BOOST_VERSION}" && "${INSTALLED_VERSION}.0" != "${BOOST_VERSION}" ]]; then
  echo "[ERROR] Expected Boost ${BOOST_VERSION}, but installed Boost is ${INSTALLED_VERSION}." >&2
  exit 1
fi

echo "[OK] Boost ${INSTALLED_VERSION} is installed at ${BOOST_PREFIX}."
