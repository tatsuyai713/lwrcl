#!/usr/bin/env bash
set -euo pipefail

FAST_DDS_GEN_TAG="2.5.2"
GRADLE_TAG="7.6.3"
INSTALL_PREFIX="/opt/fast-dds-gen"
GRADLE_PREFIX="/opt/gradle"
BUILD_DIR="${HOME}/build-fast-dds-gen"
SKIP_SYSTEM_DEPS="OFF"
FORCE_REINSTALL="OFF"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --prefix <path>       Install prefix (default: ${INSTALL_PREFIX})
  --tag <version>       Fast-DDS-Gen version (default: ${FAST_DDS_GEN_TAG})
  --gradle-tag <ver>    Gradle version (default: ${GRADLE_TAG})
  --build-dir <path>    Build directory (default: \${HOME}/build-fast-dds-gen)
  --skip-system-deps    Skip installing system dependencies
  --force               Force reinstall even if already present
  --help                Show this help message
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      INSTALL_PREFIX="$2"
      shift 2
      ;;
    --tag)
      FAST_DDS_GEN_TAG="$2"
      shift 2
      ;;
    --gradle-tag)
      GRADLE_TAG="$2"
      shift 2
      ;;
    --build-dir)
      BUILD_DIR="$2"
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
    --help)
      usage
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ "${FORCE_REINSTALL}" != "ON" ]] && [[ -x "${INSTALL_PREFIX}/bin/fastddsgen" ]]; then
  echo "[INFO] Fast-DDS-Gen already installed at ${INSTALL_PREFIX}. Skipping (use --force to reinstall)."
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
  if ! dpkg -l | grep -qw openjdk-11-jre || ! dpkg -l | grep -qw openjdk-11-jdk; then
    ${SUDO} apt-get update -qq
    ${SUDO} apt-get purge -y openjdk-* default-jdk default-jre --autoremove || true
    ${SUDO} apt-get install -y openjdk-11-jre openjdk-11-jdk
  fi
fi

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Install Gradle
${SUDO} rm -rf "${GRADLE_PREFIX}"
${SUDO} mkdir -p "${GRADLE_PREFIX}"
cd "${GRADLE_PREFIX}"
${SUDO} wget "https://services.gradle.org/distributions/gradle-${GRADLE_TAG}-bin.zip"
${SUDO} unzip "gradle-${GRADLE_TAG}-bin.zip"
${SUDO} rm -f "gradle-${GRADLE_TAG}-bin.zip"

export JAVA_HOME="$(readlink -f /usr/bin/java | sed "s:bin/java::")"
export PATH="${PATH}:${GRADLE_PREFIX}/gradle-${GRADLE_TAG}/bin"

# Install Fast-DDS-Gen
cd "${BUILD_DIR}"
${SUDO} rm -rf "${INSTALL_PREFIX}"
${SUDO} mkdir -p "${INSTALL_PREFIX}"
git clone --recursive -b "v${FAST_DDS_GEN_TAG}" https://github.com/eProsima/Fast-DDS-Gen.git fast-dds-gen
cd fast-dds-gen
gradle assemble
${SUDO} "${GRADLE_PREFIX}/gradle-${GRADLE_TAG}/bin/gradle" install --install_path="${INSTALL_PREFIX}"

# Update bashrc
sed -i -e "/export PATH=.*gradle.*\/bin/d" ~/.bashrc
echo "export PATH=\$PATH:${GRADLE_PREFIX}/gradle-${GRADLE_TAG}/bin" >> ~/.bashrc

sed -i -e '/export JAVA_HOME=/d' ~/.bashrc
echo 'export JAVA_HOME=$(readlink -f /usr/bin/java | sed "s:bin/java::")' >> ~/.bashrc

sed -i -e "/export PATH=.*fast-dds-gen\/bin/d" ~/.bashrc
echo "export PATH=\$PATH:${INSTALL_PREFIX}/bin" >> ~/.bashrc

${SUDO} ldconfig

echo "[OK] Installed Fast-DDS-Gen v${FAST_DDS_GEN_TAG} to ${INSTALL_PREFIX}"
