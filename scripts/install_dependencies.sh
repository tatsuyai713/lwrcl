#!/bin/bash
set -euo pipefail

if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y cmake build-essential libboost-dev libopencv-dev
elif command -v brew >/dev/null 2>&1; then
  brew update
  brew install git openssl@3 automake autoconf libtool bison flex
  "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_boost_macos.sh"
  "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/install_cmake_macos.sh"
else
  echo "Unsupported package manager. Install cmake, git, boost, OpenSSL, automake, autoconf, libtool, bison, and flex." >&2
  exit 1
fi
