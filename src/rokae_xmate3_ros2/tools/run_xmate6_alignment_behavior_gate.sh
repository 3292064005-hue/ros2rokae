#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 <workspace-root>" >&2
  exit 64
fi

WS_ROOT="$1"
PKG_NAME="rokae_xmate3_ros2"
BUILD_DIR="${WS_ROOT}/build/${PKG_NAME}"
if [ ! -d "${BUILD_DIR}" ]; then
  echo "xmate6_alignment_gate: build directory not found: ${BUILD_DIR}" >&2
  exit 66
fi

cd "${BUILD_DIR}"
ctest -L xmate6_alignment --output-on-failure
