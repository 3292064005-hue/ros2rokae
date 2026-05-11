#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOUSAGE'
usage: run_full_source_tree_build_gate.sh <workspace-root-or-package-root> [--build-dir DIR] [--install-dir DIR] [--ctest-labels LABELS]

Run the non-replay full source-tree build gate in the locked ROS 2 Humble/Gazebo target environment.
This gate intentionally rejects replay-only mode and validates the real source targets:
  1. target environment lock
  2. colcon build --packages-select rokae_xmate3_ros2 with ROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF
  3. ctest on the requested labels from the package build directory

It does not perform real robot / hardware validation.
EOUSAGE
}

if [ "$#" -lt 1 ]; then
  usage
  exit 64
fi

WORKSPACE_ROOT=""
BUILD_DIR=""
INSTALL_DIR=""
CTEST_LABELS="quick_gate;semantic_gate;contract_gate"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --build-dir)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      BUILD_DIR="$2"
      shift 2
      ;;
    --install-dir)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      INSTALL_DIR="$2"
      shift 2
      ;;
    --ctest-labels)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      CTEST_LABELS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [ -z "${WORKSPACE_ROOT}" ]; then
        WORKSPACE_ROOT="$1"
        shift
      else
        echo "unknown argument: $1" >&2
        usage
        exit 64
      fi
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_NAME="rokae_xmate3_ros2"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INPUT_ROOT="$(cd "${WORKSPACE_ROOT}" && pwd)"
TEMP_WORKSPACE=""
if [ -d "${INPUT_ROOT}/src/${PKG_NAME}" ]; then
  WORKSPACE_ROOT="${INPUT_ROOT}"
  PACKAGE_ROOT="${INPUT_ROOT}/src/${PKG_NAME}"
elif [ -f "${INPUT_ROOT}/package.xml" ] && grep -q "<name>${PKG_NAME}</name>" "${INPUT_ROOT}/package.xml"; then
  TEMP_WORKSPACE="$(mktemp -d)"
  mkdir -p "${TEMP_WORKSPACE}/src"
  ln -s "${INPUT_ROOT}" "${TEMP_WORKSPACE}/src/${PKG_NAME}"
  WORKSPACE_ROOT="${TEMP_WORKSPACE}"
  PACKAGE_ROOT="${INPUT_ROOT}"
else
  echo "full-source-build-gate: expected a ROS2 workspace root containing src/${PKG_NAME}, or the ${PKG_NAME} package root" >&2
  exit 64
fi
cleanup_full_source_gate() {
  if [ -n "${TEMP_WORKSPACE}" ]; then
    rm -rf "${TEMP_WORKSPACE}"
  fi
}
trap cleanup_full_source_gate EXIT
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build}"
INSTALL_DIR="${INSTALL_DIR:-${WORKSPACE_ROOT}/install}"

if [ "${ROKAE_PUBLIC_SDK_REPLAY_ONLY:-OFF}" = "ON" ] || [ "${ROKAE_PUBLIC_SDK_REPLAY_ONLY:-0}" = "1" ]; then
  echo "full-source-build-gate: ROKAE_PUBLIC_SDK_REPLAY_ONLY must be OFF for this gate" >&2
  exit 65
fi

# shellcheck disable=SC1090
. "${PACKAGE_ROOT}/tools/acceptance_cli_common.sh"
rokae_acceptance_source_ros_env

"${PACKAGE_ROOT}/tools/check_target_environment.sh" --quiet
"${PACKAGE_ROOT}/tools/run_static_sanity.sh"

"${PACKAGE_ROOT}/tools/clean_build_env.sh" \
  colcon build \
    --base-paths "${WORKSPACE_ROOT}/src" \
    --build-base "${BUILD_DIR}" \
    --install-base "${INSTALL_DIR}" \
    --packages-select "${PKG_NAME}" \
    --symlink-install \
    --cmake-args -DROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF -DBUILD_TESTING=ON

BUILD_PKG_DIR="${BUILD_DIR}/${PKG_NAME}"
if [ ! -d "${BUILD_PKG_DIR}" ]; then
  echo "full-source-build-gate: package build directory not found: ${BUILD_PKG_DIR}" >&2
  exit 66
fi

IFS=';' read -r -a LABELS <<< "${CTEST_LABELS}"
for label in "${LABELS[@]}"; do
  [ -n "${label}" ] || continue
  "${PACKAGE_ROOT}/tools/clean_build_env.sh" ctest --test-dir "${BUILD_PKG_DIR}" -L "${label}" --output-on-failure
done

echo "full-source-build-gate: passed (non-replay source build and ctest labels: ${CTEST_LABELS})"
