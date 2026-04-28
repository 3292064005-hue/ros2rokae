#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOUSAGE'
usage: run_release_gate_portable.sh [workspace-root] [--engine docker|podman] [--report-dir DIR]

Run the release gate on any host.
- If the current shell is already on the locked target environment and the workspace is built,
  execute tools/run_release_gate.sh directly.
- Otherwise, fall back to tools/run_target_env_acceptance.sh --release-gate --launch-smoke,
  which builds the locked Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 container and runs the same
  bundle there.
EOUSAGE
}

WORKSPACE_ROOT=""
ENGINE="${ROKAE_CONTAINER_ENGINE:-docker}"
REPORT_DIR=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --engine)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      ENGINE="$2"
      shift 2
      ;;
    --report-dir)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      REPORT_DIR="$2"
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
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -z "${WORKSPACE_ROOT}" ]; then
  WORKSPACE_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"
fi
if [ -z "${REPORT_DIR}" ]; then
  REPORT_DIR="${PKG_ROOT}/artifacts/portable_release_gate"
fi
mkdir -p "${REPORT_DIR}"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash
fi

if "${PKG_ROOT}/tools/check_target_environment.sh" --quiet >/dev/null 2>&1 && \
   [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  echo "[portable_release_gate] locked target environment detected; running release gate locally"
  exec "${PKG_ROOT}/tools/run_release_gate.sh" "${WORKSPACE_ROOT}"
fi

echo "[portable_release_gate] local target environment unavailable; delegating to locked container acceptance"
exec "${PKG_ROOT}/tools/run_target_env_acceptance.sh" \
  --engine "${ENGINE}" \
  --release-gate \
  --launch-smoke \
  --report-dir "${REPORT_DIR}"
