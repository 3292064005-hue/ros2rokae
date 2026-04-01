#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: run_target_env_acceptance.sh [--engine docker|podman] [--image-tag TAG] [--release-gate] [--launch-smoke]

Build the locked Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 image and execute the package gates
inside an isolated target-like container.
EOF
}

ENGINE="${ROKAE_CONTAINER_ENGINE:-docker}"
IMAGE_TAG="${ROKAE_ACCEPTANCE_IMAGE_TAG:-rokae_xmate3_ros2:humble-gazebo11}"
RUN_RELEASE_GATE=0
RUN_LAUNCH_SMOKE=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --engine)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      ENGINE="$2"
      shift 2
      ;;
    --image-tag)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      IMAGE_TAG="$2"
      shift 2
      ;;
    --release-gate)
      RUN_RELEASE_GATE=1
      shift
      ;;
    --launch-smoke)
      RUN_LAUNCH_SMOKE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      exit 64
      ;;
  esac
done

if ! command -v "$ENGINE" >/dev/null 2>&1; then
  echo "acceptance: container engine '$ENGINE' is not available" >&2
  exit 69
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TMP_WS_REL="/tmp/rokae_acceptance_ws"

"$ENGINE" build -f "${PKG_ROOT}/docker/Dockerfile.humble-gazebo11" -t "${IMAGE_TAG}" "${PKG_ROOT}"

CONTAINER_SCRIPT=$(cat <<'EOF'
set -euo pipefail
. /opt/ros/humble/setup.bash
rm -rf /tmp/rokae_acceptance_ws
mkdir -p /tmp/rokae_acceptance_ws/src
cp -R /package /tmp/rokae_acceptance_ws/src/rokae_xmate3_ros2
cd /tmp/rokae_acceptance_ws
rosdep install --from-paths src --ignore-src -r -y
/tmp/rokae_acceptance_ws/src/rokae_xmate3_ros2/tools/run_quick_gate.sh /tmp/rokae_acceptance_ws
if [ "${RUN_RELEASE_GATE}" = "1" ]; then
  /tmp/rokae_acceptance_ws/src/rokae_xmate3_ros2/tools/run_release_gate.sh /tmp/rokae_acceptance_ws
fi
if [ "${RUN_LAUNCH_SMOKE}" = "1" ]; then
  python3 -m py_compile /tmp/rokae_acceptance_ws/src/rokae_xmate3_ros2/launch/_simulation_support.py \
    /tmp/rokae_acceptance_ws/src/rokae_xmate3_ros2/launch/*.launch.py
fi
EOF
)

"$ENGINE" run --rm \
  -e RUN_RELEASE_GATE="${RUN_RELEASE_GATE}" \
  -e RUN_LAUNCH_SMOKE="${RUN_LAUNCH_SMOKE}" \
  -v "${PKG_ROOT}:/package:ro" \
  "${IMAGE_TAG}" \
  bash -lc "${CONTAINER_SCRIPT}"
