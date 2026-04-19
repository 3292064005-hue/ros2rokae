#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 <workspace-root>" >&2
  exit 64
fi

WS_ROOT="$1"
PKG_NAME="rokae_xmate3_ros2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_TOOLS="${WS_ROOT}/src/${PKG_NAME}/tools"

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  . "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "/opt/ros/humble/setup.bash"
fi
"${SCRIPT_DIR}/check_target_environment.sh" --quiet
"${SOURCE_TOOLS}/run_static_sanity.sh"
"${SCRIPT_DIR}/clean_build_env.sh" colcon build --packages-select "${PKG_NAME}" --symlink-install
cd "${WS_ROOT}/build/${PKG_NAME}"
"${SCRIPT_DIR}/clean_build_env.sh" ctest -L quick_gate --output-on-failure
"${SCRIPT_DIR}/clean_build_env.sh" ctest -L semantic_gate --output-on-failure
"${SCRIPT_DIR}/run_xmate6_alignment_behavior_gate.sh" "${WS_ROOT}"
