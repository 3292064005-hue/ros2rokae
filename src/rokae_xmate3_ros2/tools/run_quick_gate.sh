#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 <workspace-root>" >&2
  exit 64
fi

WS_ROOT="$1"
PKG_NAME="rokae_xmate3_ros2"

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  . "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "/opt/ros/humble/setup.bash"
fi
"${WS_ROOT}/src/${PKG_NAME}/tools/check_target_environment.sh" --quiet
"${WS_ROOT}/src/${PKG_NAME}/tools/run_static_sanity.sh"
"${WS_ROOT}/src/${PKG_NAME}/tools/clean_build_env.sh" colcon build --packages-select "${PKG_NAME}" --symlink-install
cd "${WS_ROOT}/build/${PKG_NAME}"
"${WS_ROOT}/src/${PKG_NAME}/tools/clean_build_env.sh" ctest -L quick_gate --output-on-failure
"${WS_ROOT}/src/${PKG_NAME}/tools/clean_build_env.sh" ctest -L semantic_gate --output-on-failure
