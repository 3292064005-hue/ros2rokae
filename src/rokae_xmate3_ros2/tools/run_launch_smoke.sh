#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
usage: run_launch_smoke.sh <workspace-root>

Validate target-environment launch discovery without starting a full long-running simulation.
The script requires a built workspace install tree.
USAGE
}

if [ $# -lt 1 ]; then
  usage
  exit 64
fi

WS_ROOT="$1"
PKG_NAME="rokae_xmate3_ros2"
PKG_SRC="${WS_ROOT}/src/${PKG_NAME}"
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"

if [ ! -d "${PKG_SRC}" ]; then
  echo "launch_smoke: package source tree not found: ${PKG_SRC}" >&2
  exit 66
fi

if [ ! -f "${INSTALL_SETUP}" ]; then
  echo "launch_smoke: workspace install setup not found: ${INSTALL_SETUP}" >&2
  exit 66
fi

if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  . "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  . /opt/ros/humble/setup.bash
else
  echo "launch_smoke: ROS environment is not available" >&2
  exit 78
fi

# shellcheck disable=SC1090
. "${INSTALL_SETUP}"

PYTHON_BIN="${ROKAE_PYTHON_EXECUTABLE:-$(command -v python3 || true)}"
if [ -z "${PYTHON_BIN}" ]; then
  echo "launch_smoke: python3 not found" >&2
  exit 72
fi

INSTALL_SHARE="${WS_ROOT}/install/${PKG_NAME}/share/${PKG_NAME}"
if [ ! -d "${INSTALL_SHARE}" ]; then
  echo "launch_smoke: installed package share not found: ${INSTALL_SHARE}" >&2
  exit 66
fi

export ROKAE_XMATE3_ROS2_SHARE_DIR="${INSTALL_SHARE}"
export ROKAE_XMATE3_ROS2_LIB_DIR="${WS_ROOT}/install/${PKG_NAME}/lib"
export ROKAE_PYTHON_EXECUTABLE="${PYTHON_BIN}"
export LIBGL_ALWAYS_SOFTWARE=1

"${PYTHON_BIN}" -m py_compile   "${PKG_SRC}/launch/_simulation_support.py"   "${PKG_SRC}/launch/rviz_only.launch.py"   "${PKG_SRC}/launch/simulation.launch.py"   "${PKG_SRC}/launch/xmate3_gazebo.launch.py"   "${PKG_SRC}/launch/xmate3_simulation.launch.py"

XACRO_INPUT="${INSTALL_SHARE}/urdf/xMate3.xacro"
if [ ! -f "${XACRO_INPUT}" ]; then
  echo "launch_smoke: installed xacro not found: ${XACRO_INPUT}" >&2
  exit 66
fi

CANONICAL_URDF="${INSTALL_SHARE}/generated/urdf/xMate3.urdf"
if [ ! -f "${CANONICAL_URDF}" ]; then
  echo "launch_smoke: installed canonical URDF not found: ${CANONICAL_URDF}" >&2
  exit 66
fi

RENDERER="${INSTALL_SHARE}/tools/render_robot_description.py"
if [ ! -f "${RENDERER}" ]; then
  echo "launch_smoke: render helper not found: ${RENDERER}" >&2
  exit 66
fi

XACRO_OUTPUT="$(mktemp /tmp/xmate3_launch_smoke_XXXXXX.urdf)"
cleanup() {
  rm -f "${XACRO_OUTPUT}"
}
trap cleanup EXIT

xacro "${XACRO_INPUT}"   mesh_root:=model://rokae_xmate3_ros2/meshes/   package_share:="${INSTALL_SHARE}"   enable_ros2_control:=true   enable_xcore_plugin:=true   backend_mode:=hybrid > "${XACRO_OUTPUT}"

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --canonical-model "${CANONICAL_URDF}"   --allow-noncanonical-model false >/dev/null

if "${PYTHON_BIN}" "${RENDERER}"   --model "${XACRO_INPUT}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --canonical-model "${CANONICAL_URDF}"   --allow-noncanonical-model false >/dev/null 2>&1; then
  echo "launch_smoke: non-canonical xacro override unexpectedly succeeded without developer-mode opt-in" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${XACRO_INPUT}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --canonical-model "${CANONICAL_URDF}"   --allow-noncanonical-model true >/dev/null

ros2 pkg prefix "${PKG_NAME}" >/dev/null
ros2 launch "${PKG_NAME}" xmate3_simulation.launch.py --show-args >/dev/null
ros2 launch "${PKG_NAME}" simulation.launch.py --show-args >/dev/null
ros2 launch "${PKG_NAME}" rviz_only.launch.py --show-args >/dev/null
