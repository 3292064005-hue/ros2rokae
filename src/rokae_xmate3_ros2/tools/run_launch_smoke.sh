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

CANONICAL_METADATA="${INSTALL_SHARE}/generated/urdf/xMate3.description.json"
if [ ! -f "${CANONICAL_METADATA}" ]; then
  echo "launch_smoke: installed canonical metadata not found: ${CANONICAL_METADATA}" >&2
  exit 66
fi

XACRO_OUTPUT="$(mktemp /tmp/xmate3_launch_smoke_XXXXXX.urdf)"
CANONICAL_PUBLIC_OUTPUT="$(mktemp /tmp/xmate3_launch_smoke_public_XXXXXX.urdf)"
CANONICAL_INTERNAL_OUTPUT="$(mktemp /tmp/xmate3_launch_smoke_internal_XXXXXX.urdf)"
CANONICAL_NO_PLUGIN_OUTPUT="$(mktemp /tmp/xmate3_launch_smoke_noplugin_XXXXXX.urdf)"
cleanup() {
  rm -f "${XACRO_OUTPUT}" "${CANONICAL_PUBLIC_OUTPUT}" "${CANONICAL_INTERNAL_OUTPUT}" "${CANONICAL_NO_PLUGIN_OUTPUT}"
}
trap cleanup EXIT

xacro "${XACRO_INPUT}"   mesh_root:=model://rokae_xmate3_ros2/meshes/   package_share:="${INSTALL_SHARE}"   enable_ros2_control:=true   enable_xcore_plugin:=true   backend_mode:=hybrid   service_exposure_profile:=internal_full > "${XACRO_OUTPUT}"

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode jtc   --service-exposure-profile public_xmate6_only   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >"${CANONICAL_PUBLIC_OUTPUT}"
if ! grep -q "<backend_mode>jtc</backend_mode>" "${CANONICAL_PUBLIC_OUTPUT}"; then
  echo "launch_smoke: canonical public render missing jtc backend tag" >&2
  exit 70
fi
if ! grep -q "<service_exposure_profile>public_xmate6_only</service_exposure_profile>" "${CANONICAL_PUBLIC_OUTPUT}"; then
  echo "launch_smoke: canonical public render missing public exposure profile tag" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --service-exposure-profile internal_full   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >"${CANONICAL_INTERNAL_OUTPUT}"
if ! cmp -s "${XACRO_OUTPUT}" "${CANONICAL_INTERNAL_OUTPUT}"; then
  echo "launch_smoke: canonical install-tree re-render does not match xacro for hybrid/internal_full" >&2
  exit 70
fi
if ! grep -q "<backend_mode>hybrid</backend_mode>" "${CANONICAL_INTERNAL_OUTPUT}"; then
  echo "launch_smoke: canonical internal render missing hybrid backend tag" >&2
  exit 70
fi
if ! grep -q "<service_exposure_profile>internal_full</service_exposure_profile>" "${CANONICAL_INTERNAL_OUTPUT}"; then
  echo "launch_smoke: canonical internal render missing internal exposure profile tag" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin false   --backend-mode jtc   --service-exposure-profile public_xmate6_only   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >"${CANONICAL_NO_PLUGIN_OUTPUT}"
if grep -q "xcore_controller_gazebo_plugin" "${CANONICAL_NO_PLUGIN_OUTPUT}"; then
  echo "launch_smoke: canonical install-tree render kept xcore plugin despite enable_xcore_plugin:=false" >&2
  exit 70
fi

if "${PYTHON_BIN}" "${RENDERER}"   --model "${XACRO_INPUT}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >/dev/null 2>&1; then
  echo "launch_smoke: non-canonical xacro override unexpectedly succeeded without developer-mode opt-in" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${XACRO_INPUT}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model true >/dev/null

ros2 pkg prefix "${PKG_NAME}" >/dev/null
ros2 launch "${PKG_NAME}" xmate3_simulation.launch.py --show-args >/dev/null
ros2 launch "${PKG_NAME}" simulation.launch.py --show-args >/dev/null
ros2 launch "${PKG_NAME}" rviz_only.launch.py --show-args >/dev/null
