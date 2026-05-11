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
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -f "${INSTALL_SETUP}" ]; then
  echo "launch_smoke: workspace install setup not found: ${INSTALL_SETUP}" >&2
  exit 66
fi

# shellcheck disable=SC1090
. "${SCRIPT_DIR}/acceptance_cli_common.sh"
rokae_acceptance_source_env "${WS_ROOT}"

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

POLICY_FILE="${INSTALL_SHARE}/config/default_runtime_host_policy.env"
if [ -f "${POLICY_FILE}" ]; then
  rokae_acceptance_source_file "${POLICY_FILE}"
fi

PUBLIC_BACKEND_MODE="${ROKAE_DEFAULT_BACKEND_MODE:-jtc}"
PUBLIC_SERVICE_EXPOSURE_PROFILE="${ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE:-public_xmate_er3_only}"
PUBLIC_ENABLE_ROS2_CONTROL="${ROKAE_DEFAULT_ENABLE_ROS2_CONTROL:-true}"
PUBLIC_ENABLE_XCORE_PLUGIN="${ROKAE_DEFAULT_ENABLE_XCORE_PLUGIN:-true}"
PUBLIC_COMPATIBILITY_ALIAS_POLICY="${ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY:-canonical_only}"


"${PYTHON_BIN}" -m py_compile   "${INSTALL_SHARE}/launch/_simulation_support.py"   "${INSTALL_SHARE}/launch/_launch_profile.py"   "${INSTALL_SHARE}/launch/rviz_only.launch.py"   "${INSTALL_SHARE}/launch/simulation.launch.py"   "${INSTALL_SHARE}/launch/xmate_er3_public.launch.py"

XACRO_INPUT="${INSTALL_SHARE}/urdf/xMateER3.xacro"
if [ ! -f "${XACRO_INPUT}" ]; then
  echo "launch_smoke: installed xacro not found: ${XACRO_INPUT}" >&2
  exit 66
fi

CANONICAL_URDF="${INSTALL_SHARE}/generated/urdf/xMateER3.urdf"
if [ ! -f "${CANONICAL_URDF}" ]; then
  echo "launch_smoke: installed canonical URDF not found: ${CANONICAL_URDF}" >&2
  exit 66
fi

RENDERER="${INSTALL_SHARE}/tools/render_robot_description.py"
if [ ! -f "${RENDERER}" ]; then
  echo "launch_smoke: render helper not found: ${RENDERER}" >&2
  exit 66
fi

CANONICAL_METADATA="${INSTALL_SHARE}/generated/urdf/xMateER3.description.json"
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

xacro "${XACRO_INPUT}"   mesh_root:=model://rokae_xmate3_ros2/meshes/   package_share:="${INSTALL_SHARE}"   enable_ros2_control:=true   enable_xcore_plugin:=true   backend_mode:=hybrid   service_exposure_profile:=internal_full   compatibility_alias_policy:=canonical_plus_compat > "${XACRO_OUTPUT}"

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control "${PUBLIC_ENABLE_ROS2_CONTROL}"   --enable-xcore-plugin "${PUBLIC_ENABLE_XCORE_PLUGIN}"   --backend-mode "${PUBLIC_BACKEND_MODE}"   --service-exposure-profile "${PUBLIC_SERVICE_EXPOSURE_PROFILE}"   --compatibility-alias-policy "${PUBLIC_COMPATIBILITY_ALIAS_POLICY}"   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >"${CANONICAL_PUBLIC_OUTPUT}"
if ! grep -q "<backend_mode>${PUBLIC_BACKEND_MODE}</backend_mode>" "${CANONICAL_PUBLIC_OUTPUT}"; then
  echo "launch_smoke: canonical public render missing expected backend tag: ${PUBLIC_BACKEND_MODE}" >&2
  exit 70
fi
if ! grep -q "<service_exposure_profile>${PUBLIC_SERVICE_EXPOSURE_PROFILE}</service_exposure_profile>" "${CANONICAL_PUBLIC_OUTPUT}"; then
  echo "launch_smoke: canonical public render missing expected service exposure profile tag: ${PUBLIC_SERVICE_EXPOSURE_PROFILE}" >&2
  exit 70
fi
if ! grep -q "<compatibility_alias_policy>${PUBLIC_COMPATIBILITY_ALIAS_POLICY}</compatibility_alias_policy>" "${CANONICAL_PUBLIC_OUTPUT}"; then
  echo "launch_smoke: canonical public render missing expected compatibility alias policy tag: ${PUBLIC_COMPATIBILITY_ALIAS_POLICY}" >&2
  exit 70
fi
if ! grep -q 'model://rokae_xmate3_ros2/meshes/collision/xMateER3_base.stl' "${CANONICAL_PUBLIC_OUTPUT}"; then
  echo "launch_smoke: Gazebo public render did not rewrite collision meshes to model:// URIs" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --service-exposure-profile internal_full   --compatibility-alias-policy canonical_plus_compat   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >"${CANONICAL_INTERNAL_OUTPUT}"
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
if ! grep -q "<compatibility_alias_policy>canonical_plus_compat</compatibility_alias_policy>" "${CANONICAL_INTERNAL_OUTPUT}"; then
  echo "launch_smoke: canonical internal render missing compatibility alias policy tag" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${CANONICAL_URDF}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control "${PUBLIC_ENABLE_ROS2_CONTROL}"   --enable-xcore-plugin false   --backend-mode "${PUBLIC_BACKEND_MODE}"   --service-exposure-profile "${PUBLIC_SERVICE_EXPOSURE_PROFILE}"   --compatibility-alias-policy "${PUBLIC_COMPATIBILITY_ALIAS_POLICY}"   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >"${CANONICAL_NO_PLUGIN_OUTPUT}"
if grep -q "xcore_controller_gazebo_plugin" "${CANONICAL_NO_PLUGIN_OUTPUT}"; then
  echo "launch_smoke: canonical install-tree render kept xcore plugin despite enable_xcore_plugin:=false" >&2
  exit 70
fi

if "${PYTHON_BIN}" "${RENDERER}"   --model "${XACRO_INPUT}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --service-exposure-profile internal_full   --compatibility-alias-policy canonical_plus_compat   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model false >/dev/null 2>&1; then
  echo "launch_smoke: non-canonical xacro override unexpectedly succeeded without developer-mode opt-in" >&2
  exit 70
fi

"${PYTHON_BIN}" "${RENDERER}"   --model "${XACRO_INPUT}"   --package-share "${INSTALL_SHARE}"   --mesh-root model://rokae_xmate3_ros2/meshes/   --enable-ros2-control true   --enable-xcore-plugin true   --backend-mode hybrid   --service-exposure-profile internal_full   --compatibility-alias-policy canonical_plus_compat   --canonical-model "${CANONICAL_URDF}"   --canonical-metadata "${CANONICAL_METADATA}"   --allow-noncanonical-model true >/dev/null

ros2 pkg prefix "${PKG_NAME}" >/dev/null
ros2 launch "${PKG_NAME}" xmate3_simulation.launch.py --show-args >/dev/null
ros2 launch "${PKG_NAME}" simulation.launch.py --show-args >/dev/null
ros2 launch "${PKG_NAME}" rviz_only.launch.py --show-args >/dev/null
