#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run_nrt_path_smoothness_gate.sh [workspace-root] [headless|gazebo|both]" >&2
  echo "  default mode: both (headless strict + gazebo diagnostic)" >&2
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -f "${PACKAGE_ROOT}/CMakeLists.txt" ]]; then
  DEFAULT_WORKSPACE_ROOT="$(cd "${PACKAGE_ROOT}/../.." && pwd)"
else
  DEFAULT_WORKSPACE_ROOT="$(cd "${PACKAGE_ROOT}/../../../.." && pwd)"
fi
WORKSPACE_ROOT="${1:-${DEFAULT_WORKSPACE_ROOT}}"
GATE_MODE="${2:-both}"

case "${GATE_MODE}" in
  headless|gazebo|both) ;;
  *)
    echo "nrt_path_smoothness_gate: unsupported mode=${GATE_MODE} (expect headless|gazebo|both)" >&2
    exit 64
    ;;
esac

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "nrt_path_smoothness_gate: missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "nrt_path_smoothness_gate: missing workspace setup: ${WS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "nrt_path_smoothness_gate: ros2 CLI not available after sourcing setup files" >&2
  exit 1
fi

BUILD_LIB_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
if [[ -d "${BUILD_LIB_DIR}" ]]; then
  export ROKAE_XMATE3_ROS2_LIB_DIR="${BUILD_LIB_DIR}"
  export LD_LIBRARY_PATH="${BUILD_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi
export PYTHONDONTWRITEBYTECODE=1

LOG_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
mkdir -p "${LOG_DIR}"
PROBE="${PACKAGE_ROOT}/test/harness/nrt_path_smoothness_probe.py"
if [[ ! -f "${PROBE}" ]]; then
  echo "nrt_path_smoothness_gate: missing probe: ${PROBE}" >&2
  exit 1
fi

MODEL_PATH="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/urdf/xMateER3.xacro"
if [[ ! -f "${MODEL_PATH}" ]]; then
  MODEL_PATH="${PACKAGE_ROOT}/urdf/xMateER3.xacro"
fi
if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "nrt_path_smoothness_gate: missing xacro model for Gazebo launch" >&2
  exit 1
fi

LAUNCH_PID=0
cleanup_launch() {
  if [[ "${LAUNCH_PID}" -ne 0 ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  LAUNCH_PID=0
}
trap cleanup_launch EXIT

run_probe() {
  local mode="$1"
  local output_log="${LOG_DIR}/nrt_path_smoothness_${mode}_probe.log"
  set +e
  timeout 90 "${ROKAE_PYTHON_EXECUTABLE:-python3}" "${PROBE}" \
    --mode "${mode}" \
    --namespace /xmate_er3 | tee "${output_log}"
  local status=${PIPESTATUS[0]}
  set -e
  return "${status}"
}

launch_headless() {
  local launch_log="${LOG_DIR}/nrt_path_smoothness_headless_launch.log"
  local runtime_exe="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/bin/rokae_sim_runtime"
  if [[ ! -x "${runtime_exe}" ]]; then
    runtime_exe="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/rokae_sim_runtime"
  fi
  if [[ ! -x "${runtime_exe}" ]]; then
    echo "nrt_path_smoothness_gate: missing runtime daemon executable rokae_sim_runtime" >&2
    return 1
  fi
  echo "[nrt_path_smoothness_gate] launching headless daemon NRT runtime..."
  "${runtime_exe}" \
    --ros-args \
    -p runtime_profile:=nrt_strict_parity \
    -p service_exposure_profile:=public_xmate_er3_only \
    -p compatibility_alias_policy:=canonical_only >"${launch_log}" 2>&1 &
  LAUNCH_PID=$!
}

launch_gazebo() {
  local launch_log="${LOG_DIR}/nrt_path_smoothness_gazebo_launch.log"
  echo "[nrt_path_smoothness_gate] launching Gazebo/JTC diagnostic runtime..."
  GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11460}" \
    ros2 launch rokae_xmate3_ros2 simulation.launch.py \
      launch_profile:=public_xmate_er3_jtc \
      gui:=false \
      rviz:=false \
      verbose:=false \
      allow_noncanonical_model:=true \
      model:="${MODEL_PATH}" \
      enable_ros2_control:=true \
      enable_xcore_plugin:=true \
      backend_mode:=jtc \
      service_exposure_profile:=public_xmate_er3_only \
      compatibility_alias_policy:=canonical_only >"${launch_log}" 2>&1 &
  LAUNCH_PID=$!
}

run_headless_gate() {
  launch_headless
  if ! run_probe headless; then
    echo "nrt_path_smoothness_gate: FAIL tier=headless classification=strict_smoothness_failure" >&2
    cleanup_launch
    return 1
  fi
  echo "nrt_path_smoothness_gate: PASS tier=headless classification=pass"
  cleanup_launch
}

run_gazebo_diagnostic() {
  launch_gazebo
  if run_probe gazebo; then
    echo "nrt_path_smoothness_gate: PASS tier=gazebo classification=pass"
    cleanup_launch
    return 0
  fi
  echo "nrt_path_smoothness_gate: DIAGNOSTIC tier=gazebo classification=gazebo_diagnostic_smoothness" >&2
  cleanup_launch
  return 0
}

HEADLESS_STATUS=0
case "${GATE_MODE}" in
  headless)
    run_headless_gate
    ;;
  gazebo)
    run_gazebo_diagnostic
    ;;
  both)
    run_headless_gate || HEADLESS_STATUS=$?
    run_gazebo_diagnostic || true
    exit "${HEADLESS_STATUS}"
    ;;
esac
