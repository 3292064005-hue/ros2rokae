#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run_experimental_opt_in_smoke.sh [workspace-root]" >&2
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
if [[ ! -d "${WORKSPACE_ROOT}" ]]; then
  echo "experimental_opt_in_smoke: workspace root not found: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "experimental_opt_in_smoke: missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "experimental_opt_in_smoke: missing workspace setup: ${WS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "experimental_opt_in_smoke: ros2 CLI not available after sourcing setup files" >&2
  exit 1
fi

BUILD_LIB_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
if [[ -d "${BUILD_LIB_DIR}" ]]; then
  export ROKAE_XMATE3_ROS2_LIB_DIR="${BUILD_LIB_DIR}"
  export LD_LIBRARY_PATH="${BUILD_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi

LOG_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
LOG_FILE="${LOG_DIR}/experimental_opt_in_smoke.log"
mkdir -p "${LOG_DIR}"
SERVICE_CALL_TIMEOUT="${ROKAE_EXPERIMENTAL_SMOKE_SERVICE_CALL_TIMEOUT:-60}"
SERVICE_CALL_RETRIES="${ROKAE_EXPERIMENTAL_SMOKE_SERVICE_CALL_RETRIES:-3}"

MODEL_PATH="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/urdf/xMateER3.xacro"
if [[ ! -f "${MODEL_PATH}" ]]; then
  MODEL_PATH="${PACKAGE_ROOT}/urdf/xMateER3.xacro"
fi
if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "experimental_opt_in_smoke: missing xacro model for non-canonical launch" >&2
  exit 1
fi

echo "[experimental_opt_in_smoke] launching public_xmate_er3_experimental_rt..."
ros2 launch rokae_xmate3_ros2 simulation.launch.py \
  launch_profile:=public_xmate_er3_experimental_rt \
  gui:=false \
  rviz:=false \
  allow_noncanonical_model:=true \
  model:="${MODEL_PATH}" \
  enable_ros2_control:=true \
  enable_xcore_plugin:=true \
  backend_mode:=hybrid \
  service_exposure_profile:=public_xmate_er3_experimental \
  compatibility_alias_policy:=canonical_only >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

wait_for_service() {
  local name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if ros2 service list | grep -Fxq "${name}"; then
      echo "[experimental_opt_in_smoke] service ready: ${name}"
      return 0
    fi
    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "experimental_opt_in_smoke: timeout waiting for service ${name}" >&2
      echo "experimental_opt_in_smoke: launch log: ${LOG_FILE}" >&2
      return 1
    fi
    sleep 1
  done
}

assert_service_absent() {
  local name="$1"
  if ros2 service list | grep -Fxq "${name}"; then
    echo "experimental_opt_in_smoke: non-target service must stay absent: ${name}" >&2
    return 1
  fi
  echo "[experimental_opt_in_smoke] non-target service absent: ${name}"
}

LAST_SERVICE_RESPONSE=""
call_service_expect_success() {
  local step_label="$1"
  local service_name="$2"
  local service_type="$3"
  local payload="$4"

  echo "[experimental_opt_in_smoke] ${step_label}"
  local response
  local attempt
  for attempt in $(seq 1 "${SERVICE_CALL_RETRIES}"); do
    if response="$(timeout "${SERVICE_CALL_TIMEOUT}" ros2 service call "${service_name}" "${service_type}" "${payload}" 2>&1)" &&
        grep -q "success=True" <<< "${response}"; then
      echo "${response}"
      LAST_SERVICE_RESPONSE="${response}"
      return 0
    fi
    echo "${response}"
    if (( attempt < SERVICE_CALL_RETRIES )); then
      echo "experimental_opt_in_smoke: retrying ${service_name} (${attempt}/${SERVICE_CALL_RETRIES})" >&2
      sleep 1
    fi
  done
  LAST_SERVICE_RESPONSE="${response}"
  echo "experimental_opt_in_smoke: ${service_name} did not return a success response after ${SERVICE_CALL_RETRIES} attempt(s)" >&2
  return 1
}

require_last_response_contains() {
  local token="$1"
  local label="$2"
  if ! grep -q "${token}" <<< "${LAST_SERVICE_RESPONSE}"; then
    echo "experimental_opt_in_smoke: ${label} missing token: ${token}" >&2
    return 1
  fi
}

wait_for_service "/xmate_er3/cobot/connect" 120
wait_for_service "/xmate_er3/cobot/get_profile_capabilities" 60
wait_for_service "/xmate_er3/cobot/enable_drag" 60
wait_for_service "/xmate_er3/cobot/disable_drag" 60
wait_for_service "/xmate_er3/cobot/start_record_path" 60
wait_for_service "/xmate_er3/cobot/cancel_record_path" 60
wait_for_service "/xmate_er3/cobot/set_rt_control_mode" 60
wait_for_service "/xmate_er3/cobot/get_rt_joint_data" 60
wait_for_service "/xmate_er3/cobot/stop" 60
wait_for_service "/xmate_er3/cobot/move_reset" 60

assert_service_absent "/xmate3/io/get_di"
assert_service_absent "/xmate3/cobot/read_register"
assert_service_absent "/xmate3/cobot/load_rl_project"

call_service_expect_success "connect" \
  /xmate_er3/cobot/connect \
  rokae_xmate3_ros2/srv/Connect \
  "{remote_ip: '127.0.0.1', local_ip: '127.0.0.1'}"

call_service_expect_success "profile capabilities experimental" \
  /xmate_er3/cobot/get_profile_capabilities \
  rokae_xmate3_ros2/srv/GetProfileCapabilities \
  "{}"
require_last_response_contains "rt_sim_experimental_best_effort" "profile capabilities"
require_last_response_contains "best_effort_non_controller_grade" "profile capabilities"
require_last_response_contains "MoveSP=surface" "profile capabilities"

call_service_expect_success "set_operate_mode(manual)" \
  /xmate_er3/cobot/set_operate_mode \
  rokae_xmate3_ros2/srv/SetOperateMode \
  "{mode: 0}"

call_service_expect_success "enable_drag" \
  /xmate_er3/cobot/enable_drag \
  rokae_xmate3_ros2/srv/EnableDrag \
  "{space: 0, type: 0}"

call_service_expect_success "disable_drag" \
  /xmate_er3/cobot/disable_drag \
  rokae_xmate3_ros2/srv/DisableDrag \
  "{}"

call_service_expect_success "stop(after drag probe)" \
  /xmate_er3/cobot/stop \
  rokae_xmate3_ros2/srv/Stop \
  "{}"

call_service_expect_success "move_reset(after drag probe)" \
  /xmate_er3/cobot/move_reset \
  rokae_xmate3_ros2/srv/MoveReset \
  "{}"

call_service_expect_success "set_power_state(on before path record)" \
  /xmate_er3/cobot/set_power_state \
  rokae_xmate3_ros2/srv/SetPowerState \
  "{'on': true}"

call_service_expect_success "start_record_path" \
  /xmate_er3/cobot/start_record_path \
  rokae_xmate3_ros2/srv/StartRecordPath \
  "{duration: 1}"

call_service_expect_success "cancel_record_path" \
  /xmate_er3/cobot/cancel_record_path \
  rokae_xmate3_ros2/srv/CancelRecordPath \
  "{}"

call_service_expect_success "set_power_state(on)" \
  /xmate_er3/cobot/set_power_state \
  rokae_xmate3_ros2/srv/SetPowerState \
  "{'on': true}"

call_service_expect_success "set_operate_mode(automatic)" \
  /xmate_er3/cobot/set_operate_mode \
  rokae_xmate3_ros2/srv/SetOperateMode \
  "{mode: 1}"

call_service_expect_success "set_rt_control_mode(joint position)" \
  /xmate_er3/cobot/set_rt_control_mode \
  rokae_xmate3_ros2/srv/SetRtControlMode \
  "{mode: 0}"

call_service_expect_success "get_rt_joint_data" \
  /xmate_er3/cobot/get_rt_joint_data \
  rokae_xmate3_ros2/srv/GetRtJointData \
  "{}"

call_service_expect_success "disconnect" \
  /xmate_er3/cobot/disconnect \
  rokae_xmate3_ros2/srv/Disconnect \
  "{}"

echo "[experimental_opt_in_smoke] success"
echo "[experimental_opt_in_smoke] launch log: ${LOG_FILE}"
