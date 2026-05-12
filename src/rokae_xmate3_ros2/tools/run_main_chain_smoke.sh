#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run_main_chain_smoke.sh [workspace-root]" >&2
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
  echo "main_chain_smoke: workspace root not found: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "main_chain_smoke: missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "main_chain_smoke: missing workspace setup: ${WS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "main_chain_smoke: ros2 CLI not available after sourcing setup files" >&2
  exit 1
fi

BUILD_LIB_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
if [[ -d "${BUILD_LIB_DIR}" ]]; then
  export ROKAE_XMATE3_ROS2_LIB_DIR="${BUILD_LIB_DIR}"
  export LD_LIBRARY_PATH="${BUILD_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi

LOG_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
SMOKE_LABEL="${ROKAE_MAIN_CHAIN_SMOKE_LABEL:-main_chain_smoke}"
LOG_FILE="${LOG_DIR}/${SMOKE_LABEL}.log"
mkdir -p "${LOG_DIR}"

LAUNCH_PROFILE="${ROKAE_MAIN_CHAIN_LAUNCH_PROFILE:-public_xmate_er3_jtc}"
BACKEND_MODE="${ROKAE_MAIN_CHAIN_BACKEND_MODE:-jtc}"
SERVICE_EXPOSURE_PROFILE="${ROKAE_MAIN_CHAIN_SERVICE_EXPOSURE_PROFILE:-public_xmate_er3_only}"
ENABLE_ROS2_CONTROL="${ROKAE_MAIN_CHAIN_ENABLE_ROS2_CONTROL:-true}"
ENABLE_XCORE_PLUGIN="${ROKAE_MAIN_CHAIN_ENABLE_XCORE_PLUGIN:-true}"
COMPATIBILITY_ALIAS_POLICY="${ROKAE_MAIN_CHAIN_COMPATIBILITY_ALIAS_POLICY:-canonical_only}"
SERVICE_CALL_TIMEOUT="${ROKAE_MAIN_CHAIN_SERVICE_CALL_TIMEOUT:-60}"
SERVICE_CALL_RETRIES="${ROKAE_MAIN_CHAIN_SERVICE_CALL_RETRIES:-3}"
MOVE_TARGET_JOINTS="${ROKAE_MAIN_CHAIN_MOVE_TARGET_JOINTS:-[0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926]}"
MOVE_TARGET_SPEED="${ROKAE_MAIN_CHAIN_MOVE_TARGET_SPEED:-80}"
MOVE_TARGET_ZONE="${ROKAE_MAIN_CHAIN_MOVE_TARGET_ZONE:-5}"

MODEL_PATH="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/urdf/xMateER3.xacro"
if [[ ! -f "${MODEL_PATH}" ]]; then
  MODEL_PATH="${PACKAGE_ROOT}/urdf/xMateER3.xacro"
fi
if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "main_chain_smoke: missing xacro model for non-canonical launch" >&2
  exit 1
fi

echo "[main_chain_smoke] launching simulation profile=${LAUNCH_PROFILE} backend=${BACKEND_MODE} exposure=${SERVICE_EXPOSURE_PROFILE}..."
ros2 launch rokae_xmate3_ros2 simulation.launch.py \
  launch_profile:="${LAUNCH_PROFILE}" \
  gui:=false \
  rviz:=false \
  allow_noncanonical_model:=true \
  model:="${MODEL_PATH}" \
  enable_ros2_control:="${ENABLE_ROS2_CONTROL}" \
  enable_xcore_plugin:="${ENABLE_XCORE_PLUGIN}" \
  backend_mode:="${BACKEND_MODE}" \
  service_exposure_profile:="${SERVICE_EXPOSURE_PROFILE}" \
  compatibility_alias_policy:="${COMPATIBILITY_ALIAS_POLICY}" >"${LOG_FILE}" 2>&1 &
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
      echo "[main_chain_smoke] service ready: ${name}"
      return 0
    fi
    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "main_chain_smoke: timeout waiting for service ${name}" >&2
      echo "main_chain_smoke: launch log: ${LOG_FILE}" >&2
      return 1
    fi
    sleep 1
  done
}

assert_service_absent() {
  local name="$1"
  if ros2 service list | grep -Fxq "${name}"; then
    echo "main_chain_smoke: service must not be registered in ${SERVICE_EXPOSURE_PROFILE}: ${name}" >&2
    return 1
  fi
  echo "[main_chain_smoke] service correctly absent: ${name}"
}

LAST_SERVICE_RESPONSE=""
call_service_expect_success() {
  local step_label="$1"
  local service_name="$2"
  local service_type="$3"
  local payload="$4"

  echo "[main_chain_smoke] ${step_label}"
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
      echo "main_chain_smoke: retrying ${service_name} (${attempt}/${SERVICE_CALL_RETRIES})" >&2
      sleep 1
    fi
  done
  LAST_SERVICE_RESPONSE="${response}"
  echo "main_chain_smoke: ${service_name} did not return a success response after ${SERVICE_CALL_RETRIES} attempt(s)" >&2
  return 1
}

require_last_response_contains() {
  local token="$1"
  local label="$2"
  if ! grep -q "${token}" <<< "${LAST_SERVICE_RESPONSE}"; then
    echo "main_chain_smoke: ${label} missing token: ${token}" >&2
    return 1
  fi
}

wait_for_service "/xmate_er3/cobot/connect" 120
wait_for_service "/xmate_er3/cobot/set_power_state" 60
wait_for_service "/xmate_er3/cobot/set_operate_mode" 60
wait_for_service "/xmate_er3/cobot/set_motion_control_mode" 60
wait_for_service "/xmate_er3/cobot/move_reset" 60
wait_for_service "/xmate_er3/cobot/move_start" 60
wait_for_service "/xmate_er3/cobot/get_joint_pos" 60
wait_for_service "/xmate_er3/cobot/get_toolset" 60
wait_for_service "/xmate_er3/cobot/set_toolset" 60
wait_for_service "/xmate_er3/cobot/set_toolset_by_name" 60
wait_for_service "/xmate_er3/cobot/get_soft_limit" 60
wait_for_service "/xmate_er3/cobot/set_soft_limit" 60
wait_for_service "/xmate_er3/cobot/calc_fk" 60
wait_for_service "/xmate_er3/cobot/calc_ik" 60
wait_for_service "/xmate_er3/cobot/get_end_wrench" 60
wait_for_service "/xmate_er3/cobot/calc_joint_torque" 60
wait_for_service "/xmate_er3/cobot/get_runtime_diagnostics" 60
wait_for_service "/xmate_er3/cobot/get_runtime_state_snapshot" 60
wait_for_service "/xmate_er3/cobot/get_profile_capabilities" 60
wait_for_service "/xmate_er3/cobot/disconnect" 60

if [[ "${SERVICE_EXPOSURE_PROFILE}" == "public_xmate_er3_only" ]]; then
  assert_service_absent "/xmate_er3/cobot/enable_drag"
  assert_service_absent "/xmate_er3/cobot/start_record_path"
  assert_service_absent "/xmate_er3/cobot/set_rt_control_mode"
  assert_service_absent "/xmate3/io/get_di"
  assert_service_absent "/xmate3/cobot/read_register"
  assert_service_absent "/xmate3/cobot/load_rl_project"
fi

call_service_expect_success "connect" \
  /xmate_er3/cobot/connect \
  rokae_xmate3_ros2/srv/Connect \
  "{remote_ip: '127.0.0.1', local_ip: '127.0.0.1'}"

call_service_expect_success "profile capabilities" \
  /xmate_er3/cobot/get_profile_capabilities \
  rokae_xmate3_ros2/srv/GetProfileCapabilities \
  "{}"
require_last_response_contains "simulation_grade" "profile capabilities"
require_last_response_contains "xmate_er3" "profile capabilities"

call_service_expect_success "set_operate_mode(manual)" \
  /xmate_er3/cobot/set_operate_mode \
  rokae_xmate3_ros2/srv/SetOperateMode \
  "{mode: 0}"

call_service_expect_success "set_soft_limit(enable broad public limits)" \
  /xmate_er3/cobot/set_soft_limit \
  rokae_xmate3_ros2/srv/SetSoftLimit \
  "{enable: true, limits: [-3.0527, 3.0527, -2.0933, 2.0933, -2.0933, 2.0933, -3.0527, 3.0527, -2.0933, 2.0933, -6.1082, 6.1082]}"

call_service_expect_success "get_soft_limit(enabled)" \
  /xmate_er3/cobot/get_soft_limit \
  rokae_xmate3_ros2/srv/GetSoftLimit \
  "{}"
require_last_response_contains "enable=True" "get_soft_limit(enabled)"

call_service_expect_success "set_soft_limit(disable before motion)" \
  /xmate_er3/cobot/set_soft_limit \
  rokae_xmate3_ros2/srv/SetSoftLimit \
  "{enable: false, limits: [-3.0527, 3.0527, -2.0933, 2.0933, -2.0933, 2.0933, -3.0527, 3.0527, -2.0933, 2.0933, -6.1082, 6.1082]}"

call_service_expect_success "set_toolset" \
  /xmate_er3/cobot/set_toolset \
  rokae_xmate3_ros2/srv/SetToolset \
  "{tool_name: 'tool_smoke', wobj_name: 'fixture_smoke', tool_pose: [0.0, 0.0, 0.08, 0.0, 0.0, 0.0], wobj_pose: [0.20, -0.10, 0.30, 0.0, 0.0, 0.0]}"

call_service_expect_success "set_toolset_by_name" \
  /xmate_er3/cobot/set_toolset_by_name \
  rokae_xmate3_ros2/srv/SetToolsetByName \
  "{tool_name: 'tool_smoke', wobj_name: 'fixture_smoke'}"

call_service_expect_success "get_toolset" \
  /xmate_er3/cobot/get_toolset \
  rokae_xmate3_ros2/srv/GetToolset \
  "{}"
require_last_response_contains "tool_smoke" "get_toolset"
require_last_response_contains "fixture_smoke" "get_toolset"

call_service_expect_success "calc_joint_torque" \
  /xmate_er3/cobot/calc_joint_torque \
  rokae_xmate3_ros2/srv/CalcJointTorque \
  "{joint_pos: [0.0, -0.4, 0.35, 0.0, 0.45, 0.0], joint_vel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], joint_acc: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], external_force: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

call_service_expect_success "set_power_state(on)" \
  /xmate_er3/cobot/set_power_state \
  rokae_xmate3_ros2/srv/SetPowerState \
  "{'on': true}"

call_service_expect_success "set_operate_mode(automatic)" \
  /xmate_er3/cobot/set_operate_mode \
  rokae_xmate3_ros2/srv/SetOperateMode \
  "{mode: 1}"

call_service_expect_success "set_motion_control_mode(NRT)" \
  /xmate_er3/cobot/set_motion_control_mode \
  rokae_xmate3_ros2/srv/SetMotionControlMode \
  "{mode: 0}"

call_service_expect_success "move_reset" \
  /xmate_er3/cobot/move_reset \
  rokae_xmate3_ros2/srv/MoveReset \
  "{}"

echo "[main_chain_smoke] move_append(absj) + move_start"
MOVE_APPEND_LOG="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/main_chain_move_append.log"
timeout 120 ros2 action send_goal /xmate_er3/cobot/move_append rokae_xmate3_ros2/action/MoveAppend "{absj_cmds: [{target: {joints: ${MOVE_TARGET_JOINTS}, external: []}, speed: ${MOVE_TARGET_SPEED}, zone: ${MOVE_TARGET_ZONE}}], j_cmds: [], l_cmds: [], c_cmds: [], cf_cmds: [], sp_cmds: []}" >"${MOVE_APPEND_LOG}" 2>&1 &
MOVE_APPEND_PID=$!
MOVE_APPEND_ACCEPT_TIMEOUT=30
MOVE_APPEND_ACCEPT_START="$(date +%s)"
while true; do
  if grep -q "Goal accepted with ID" "${MOVE_APPEND_LOG}" 2>/dev/null; then
    break
  fi
  if grep -q "Goal rejected" "${MOVE_APPEND_LOG}" 2>/dev/null; then
    cat "${MOVE_APPEND_LOG}"
    echo "main_chain_smoke: move_append goal rejected" >&2
    exit 1
  fi
  if ! kill -0 "${MOVE_APPEND_PID}" 2>/dev/null; then
    cat "${MOVE_APPEND_LOG}"
    echo "main_chain_smoke: move_append action exited before goal acceptance" >&2
    exit 1
  fi
  if (( "$(date +%s)" - MOVE_APPEND_ACCEPT_START >= MOVE_APPEND_ACCEPT_TIMEOUT )); then
    cat "${MOVE_APPEND_LOG}"
    echo "main_chain_smoke: timeout waiting for move_append goal acceptance" >&2
    exit 1
  fi
  sleep 0.2
done
call_service_expect_success "move_start" \
  /xmate_er3/cobot/move_start \
  rokae_xmate3_ros2/srv/MoveStart \
  "{}"
if ! wait "${MOVE_APPEND_PID}"; then
  cat "${MOVE_APPEND_LOG}"
  echo "main_chain_smoke: move_append action command failed" >&2
  exit 1
fi
cat "${MOVE_APPEND_LOG}"
if grep -q "success: true" "${MOVE_APPEND_LOG}" && grep -q "Goal finished with status: SUCCEEDED" "${MOVE_APPEND_LOG}"; then
  :
elif grep -q "message: completed_with_relaxed_settle" "${MOVE_APPEND_LOG}" && \
    grep -q "Goal finished with status: ABORTED" "${MOVE_APPEND_LOG}"; then
  echo "[main_chain_smoke] move_append reached relaxed settle handoff state"
else
  echo "main_chain_smoke: move_append action did not succeed" >&2
  exit 1
fi

POST_MOVE_SETTLE_SEC="${ROKAE_MAIN_CHAIN_POST_MOVE_SETTLE_SEC:-5}"
echo "[main_chain_smoke] waiting ${POST_MOVE_SETTLE_SEC}s for runtime/JTC state to advance"
sleep "${POST_MOVE_SETTLE_SEC}"

call_service_expect_success "get_joint_pos" \
  /xmate_er3/cobot/get_joint_pos \
  rokae_xmate3_ros2/srv/GetJointPos \
  "{}"

KINEMATICS_PROBE="${SCRIPT_DIR}/run_public_kinematics_probe.py"
if [[ ! -f "${KINEMATICS_PROBE}" ]]; then
  echo "main_chain_smoke: missing public kinematics probe next to installed tools" >&2
  exit 1
fi
call_service_expect_success "set_toolset_by_name(default for kinematics)" \
  /xmate_er3/cobot/set_toolset_by_name \
  rokae_xmate3_ros2/srv/SetToolsetByName \
  "{tool_name: 'tool0', wobj_name: 'wobj0'}"
echo "[main_chain_smoke] public kinematics fk/ik probe"
"${ROKAE_PYTHON_EXECUTABLE:-python3}" "${KINEMATICS_PROBE}"
call_service_expect_success "restore smoke toolset" \
  /xmate_er3/cobot/set_toolset_by_name \
  rokae_xmate3_ros2/srv/SetToolsetByName \
  "{tool_name: 'tool_smoke', wobj_name: 'fixture_smoke'}"

call_service_expect_success "get_end_wrench" \
  /xmate_er3/cobot/get_end_wrench \
  rokae_xmate3_ros2/srv/GetEndWrench \
  "{ref_type: 0}"
require_last_response_contains "SimApprox" "get_end_wrench"
require_last_response_contains "joint_torque_measured" "get_end_wrench"

call_service_expect_success "runtime state snapshot" \
  /xmate_er3/cobot/get_runtime_state_snapshot \
  rokae_xmate3_ros2/srv/GetRuntimeStateSnapshot \
  "{}"
require_last_response_contains "runtime-owned read snapshot" "runtime state snapshot"
require_last_response_contains "tool_smoke" "runtime state snapshot"

call_service_expect_success "stop" \
  /xmate_er3/cobot/stop \
  rokae_xmate3_ros2/srv/Stop \
  "{}"

call_service_expect_success "move_reset(after stop)" \
  /xmate_er3/cobot/move_reset \
  rokae_xmate3_ros2/srv/MoveReset \
  "{}"

echo "[main_chain_smoke] get_runtime_diagnostics"
RUNTIME_DIAG_LOG="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/main_chain_runtime_diagnostics.log"
if ! ros2 service call /xmate_er3/cobot/get_runtime_diagnostics rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}" >"${RUNTIME_DIAG_LOG}" 2>&1; then
  cat "${RUNTIME_DIAG_LOG}"
  echo "main_chain_smoke: runtime diagnostics query failed" >&2
  exit 1
fi
cat "${RUNTIME_DIAG_LOG}"
if ! grep -q "rt_late_cycle_count" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_late_cycle_count" >&2
  exit 1
fi
if ! grep -q "rt_last_trigger_reason" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_last_trigger_reason" >&2
  exit 1
fi
if ! grep -q "rt_profile" "${RUNTIME_DIAG_LOG}" && ! grep -q "active_profile" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing profile marker" >&2
  exit 1
fi
if ! grep -q "rt_transport_source" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_transport_source" >&2
  exit 1
fi
if ! grep -q "rt_scheduler_state" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_scheduler_state" >&2
  exit 1
fi
if ! grep -q "rt_deadline_miss" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_deadline_miss" >&2
  exit 1
fi
if ! grep -q "rt_rx_latency_us" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_rx_latency_us" >&2
  exit 1
fi
if ! grep -q "rt_queue_depth" "${RUNTIME_DIAG_LOG}"; then
  echo "main_chain_smoke: runtime diagnostics missing rt_queue_depth" >&2
  exit 1
fi

RUNTIME_STATUS_LOG="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/main_chain_runtime_status_topic.log"
if ! timeout 20 ros2 topic echo /xmate_er3/cobot/runtime_status --once >"${RUNTIME_STATUS_LOG}" 2>&1; then
  cat "${RUNTIME_STATUS_LOG}"
  echo "main_chain_smoke: failed to capture runtime status topic" >&2
  exit 1
fi

resolve_limits_file() {
  local candidate=""
  if [[ -n "${ROKAE_RT_GATE_LIMITS_FILE:-}" ]]; then
    candidate="${ROKAE_RT_GATE_LIMITS_FILE}"
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
    echo "main_chain_smoke: configured ROKAE_RT_GATE_LIMITS_FILE not found: ${candidate}" >&2
    exit 1
  fi

  local source_candidate="${PACKAGE_ROOT}/config/runtime_diag_gate.default.json"
  if [[ ! -f "${source_candidate}" ]]; then
    source_candidate="${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/config/runtime_diag_gate.default.json"
  fi
  if [[ -f "${source_candidate}" ]]; then
    printf '%s\n' "${source_candidate}"
    return 0
  fi

  local install_candidate="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/config/runtime_diag_gate.default.json"
  if [[ -f "${install_candidate}" ]]; then
    printf '%s\n' "${install_candidate}"
    return 0
  fi

  return 1
}

RUNTIME_DIAG_LIMITS_FILE="$(resolve_limits_file || true)"
DIAG_GATE_ARGS=("${RUNTIME_STATUS_LOG}")
if [[ -n "${RUNTIME_DIAG_LIMITS_FILE}" ]]; then
  echo "[main_chain_smoke] runtime diagnostics limits file: ${RUNTIME_DIAG_LIMITS_FILE}"
  DIAG_GATE_ARGS+=(--limits-file "${RUNTIME_DIAG_LIMITS_FILE}")
else
  echo "[main_chain_smoke] runtime diagnostics limits file not found; falling back to explicit defaults" >&2
fi
if [[ "${ROKAE_RT_GATE_REQUIRE_NO_DEADLINE_MISS:-0}" == "1" ]]; then
  DIAG_GATE_ARGS+=(--require-no-deadline-miss)
fi
DIAG_GATE_ARGS+=(--max-gap-ms "${ROKAE_RT_GATE_MAX_GAP_MS:-50}")
DIAG_GATE_ARGS+=(--max-rx-latency-us "${ROKAE_RT_GATE_MAX_RX_LATENCY_US:-50000}")
DIAG_GATE_ARGS+=(--max-queue-depth "${ROKAE_RT_GATE_MAX_QUEUE_DEPTH:-8}")
DIAG_GATE_ARGS+=(--print-effective-limits)

RUNTIME_DIAG_GATE_TOOL="${SCRIPT_DIR}/check_runtime_diag_gate.py"
if [[ ! -f "${RUNTIME_DIAG_GATE_TOOL}" ]]; then
  echo "main_chain_smoke: missing runtime diagnostics gate helper next to installed tools" >&2
  exit 1
fi
if ! "${ROKAE_PYTHON_EXECUTABLE:-python3}"   "${RUNTIME_DIAG_GATE_TOOL}"   "${DIAG_GATE_ARGS[@]}"; then
  echo "main_chain_smoke: runtime diagnostics threshold gate failed" >&2
  exit 1
fi

call_service_expect_success "disconnect" \
  /xmate_er3/cobot/disconnect \
  rokae_xmate3_ros2/srv/Disconnect \
  "{}"

echo "[main_chain_smoke] success"
echo "[main_chain_smoke] launch log: ${LOG_FILE}"
