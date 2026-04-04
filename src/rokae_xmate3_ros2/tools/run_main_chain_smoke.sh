#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run_main_chain_smoke.sh [workspace-root]" >&2
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

WORKSPACE_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
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

LOG_FILE="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/main_chain_smoke.log"
mkdir -p "$(dirname "${LOG_FILE}")"

MODEL_PATH="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/urdf/xMate3.xacro"
if [[ ! -f "${MODEL_PATH}" ]]; then
  MODEL_PATH="${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/urdf/xMate3.xacro"
fi
if [[ ! -f "${MODEL_PATH}" ]]; then
  echo "main_chain_smoke: missing xacro model for non-canonical launch" >&2
  exit 1
fi

echo "[main_chain_smoke] launching simulation..."
ros2 launch rokae_xmate3_ros2 simulation.launch.py \
  gui:=false \
  rviz:=false \
  allow_noncanonical_model:=true \
  model:="${MODEL_PATH}" \
  enable_ros2_control:=false >"${LOG_FILE}" 2>&1 &
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

call_service_expect_success() {
  local step_label="$1"
  local service_name="$2"
  local service_type="$3"
  local payload="$4"

  echo "[main_chain_smoke] ${step_label}"
  local response
  if ! response="$(ros2 service call "${service_name}" "${service_type}" "${payload}" 2>&1)"; then
    echo "${response}"
    echo "main_chain_smoke: service call failed at ${service_name}" >&2
    return 1
  fi
  echo "${response}"
  if ! grep -q "success=True" <<< "${response}"; then
    echo "main_chain_smoke: ${service_name} returned non-success response" >&2
    return 1
  fi
}

wait_for_service "/xmate3/cobot/connect" 120
wait_for_service "/xmate3/cobot/set_power_state" 60
wait_for_service "/xmate3/cobot/set_operate_mode" 60
wait_for_service "/xmate3/cobot/set_motion_control_mode" 60
wait_for_service "/xmate3/cobot/move_reset" 60
wait_for_service "/xmate3/cobot/move_start" 60
wait_for_service "/xmate3/cobot/get_joint_pos" 60
wait_for_service "/xmate3/internal/get_runtime_diagnostics" 60

call_service_expect_success "connect" \
  /xmate3/cobot/connect \
  rokae_xmate3_ros2/srv/Connect \
  "{remote_ip: '127.0.0.1', local_ip: '127.0.0.1'}"

call_service_expect_success "set_power_state(on)" \
  /xmate3/cobot/set_power_state \
  rokae_xmate3_ros2/srv/SetPowerState \
  "{'on': true}"

call_service_expect_success "set_operate_mode(automatic)" \
  /xmate3/cobot/set_operate_mode \
  rokae_xmate3_ros2/srv/SetOperateMode \
  "{mode: 1}"

call_service_expect_success "set_motion_control_mode(NRT)" \
  /xmate3/cobot/set_motion_control_mode \
  rokae_xmate3_ros2/srv/SetMotionControlMode \
  "{mode: 0}"

call_service_expect_success "move_reset" \
  /xmate3/cobot/move_reset \
  rokae_xmate3_ros2/srv/MoveReset \
  "{}"

echo "[main_chain_smoke] move_append(absj) + move_start"
MOVE_APPEND_LOG="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/main_chain_move_append.log"
timeout 120 ros2 action send_goal /xmate3/cobot/move_append rokae_xmate3_ros2/action/MoveAppend "{absj_cmds: [{target: {joints: [0.2, -0.2, 0.3, -0.1, 0.2, 0.4], external: []}, speed: 20, zone: 5}], j_cmds: [], l_cmds: [], c_cmds: [], cf_cmds: [], sp_cmds: []}" >"${MOVE_APPEND_LOG}" 2>&1 &
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
  /xmate3/cobot/move_start \
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

call_service_expect_success "get_joint_pos" \
  /xmate3/cobot/get_joint_pos \
  rokae_xmate3_ros2/srv/GetJointPos \
  "{}"

echo "[main_chain_smoke] get_runtime_diagnostics"
RUNTIME_DIAG_LOG="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/main_chain_runtime_diagnostics.log"
if ! ros2 service call /xmate3/internal/get_runtime_diagnostics rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}" >"${RUNTIME_DIAG_LOG}" 2>&1; then
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
if ! timeout 20 ros2 topic echo /xmate3/internal/runtime_status --once >"${RUNTIME_STATUS_LOG}" 2>&1; then
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

  local source_candidate="${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/config/runtime_diag_gate.default.json"
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

if ! "${ROKAE_PYTHON_EXECUTABLE:-python3}"   "${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/tools/check_runtime_diag_gate.py"   "${DIAG_GATE_ARGS[@]}"; then
  echo "main_chain_smoke: runtime diagnostics threshold gate failed" >&2
  exit 1
fi

echo "[main_chain_smoke] success"
echo "[main_chain_smoke] launch log: ${LOG_FILE}"
