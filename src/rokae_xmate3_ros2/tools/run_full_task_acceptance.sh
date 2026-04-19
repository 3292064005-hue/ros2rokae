#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
usage: run_full_task_acceptance.sh <workspace-root> [--namespace /xmate3]

Run the L5 full-task acceptance bundle against an already running real-runtime namespace.
Required environment variables:
  ROKAE_ACCEPTANCE_REMOTE_IP
  ROKAE_ACCEPTANCE_ALLOW_MOTION=1
  ROKAE_ACCEPTANCE_MOVE_APPEND_GOAL   (YAML payload for rokae_xmate3_ros2/action/MoveAppend)
Optional:
  ROKAE_ACCEPTANCE_LOCAL_IP
USAGE
}

if [ $# -lt 1 ]; then
  usage
  exit 64
fi

WORKSPACE_ROOT="$1"
shift
NAMESPACE="${ROKAE_ACCEPTANCE_NAMESPACE:-/xmate3}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --namespace)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      NAMESPACE="$2"
      shift 2
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

REMOTE_IP="${ROKAE_ACCEPTANCE_REMOTE_IP:-}"
LOCAL_IP="${ROKAE_ACCEPTANCE_LOCAL_IP:-127.0.0.1}"
ALLOW_MOTION="${ROKAE_ACCEPTANCE_ALLOW_MOTION:-0}"
MOVE_GOAL="${ROKAE_ACCEPTANCE_MOVE_APPEND_GOAL:-}"
if [ -z "${REMOTE_IP}" ] || [ "${ALLOW_MOTION}" != "1" ] || [ -z "${MOVE_GOAL}" ]; then
  echo "L5 full-task acceptance requires ROKAE_ACCEPTANCE_REMOTE_IP, ROKAE_ACCEPTANCE_ALLOW_MOTION=1 and ROKAE_ACCEPTANCE_MOVE_APPEND_GOAL" >&2
  exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
. "${SCRIPT_DIR}/acceptance_cli_common.sh"
rokae_acceptance_source_env "${WORKSPACE_ROOT}"

CONNECT_SERVICE="${NAMESPACE}/cobot/connect"
DISCONNECT_SERVICE="${NAMESPACE}/cobot/disconnect"
POWER_SERVICE="${NAMESPACE}/cobot/set_power_state"
OPERATE_SERVICE="${NAMESPACE}/cobot/set_operate_mode"
MODE_SERVICE="${NAMESPACE}/cobot/set_motion_control_mode"
RESET_SERVICE="${NAMESPACE}/cobot/move_reset"
START_SERVICE="${NAMESPACE}/cobot/move_start"
DIAG_SERVICE="${NAMESPACE}/internal/get_runtime_diagnostics"
ACTION_NAME="${NAMESPACE}/cobot/move_append"

for service_name in "${CONNECT_SERVICE}" "${DISCONNECT_SERVICE}" "${POWER_SERVICE}" "${OPERATE_SERVICE}" "${MODE_SERVICE}" "${RESET_SERVICE}" "${START_SERVICE}" "${DIAG_SERVICE}"; do
  rokae_acceptance_wait_for_service "${service_name}" 60
done

rokae_acceptance_call_service "${CONNECT_SERVICE}" rokae_xmate3_ros2/srv/Connect "{remote_ip: '${REMOTE_IP}', local_ip: '${LOCAL_IP}'}" >/dev/null
rokae_acceptance_call_service "${OPERATE_SERVICE}" rokae_xmate3_ros2/srv/SetOperateMode "{mode: 1}" >/dev/null
rokae_acceptance_call_service "${POWER_SERVICE}" rokae_xmate3_ros2/srv/SetPowerState "{on: true}" >/dev/null
rokae_acceptance_call_service "${MODE_SERVICE}" rokae_xmate3_ros2/srv/SetMotionControlMode "{mode: 0}" >/dev/null
rokae_acceptance_call_service "${RESET_SERVICE}" rokae_xmate3_ros2/srv/MoveReset "{}" >/dev/null

ACTION_LOG="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/l5_full_task_move_append.log"
mkdir -p "$(dirname "${ACTION_LOG}")"
timeout 180 ros2 action send_goal "${ACTION_NAME}" rokae_xmate3_ros2/action/MoveAppend "${MOVE_GOAL}" >"${ACTION_LOG}" 2>&1 &
ACTION_PID=$!
ACCEPT_START="$(date +%s)"
while true; do
  if grep -q "Goal accepted with ID" "${ACTION_LOG}" 2>/dev/null; then
    break
  fi
  if grep -q "Goal rejected" "${ACTION_LOG}" 2>/dev/null; then
    cat "${ACTION_LOG}" >&2
    exit 1
  fi
  if ! kill -0 "${ACTION_PID}" 2>/dev/null; then
    cat "${ACTION_LOG}" >&2
    exit 1
  fi
  if (( "$(date +%s)" - ACCEPT_START >= 45 )); then
    cat "${ACTION_LOG}" >&2
    echo "L5 acceptance timed out waiting for goal acceptance" >&2
    exit 1
  fi
  sleep 0.2
done

rokae_acceptance_call_service "${START_SERVICE}" rokae_xmate3_ros2/srv/MoveStart "{}" >/dev/null
wait "${ACTION_PID}"
if ! grep -q "Goal finished with status: SUCCEEDED" "${ACTION_LOG}" && ! grep -q "completed_with_relaxed_settle" "${ACTION_LOG}"; then
  cat "${ACTION_LOG}" >&2
  echo "L5 acceptance did not complete successfully" >&2
  exit 1
fi
rokae_acceptance_call_service "${DIAG_SERVICE}" rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}" >/dev/null
rokae_acceptance_call_service "${DISCONNECT_SERVICE}" rokae_xmate3_ros2/srv/Disconnect "{}" >/dev/null
