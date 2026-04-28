#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
usage: run_real_dryrun_acceptance.sh <workspace-root> [--namespace /xmate_er3]

Run the L3 dry-run acceptance bundle against an already running real-runtime namespace.
No motion is commanded. Required environment variables:
  ROKAE_ACCEPTANCE_REMOTE_IP
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
NAMESPACE="${ROKAE_ACCEPTANCE_NAMESPACE:-/xmate_er3}"
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
if [ -z "${REMOTE_IP}" ]; then
  echo "L3 dry-run requires ROKAE_ACCEPTANCE_REMOTE_IP" >&2
  exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
. "${SCRIPT_DIR}/acceptance_cli_common.sh"
rokae_acceptance_source_env "${WORKSPACE_ROOT}"

CONNECT_SERVICE="${NAMESPACE}/cobot/connect"
DISCONNECT_SERVICE="${NAMESPACE}/cobot/disconnect"
MODE_SERVICE="${NAMESPACE}/cobot/set_motion_control_mode"
GET_JOINT_SERVICE="${NAMESPACE}/cobot/get_joint_pos"
DIAG_SERVICE="${NAMESPACE}/cobot/get_runtime_diagnostics"

rokae_acceptance_wait_for_service "${CONNECT_SERVICE}" 60
rokae_acceptance_wait_for_service "${DISCONNECT_SERVICE}" 60
rokae_acceptance_wait_for_service "${MODE_SERVICE}" 60
rokae_acceptance_wait_for_service "${GET_JOINT_SERVICE}" 60
rokae_acceptance_wait_for_service "${DIAG_SERVICE}" 60

rokae_acceptance_call_service "${CONNECT_SERVICE}" rokae_xmate3_ros2/srv/Connect "{remote_ip: '${REMOTE_IP}', local_ip: '${LOCAL_IP}'}" >/dev/null
rokae_acceptance_call_service "${MODE_SERVICE}" rokae_xmate3_ros2/srv/SetMotionControlMode "{mode: 0}" >/dev/null
rokae_acceptance_call_service "${GET_JOINT_SERVICE}" rokae_xmate3_ros2/srv/GetJointPos "{}" >/dev/null
rokae_acceptance_call_service "${DIAG_SERVICE}" rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}" >/dev/null
rokae_acceptance_call_service "${DISCONNECT_SERVICE}" rokae_xmate3_ros2/srv/Disconnect "{}" >/dev/null
