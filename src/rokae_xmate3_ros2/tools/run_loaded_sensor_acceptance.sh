#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
usage: run_loaded_sensor_acceptance.sh <workspace-root> [--namespace /xmate_er3]

Run the L4 loaded-sensor acceptance bundle. Required environment variables:
  ROKAE_ACCEPTANCE_REMOTE_IP
  ROKAE_ACCEPTANCE_SENSOR_TOPIC
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
SENSOR_TOPIC="${ROKAE_ACCEPTANCE_SENSOR_TOPIC:-}"
if [ -z "${REMOTE_IP}" ] || [ -z "${SENSOR_TOPIC}" ]; then
  echo "L4 loaded-sensor acceptance requires ROKAE_ACCEPTANCE_REMOTE_IP and ROKAE_ACCEPTANCE_SENSOR_TOPIC" >&2
  exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
. "${SCRIPT_DIR}/acceptance_cli_common.sh"
rokae_acceptance_source_env "${WORKSPACE_ROOT}"

CONNECT_SERVICE="${NAMESPACE}/cobot/connect"
DISCONNECT_SERVICE="${NAMESPACE}/cobot/disconnect"
START_RECORD_SERVICE="${NAMESPACE}/cobot/start_record_path"
STOP_RECORD_SERVICE="${NAMESPACE}/cobot/stop_record_path"
SAVE_RECORD_SERVICE="${NAMESPACE}/cobot/save_record_path"
QUERY_PATHS_SERVICE="${NAMESPACE}/cobot/query_path_lists"

rokae_acceptance_wait_for_service "${CONNECT_SERVICE}" 60
rokae_acceptance_wait_for_service "${DISCONNECT_SERVICE}" 60
rokae_acceptance_wait_for_service "${START_RECORD_SERVICE}" 60
rokae_acceptance_wait_for_service "${STOP_RECORD_SERVICE}" 60
rokae_acceptance_wait_for_service "${SAVE_RECORD_SERVICE}" 60
rokae_acceptance_wait_for_service "${QUERY_PATHS_SERVICE}" 60
rokae_acceptance_wait_for_topic "${SENSOR_TOPIC}" 60
ros2 topic echo --once "${SENSOR_TOPIC}" >/dev/null

rokae_acceptance_call_service "${CONNECT_SERVICE}" rokae_xmate3_ros2/srv/Connect "{remote_ip: '${REMOTE_IP}', local_ip: '${LOCAL_IP}'}" >/dev/null
rokae_acceptance_call_service "${START_RECORD_SERVICE}" rokae_xmate3_ros2/srv/StartRecordPath "{duration: 5.0}" >/dev/null
sleep 1
rokae_acceptance_call_service "${STOP_RECORD_SERVICE}" rokae_xmate3_ros2/srv/StopRecordPath "{}" >/dev/null
rokae_acceptance_call_service "${SAVE_RECORD_SERVICE}" rokae_xmate3_ros2/srv/SaveRecordPath "{name: 'l4_sensor_record', save_as: ''}" >/dev/null
rokae_acceptance_call_service "${QUERY_PATHS_SERVICE}" rokae_xmate3_ros2/srv/QueryPathLists "{}" >/dev/null
rokae_acceptance_call_service "${DISCONNECT_SERVICE}" rokae_xmate3_ros2/srv/Disconnect "{}" >/dev/null
