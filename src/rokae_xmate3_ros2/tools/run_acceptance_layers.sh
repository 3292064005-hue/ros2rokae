#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
usage: run_acceptance_layers.sh <workspace-root> [--up-to L0|L1|L2|L3|L4|L5] [--namespace /xmate_er3]

Execute the confirmed L0-L5 layered acceptance ladder. L0-L2 run by default; L3-L5 require
real runtime endpoints and environment variables consumed by the corresponding layer scripts.
USAGE
}

if [ $# -lt 1 ]; then
  usage
  exit 64
fi

WORKSPACE_ROOT="$1"
shift
UP_TO="L2"
NAMESPACE="${ROKAE_ACCEPTANCE_NAMESPACE:-/xmate_er3}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --up-to)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      UP_TO="$2"
      shift 2
      ;;
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1090
. "${SCRIPT_DIR}/acceptance_cli_common.sh"

run_layer() {
  local layer="$1"
  case "${layer}" in
    L0)
      (cd "${WORKSPACE_ROOT}/build/rokae_xmate3_ros2" && ctest -L quick_gate --output-on-failure)
      ;;
    L1)
      (cd "${WORKSPACE_ROOT}/build/rokae_xmate3_ros2" && ctest -L semantic_gate --output-on-failure)
      local xmate_er3_alignment_gate
      xmate_er3_alignment_gate="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_xmate_er3_alignment_behavior_gate.sh)"
      "${xmate_er3_alignment_gate}" "${WORKSPACE_ROOT}"
      ;;
    L2)
      local launch_smoke
      local main_chain_smoke
      local headless_sdk_smoke
      launch_smoke="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_launch_smoke.sh)"
      main_chain_smoke="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_main_chain_smoke.sh)"
      headless_sdk_smoke="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_headless_sdk_smoke.sh)"
      "${launch_smoke}" "${WORKSPACE_ROOT}"
      "${main_chain_smoke}" "${WORKSPACE_ROOT}"
      "${headless_sdk_smoke}" "${WORKSPACE_ROOT}"
      ;;
    L3)
      local dryrun
      dryrun="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_real_dryrun_acceptance.sh)"
      "${dryrun}" "${WORKSPACE_ROOT}" --namespace "${NAMESPACE}"
      ;;
    L4)
      local loaded_sensor
      loaded_sensor="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_loaded_sensor_acceptance.sh)"
      "${loaded_sensor}" "${WORKSPACE_ROOT}" --namespace "${NAMESPACE}"
      ;;
    L5)
      local full_task
      full_task="$(rokae_acceptance_tool_path "${SCRIPT_DIR}" "${WORKSPACE_ROOT}" run_full_task_acceptance.sh)"
      "${full_task}" "${WORKSPACE_ROOT}" --namespace "${NAMESPACE}"
      ;;
    *)
      echo "unknown layer ${layer}" >&2
      return 64
      ;;
  esac
}

LAYERS=(L0 L1 L2 L3 L4 L5)
for layer in "${LAYERS[@]}"; do
  run_layer "${layer}"
  if [ "${layer}" = "${UP_TO}" ]; then
    exit 0
  fi
done

echo "acceptance: invalid --up-to value ${UP_TO}" >&2
exit 64
