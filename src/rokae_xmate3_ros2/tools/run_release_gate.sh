#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 <workspace-root>" >&2
  exit 64
fi

WS_ROOT="$1"
PKG_NAME="rokae_xmate3_ros2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_TOOLS="${WS_ROOT}/src/${PKG_NAME}/tools"

# shellcheck disable=SC1090
. "${SOURCE_TOOLS}/acceptance_cli_common.sh"
rokae_acceptance_source_ros_env
"${SOURCE_TOOLS}/run_full_source_tree_build_gate.sh" "${WS_ROOT}" --ctest-labels "quick_gate;semantic_gate;release_gate"
"${SCRIPT_DIR}/run_xmate_er3_alignment_behavior_gate.sh" "${WS_ROOT}"
"${SCRIPT_DIR}/run_main_chain_smoke.sh" "${WS_ROOT}"
