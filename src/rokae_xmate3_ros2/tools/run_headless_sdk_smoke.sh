#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run_headless_sdk_smoke.sh [workspace-root]" >&2
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROKAE_MAIN_CHAIN_SMOKE_LABEL="${ROKAE_MAIN_CHAIN_SMOKE_LABEL:-headless_sdk_smoke}"
export ROKAE_MAIN_CHAIN_LAUNCH_PROFILE="public_xmate_er3_headless_sdk_smoke"
export ROKAE_MAIN_CHAIN_BACKEND_MODE="headless_sim"
export ROKAE_MAIN_CHAIN_SERVICE_EXPOSURE_PROFILE="public_xmate_er3_only"
export ROKAE_MAIN_CHAIN_ENABLE_ROS2_CONTROL="false"
export ROKAE_MAIN_CHAIN_ENABLE_XCORE_PLUGIN="false"
export ROKAE_MAIN_CHAIN_COMPATIBILITY_ALIAS_POLICY="canonical_only"

exec "${SCRIPT_DIR}/run_main_chain_smoke.sh" "$@"
