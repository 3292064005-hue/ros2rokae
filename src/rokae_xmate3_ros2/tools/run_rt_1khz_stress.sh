#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: run_rt_1khz_stress.sh [workspace-root] [duration-sec] [warmup-sec] [mode]" >&2
  echo "  mode: daemon (default) | simulation" >&2
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

check_rt_host_readiness() {
  local runtime_exe="$1"
  local skip_check="${ROKAE_RT_SKIP_HOST_CHECK:-0}"
  if [[ "${skip_check}" == "1" ]]; then
    echo "[rt_1khz_stress] host readiness check skipped by ROKAE_RT_SKIP_HOST_CHECK=1"
    return 0
  fi

  local rtprio_limit memlock_limit caps
  rtprio_limit="$(ulimit -r || echo 0)"
  memlock_limit="$(ulimit -l || echo 0)"
  caps="$(getcap "${runtime_exe}" 2>/dev/null || true)"

  local has_sys_nice=0
  local has_ipc_lock=0
  if [[ "${caps}" == *"cap_sys_nice"* ]]; then
    has_sys_nice=1
  fi
  if [[ "${caps}" == *"cap_ipc_lock"* ]]; then
    has_ipc_lock=1
  fi

  if [[ "${EUID}" -eq 0 ]]; then
    echo "[rt_1khz_stress] host readiness: running as root (strict RT scheduling allowed)"
    return 0
  fi

  if [[ "${has_sys_nice}" -eq 1 && "${has_ipc_lock}" -eq 1 ]]; then
    echo "[rt_1khz_stress] host readiness: runtime executable has cap_sys_nice + cap_ipc_lock"
    return 0
  fi

  echo "rt_1khz_stress: host is not ready for hard_1khz strict scheduler contract." >&2
  echo "rt_1khz_stress: current user=$(id -un) euid=${EUID} rtprio_limit=${rtprio_limit} memlock_limit=${memlock_limit}" >&2
  echo "rt_1khz_stress: executable capabilities: ${caps:-<none>}" >&2
  echo "rt_1khz_stress: fix with one of the following options:" >&2
  echo "  1) run as root for stress gating;" >&2
  echo "  2) grant caps to runtime binary:" >&2
  echo "     sudo setcap cap_sys_nice,cap_ipc_lock+ep \"${runtime_exe}\"" >&2
  echo "     getcap \"${runtime_exe}\"" >&2
  echo "  3) or run simulation fallback mode:" >&2
  echo "     bash src/rokae_xmate3_ros2/tools/run_rt_1khz_stress.sh \"${WORKSPACE_ROOT}\" ${DURATION_SEC} ${WARMUP_SEC} simulation" >&2
  return 1
}

WORKSPACE_ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
DURATION_SEC="${2:-600}"
WARMUP_SEC="${3:-5}"
RT_MODE="${4:-${ROKAE_RT_STRESS_MODE:-daemon}}"

if [[ ! -d "${WORKSPACE_ROOT}" ]]; then
  echo "rt_1khz_stress: workspace root not found: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "rt_1khz_stress: missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "rt_1khz_stress: missing workspace setup: ${WS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "rt_1khz_stress: ros2 CLI not available after sourcing setup files" >&2
  exit 1
fi

BUILD_LIB_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
if [[ -d "${BUILD_LIB_DIR}" ]]; then
  export ROKAE_XMATE3_ROS2_LIB_DIR="${BUILD_LIB_DIR}"
  export LD_LIBRARY_PATH="${BUILD_LIB_DIR}:${LD_LIBRARY_PATH:-}"
fi

LOG_DIR="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2"
mkdir -p "${LOG_DIR}"
LAUNCH_LOG="${LOG_DIR}/rt_1khz_stress_launch.log"
EXAMPLE_LOG="${LOG_DIR}/rt_1khz_stress_example.log"
DIAG_LOG="${LOG_DIR}/rt_1khz_stress_runtime_status.log"

MODEL_PATH="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/urdf/xMate3.xacro"
LAUNCH_PID=0
if [[ "${RT_MODE}" == "daemon" ]]; then
  export ROKAE_RT_TRANSPORT_MODE="${ROKAE_RT_TRANSPORT_MODE:-shm_only}"
  export ROKAE_REMOTE_IP="${ROKAE_REMOTE_IP:-127.0.0.1}"
  export ROKAE_LOCAL_IP="${ROKAE_LOCAL_IP:-127.0.0.1}"
  RUNTIME_EXE="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/bin/rokae_sim_runtime"
  if [[ ! -x "${RUNTIME_EXE}" ]]; then
    RUNTIME_EXE="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/rokae_sim_runtime"
  fi
  if [[ ! -x "${RUNTIME_EXE}" ]]; then
    echo "rt_1khz_stress: missing runtime daemon executable rokae_sim_runtime" >&2
    exit 1
  fi
  check_rt_host_readiness "${RUNTIME_EXE}"
  echo "[rt_1khz_stress] launching daemonized runtime (hard_1khz)..."
  "${RUNTIME_EXE}" \
    --ros-args \
    -p runtime_profile:=hard_1khz \
    -p rt_scheduler.enable:=true \
    -p rt_scheduler.policy:=fifo \
    -p rt_scheduler.priority:=90 \
    -p rt_memory.lock_all:=true >"${LAUNCH_LOG}" 2>&1 &
elif [[ "${RT_MODE}" == "simulation" ]]; then
  if [[ ! -f "${MODEL_PATH}" ]]; then
    MODEL_PATH="${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/urdf/xMate3.xacro"
  fi
  if [[ ! -f "${MODEL_PATH}" ]]; then
    echo "rt_1khz_stress: missing xacro model for non-canonical launch" >&2
    exit 1
  fi
  export ROKAE_RT_TRANSPORT_MODE="${ROKAE_RT_TRANSPORT_MODE:-shm_only}"
  WORLD_PATH="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/share/rokae_xmate3_ros2/worlds/rt_1khz.world"
  if [[ ! -f "${WORLD_PATH}" ]]; then
    WORLD_PATH="${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/worlds/rt_1khz.world"
  fi
  if [[ ! -f "${WORLD_PATH}" ]]; then
    WORLD_PATH="${WORKSPACE_ROOT}/src/rokae_xmate3_ros2/worlds/empty.world"
  fi
  echo "[rt_1khz_stress] launching gazebo simulation (effort backend)..."
  ros2 launch rokae_xmate3_ros2 simulation.launch.py \
    world:="${WORLD_PATH}" \
    gui:=false \
    rviz:=false \
    verbose:=false \
    allow_noncanonical_model:=true \
    model:="${MODEL_PATH}" \
    enable_ros2_control:=false \
    backend_mode:=effort >"${LAUNCH_LOG}" 2>&1 &
else
  echo "rt_1khz_stress: unsupported mode=${RT_MODE} (expect daemon|simulation)" >&2
  exit 1
fi
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
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      echo "rt_1khz_stress: runtime process exited before ${name} became available" >&2
      if grep -q "degraded_best_effort(scheduler_failed).*hard_failure=true" "${LAUNCH_LOG}" 2>/dev/null; then
        echo "rt_1khz_stress: hard_1khz requires real-time privileges (CAP_SYS_NICE/CAP_IPC_LOCK or root)." >&2
      fi
      echo "rt_1khz_stress: launch log: ${LAUNCH_LOG}" >&2
      return 1
    fi
    if ros2 service list | grep -Fxq "${name}"; then
      echo "[rt_1khz_stress] service ready: ${name}"
      return 0
    fi
    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "rt_1khz_stress: timeout waiting for service ${name}" >&2
      echo "rt_1khz_stress: launch log: ${LAUNCH_LOG}" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_service "/xmate3/cobot/connect" 120
wait_for_service "/xmate3/cobot/set_motion_control_mode" 60

STRESS_EXE="${WORKSPACE_ROOT}/install/rokae_xmate3_ros2/bin/example_27_rt_1khz_stress"
if [[ ! -x "${STRESS_EXE}" ]]; then
  STRESS_EXE="${WORKSPACE_ROOT}/build/rokae_xmate3_ros2/example_27_rt_1khz_stress"
fi
if [[ ! -x "${STRESS_EXE}" ]]; then
  echo "rt_1khz_stress: missing executable example_27_rt_1khz_stress" >&2
  exit 1
fi

EXAMPLE_TIMEOUT="$(python3 - <<'PY'
import math
import os
duration = float(os.environ.get("ROKAE_RT_STRESS_DURATION", "600"))
print(max(120, int(math.ceil(duration + 180.0))))
PY
)"
export ROKAE_RT_STRESS_DURATION="${DURATION_SEC}"

echo "[rt_1khz_stress] running example_27_rt_1khz_stress duration=${DURATION_SEC}s warmup=${WARMUP_SEC}s"
if ! timeout "${EXAMPLE_TIMEOUT}" \
    "${STRESS_EXE}" \
      --duration "${DURATION_SEC}" \
      --warmup "${WARMUP_SEC}" >"${EXAMPLE_LOG}" 2>&1; then
  cat "${EXAMPLE_LOG}" || true
  echo "rt_1khz_stress: stress example failed or timed out" >&2
  exit 1
fi

if ! timeout 20 ros2 topic echo /xmate3/internal/runtime_status --once >"${DIAG_LOG}" 2>&1; then
  cat "${DIAG_LOG}" || true
  echo "rt_1khz_stress: failed to capture runtime status topic" >&2
  exit 1
fi

python3 - "${EXAMPLE_LOG}" "${DIAG_LOG}" <<'PY'
import json
import re
import sys
from pathlib import Path

example_path = Path(sys.argv[1])
diag_path = Path(sys.argv[2])
example_text = example_path.read_text(encoding="utf-8", errors="replace")
diag_text = diag_path.read_text(encoding="utf-8", errors="replace")

metric_match = re.search(r"(?m)^RT_STRESS_JSON\s+(\{.*\})\s*$", example_text)
if metric_match is None:
    if "[skipped]" in example_text or "RT controller unavailable" in example_text:
      print(
          "rt_1khz_stress: FAIL classification=scheduler_not_active "
          "reason=rt_controller_unavailable_in_current_backend",
          file=sys.stderr,
      )
      raise SystemExit(1)
    print("rt_1khz_stress: missing RT_STRESS_JSON metrics line", file=sys.stderr)
    print(example_text, file=sys.stderr)
    raise SystemExit(1)

try:
    metrics = json.loads(metric_match.group(1))
except Exception as exc:
    print(f"rt_1khz_stress: invalid RT_STRESS_JSON payload: {exc}", file=sys.stderr)
    raise SystemExit(1)

def parse_diag_scalar(name: str, default: str = "") -> str:
    m = re.search(rf"(?m)^\s*{re.escape(name)}:\s*(.+?)\s*$", diag_text)
    if m is None:
        return default
    return m.group(1).strip().strip('"').strip("'")

def parse_diag_float(name: str, default: float = 0.0) -> float:
    raw = parse_diag_scalar(name, "")
    if not raw:
        return default
    try:
        return float(raw)
    except Exception:
        return default

avg_hz = float(metrics.get("avg_hz", 0.0))
p95_ms = float(metrics.get("p95_ms", 0.0))
p99_ms = float(metrics.get("p99_ms", 0.0))
samples = int(metrics.get("samples", 0))

rt_scheduler_state = parse_diag_scalar("rt_scheduler_state", "unknown")
rt_transport_source = parse_diag_scalar("rt_transport_source", "unknown")
rt_deadline_miss = parse_diag_float("rt_deadline_miss", 0.0)
rt_max_gap_ms = parse_diag_float("rt_max_gap_ms", 0.0)

ok = (
    samples > 100 and
    avg_hz >= 995.0 and
    p95_ms <= 1.05 and
    p99_ms <= 1.20
)

if ok:
    print(
        f"rt_1khz_stress: PASS samples={samples} avg_hz={avg_hz:.3f} "
        f"p95_ms={p95_ms:.3f} p99_ms={p99_ms:.3f} "
        f"transport={rt_transport_source} scheduler={rt_scheduler_state}"
    )
    raise SystemExit(0)

classification = "executor_contention"
lower_sched = rt_scheduler_state.lower()
if "degraded" in lower_sched or "failed" in lower_sched or "unknown" == lower_sched:
    classification = "scheduler_not_active"
elif rt_transport_source in {"legacy_custom_data", "unknown"}:
    classification = "transport_bottleneck"
elif rt_deadline_miss > 0.0 or rt_max_gap_ms > 1.2:
    classification = "backend_dt_jitter"

print(
    f"rt_1khz_stress: FAIL samples={samples} avg_hz={avg_hz:.3f} "
    f"p95_ms={p95_ms:.3f} p99_ms={p99_ms:.3f} "
    f"transport={rt_transport_source} scheduler={rt_scheduler_state} "
    f"rt_deadline_miss={rt_deadline_miss:.0f} rt_max_gap_ms={rt_max_gap_ms:.3f} "
    f"classification={classification}",
    file=sys.stderr,
)
raise SystemExit(1)
PY

echo "[rt_1khz_stress] success"
echo "[rt_1khz_stress] launch log: ${LAUNCH_LOG}"
echo "[rt_1khz_stress] example log: ${EXAMPLE_LOG}"
echo "[rt_1khz_stress] runtime status log: ${DIAG_LOG}"
