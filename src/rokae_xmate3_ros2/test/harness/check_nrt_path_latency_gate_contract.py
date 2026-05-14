#!/usr/bin/env python3
"""Static contract checks for the NRT path-planning latency gate."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PACKAGE_ROOT / "tools" / "run_nrt_path_latency_gate.sh"
PROBE = PACKAGE_ROOT / "test" / "harness" / "nrt_path_latency_probe.py"
MOVE_APPEND_ACTION = PACKAGE_ROOT / "action" / "MoveAppend.action"
MOVE_APPEND_REGISTRY = PACKAGE_ROOT / "src" / "runtime" / "move_append_action_registry.cpp"
COMMON = PACKAGE_ROOT / "test" / "harness" / "common.py"
TARGETS_TESTS = PACKAGE_ROOT / "cmake" / "targets_tests.cmake"


def require_snippets(text: str, snippets: dict[str, str]) -> list[str]:
    return [
        f"{description}: missing `{snippet}`"
        for snippet, description in snippets.items()
        if snippet not in text
    ]


def main() -> int:
    failures: list[str] = []
    for path in (SCRIPT, PROBE, MOVE_APPEND_ACTION, MOVE_APPEND_REGISTRY, COMMON, TARGETS_TESTS):
        if not path.is_file():
            failures.append(f"required file missing: {path.relative_to(PACKAGE_ROOT)}")
    if failures:
        print("check_nrt_path_latency_gate_contract: FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    script = SCRIPT.read_text(encoding="utf-8")
    probe = PROBE.read_text(encoding="utf-8")
    action = MOVE_APPEND_ACTION.read_text(encoding="utf-8")
    registry = MOVE_APPEND_REGISTRY.read_text(encoding="utf-8")
    common = COMMON.read_text(encoding="utf-8")
    targets = TARGETS_TESTS.read_text(encoding="utf-8")

    failures.extend(require_snippets(script, {
        "headless|gazebo|both": "runner must expose both strict and diagnostic tiers",
        "runtime_profile:=nrt_strict_parity": "headless tier must use NRT strict parity",
        "public_xmate_er3_jtc": "Gazebo tier must target the public JTC profile",
        "classification=gazebo_diagnostic_latency": "Gazebo latency failures must be diagnostic-only",
        "nrt_path_latency_gate: FAIL tier=headless": "headless tier must be strict",
    }))
    failures.extend(require_snippets(probe, {
        '"move_append_accept_ms": 100.0': "MoveAppend accept SLA must be 100ms",
        '"move_append_result_ms": 150.0': "queue result SLA must be 150ms",
        '"move_start_to_planning_ms": 50.0': "planning dispatch SLA must be 50ms",
        '"planning_to_plan_queued_ms": 100.0': "planner latency SLA must be 100ms",
        '"plan_queued_to_execution_started_ms": 50.0': "execution dispatch SLA must be 50ms",
        '"move_start_to_execution_started_ms": 200.0': "start-to-execution SLA must be 200ms",
        "NRT_PATH_LATENCY_JSON": "probe must emit machine-readable JSON",
        "require_jtc=args.mode == \"gazebo\"": "Gazebo tier must wait for JTC readiness outside latency timing",
        "Accepted new action goal": "Gazebo tier must use JTC goal acceptance as execution-start diagnostic",
        "action_accept_slow": "probe must classify slow action accept",
        "queue_result_slow": "probe must classify slow queue result",
        "planning_dispatch_slow": "probe must classify slow planning dispatch",
        "planner_latency_regression": "probe must classify planner latency regression",
        "execution_start_slow": "probe must classify slow execution start",
        "runtime_event_missing": "probe must classify missing runtime events",
    }))
    failures.extend(require_snippets(common, {
        '"rokae_sim_runtime"': "telemetry must accept headless daemon runtime logs",
        '"xcore_gazebo_controller"': "telemetry must accept Gazebo plugin runtime logs",
    }))
    failures.extend(require_snippets(registry, {
        "MoveAppend is queue-only": "MoveAppend action must remain queue-only",
        "must not\n *       wait for execution terminal states": "MoveAppend must not wait for terminal execution",
    }))
    failures.extend(require_snippets(targets, {
        "nrt_path_latency_gate_contract": "latency gate contract must be registered in CTest",
        "check_nrt_path_latency_gate_contract.py": "CMake must call the latency gate contract checker",
        "quick_gate;semantic_gate;contract_gate": "latency contract must be in quick/semantic labels",
    }))

    expected_action_tokens = [
        "rokae_xmate3_ros2/MoveAbsJCommand[] absj_cmds",
        "rokae_xmate3_ros2/MoveJCommand[] j_cmds",
        "rokae_xmate3_ros2/MoveLCommand[] l_cmds",
        "rokae_xmate3_ros2/MoveCCommand[] c_cmds",
        "rokae_xmate3_ros2/MoveCFCommand[] cf_cmds",
        "rokae_xmate3_ros2/MoveSPCommand[] sp_cmds",
        "bool success",
        "string cmd_id",
        "string message",
        "float64 progress",
        "string current_state",
        "int32 current_cmd_index",
    ]
    for token in expected_action_tokens:
        if token not in action:
            failures.append(f"MoveAppend wire shape token missing: `{token}`")

    if failures:
        print("check_nrt_path_latency_gate_contract: FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("check_nrt_path_latency_gate_contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
