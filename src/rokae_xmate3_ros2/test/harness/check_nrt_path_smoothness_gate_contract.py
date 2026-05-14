#!/usr/bin/env python3
"""Static contract checks for the NRT path-planning smoothness gate."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PACKAGE_ROOT / "tools" / "run_nrt_path_smoothness_gate.sh"
PROBE = PACKAGE_ROOT / "test" / "harness" / "nrt_path_smoothness_probe.py"
MOVE_APPEND_ACTION = PACKAGE_ROOT / "action" / "MoveAppend.action"
GENERATE_S_TRAJECTORY = PACKAGE_ROOT / "srv" / "GenerateSTrajectory.srv"
TARGETS_TESTS = PACKAGE_ROOT / "cmake" / "targets_tests.cmake"
STATIC_SANITY = PACKAGE_ROOT / "tools" / "run_static_sanity.sh"
UNIFIED_RETIMER_TEST = PACKAGE_ROOT / "test" / "unit" / "test_unified_retimer.cpp"
MOTION_PLANNER_TEST = PACKAGE_ROOT / "test" / "unit" / "test_motion_planner_core.cpp"


def require_snippets(text: str, snippets: dict[str, str]) -> list[str]:
    return [
        f"{description}: missing `{snippet}`"
        for snippet, description in snippets.items()
        if snippet not in text
    ]


def main() -> int:
    failures: list[str] = []
    for path in (
        SCRIPT,
        PROBE,
        MOVE_APPEND_ACTION,
        GENERATE_S_TRAJECTORY,
        TARGETS_TESTS,
        STATIC_SANITY,
        UNIFIED_RETIMER_TEST,
        MOTION_PLANNER_TEST,
    ):
        if not path.is_file():
            failures.append(f"required file missing: {path.relative_to(PACKAGE_ROOT)}")
    if failures:
        print("check_nrt_path_smoothness_gate_contract: FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    script = SCRIPT.read_text(encoding="utf-8")
    probe = PROBE.read_text(encoding="utf-8")
    action = MOVE_APPEND_ACTION.read_text(encoding="utf-8")
    srv = GENERATE_S_TRAJECTORY.read_text(encoding="utf-8")
    targets = TARGETS_TESTS.read_text(encoding="utf-8")
    static_sanity = STATIC_SANITY.read_text(encoding="utf-8")
    unified_test = UNIFIED_RETIMER_TEST.read_text(encoding="utf-8")
    planner_test = MOTION_PLANNER_TEST.read_text(encoding="utf-8")

    failures.extend(require_snippets(script, {
        "headless|gazebo|both": "runner must expose strict and diagnostic tiers",
        "runtime_profile:=nrt_strict_parity": "headless tier must use NRT strict parity",
        "public_xmate_er3_jtc": "Gazebo tier must target the public JTC profile",
        "classification=gazebo_diagnostic_smoothness": "Gazebo smoothness failures must be diagnostic-only",
        "nrt_path_smoothness_gate: FAIL tier=headless": "headless tier must be strict",
    }))
    failures.extend(require_snippets(probe, {
        "NRT_PATH_SMOOTHNESS_JSON": "probe must emit machine-readable JSON",
        '"min_point_count": 3': "probe must enforce minimum point count",
        '"max_endpoint_error_rad": 1e-3': "probe must enforce endpoint tolerance",
        '"velocity_limit_scale": 1.25': "probe must enforce velocity estimate guard",
        '"acceleration_limit_scale": 1.50': "probe must enforce acceleration estimate guard",
        '"jerk_p99_est_rad_s3": 500.0': "probe must enforce jerk p99 guard",
        '"jerk_max_est_rad_s3": 1500.0': "probe must enforce jerk max guard",
        "trajectory_empty": "probe must classify empty trajectories",
        "nonfinite_sample": "probe must classify non-finite samples",
        "endpoint_mismatch": "probe must classify endpoint mismatch",
        "velocity_limit_regression": "probe must classify velocity regressions",
        "acceleration_limit_regression": "probe must classify acceleration regressions",
        "jerk_spike": "probe must classify jerk spikes",
        "service_missing": "probe must classify missing service readiness",
        "runtime_error": "probe must classify runtime errors",
    }))
    failures.extend(require_snippets(unified_test, {
        "StrictJerkScalarProfileIsFiniteAndEndpointSmooth": "strict jerk profile smoothness test must exist",
        "PointToPointRetimingPassesStrictSmoothnessGate": "point-to-point smoothness test must exist",
        "MultiWaypointPathRetimingPassesStrictSmoothnessGate": "multi-waypoint smoothness test must exist",
        "expectSmoothCanonicalSamples": "C++ retimer test must check velocity/acceleration/jerk-related continuity",
    }))
    failures.extend(require_snippets(planner_test, {
        "PlannerOutputPassesStrictSmoothnessGateAcrossSegmentJoin": "planner smoothness test must exist",
        "expectPlannerSegmentJoinSmoothness": "planner test must verify segment join continuity",
        "expectPlannerSegmentSmoothness": "planner test must verify finite velocity/acceleration samples",
    }))
    failures.extend(require_snippets(targets, {
        "nrt_path_smoothness_gate_contract": "smoothness gate contract must be registered in CTest",
        "check_nrt_path_smoothness_gate_contract.py": "CMake must call the smoothness gate contract checker",
        "quick_gate;semantic_gate;contract_gate": "smoothness contract must be in quick/semantic labels",
    }))
    failures.extend(require_snippets(static_sanity, {
        "nrt_path_smoothness_probe.py": "static sanity must py_compile the smoothness probe",
        "check_nrt_path_smoothness_gate_contract.py": "static sanity must execute the smoothness contract",
    }))

    expected_action_tokens = [
        "rokae_xmate3_ros2/MoveAbsJCommand[] absj_cmds",
        "bool success",
        "string cmd_id",
        "string message",
    ]
    for token in expected_action_tokens:
        if token not in action:
            failures.append(f"MoveAppend wire shape token missing: `{token}`")

    expected_srv_tokens = [
        "float64[6] start_joint_pos",
        "float64[6] target_joint_pos",
        "float64 max_velocity",
        "float64 max_acceleration",
        "rokae_xmate3_ros2/JointPos6[] trajectory_points",
    ]
    for token in expected_srv_tokens:
        if token not in srv:
            failures.append(f"GenerateSTrajectory wire shape token missing: `{token}`")

    if failures:
        print("check_nrt_path_smoothness_gate_contract: FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("check_nrt_path_smoothness_gate_contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
