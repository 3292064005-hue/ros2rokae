#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PASSED = 'passed'
NOT_APPLICABLE = 'not_applicable'
NOT_REQUESTED = 'not_requested'
REQUIRED_EVIDENCE_LOGS = ('environment_check', 'rosdep_install', 'full_source_gate', 'quick_gate')
REQUIRED_TOOL_KEYS = ('colcon', 'ros2', 'rosdep', 'gazebo', 'pkg_gazebo_ros')
FULL_SOURCE_SUCCESS_MARKER = 'full-source-build-gate: passed'
ENVIRONMENT_SUCCESS_MARKER = 'environment-lock: OK'


def fail(message: str) -> None:
    print(f'target acceptance report verification failed: {message}', file=sys.stderr)
    raise SystemExit(1)


def require_status(status: dict[str, object], key: str, expected: set[str]) -> None:
    value = status.get(key)
    if value not in expected:
        fail(f'status.{key} must be one of {sorted(expected)}, got {value!r}')


def require_target_environment(payload: dict[str, object]) -> None:
    environment = payload.get('environment')
    if not isinstance(environment, dict):
        fail('missing object field: environment')
    if environment.get('ROS_DISTRO') != 'humble':
        fail(f"environment.ROS_DISTRO must be 'humble', got {environment.get('ROS_DISTRO')!r}")
    if str(environment.get('ROKAE_IGNORE_ENV_LOCK', '')).strip() == '1':
        fail('environment.ROKAE_IGNORE_ENV_LOCK must not bypass the target environment lock')

    tools = payload.get('tools')
    if not isinstance(tools, dict):
        fail('missing object field: tools')
    for key in REQUIRED_TOOL_KEYS:
        value = tools.get(key)
        if not isinstance(value, str) or not value.strip():
            fail(f'tools.{key} must be present and non-empty')
        if value.startswith('unavailable') or 'unavailable:' in value:
            fail(f'tools.{key} must be available in the verified target environment, got {value!r}')


def resolve_report_log(report_path: Path, log_value: object, log_key: str) -> Path:
    if not isinstance(log_value, str) or not log_value.strip():
        fail(f'artifacts.expected_logs.{log_key} must be a non-empty relative path')
    raw = Path(log_value)
    if raw.is_absolute():
        fail(f'artifacts.expected_logs.{log_key} must be relative to the report directory, got {log_value!r}')
    resolved = (report_path.parent / raw).resolve()
    report_dir = report_path.parent.resolve()
    try:
        resolved.relative_to(report_dir)
    except ValueError:
        fail(f'artifacts.expected_logs.{log_key} must not escape the report directory, got {log_value!r}')
    return resolved


def require_log_file(report_path: Path, expected_logs: dict[str, object], key: str, *, marker: str | None = None) -> None:
    if key not in expected_logs:
        fail(f'artifacts.expected_logs.{key} must be present')
    path = resolve_report_log(report_path, expected_logs[key], key)
    if not path.is_file():
        fail(f'artifacts.expected_logs.{key} file does not exist: {path}')
    try:
        content = path.read_text(encoding='utf-8', errors='replace')
    except OSError as exc:
        fail(f'artifacts.expected_logs.{key} file is not readable: {path}: {exc}')
    if marker is not None and marker not in content:
        fail(f'artifacts.expected_logs.{key} must contain marker {marker!r}: {path}')


def require_evidence_logs(report_path: Path, payload: dict[str, object], status: dict[str, object], *, release_requested: bool, launch_smoke_requested: bool) -> None:
    artifacts = payload.get('artifacts')
    if not isinstance(artifacts, dict):
        fail('missing object field: artifacts')
    expected_logs = artifacts.get('expected_logs')
    if not isinstance(expected_logs, dict):
        fail('artifacts.expected_logs must be an object')

    require_log_file(report_path, expected_logs, 'environment_check', marker=ENVIRONMENT_SUCCESS_MARKER)
    require_log_file(report_path, expected_logs, 'rosdep_install')
    require_log_file(report_path, expected_logs, 'full_source_gate', marker=FULL_SOURCE_SUCCESS_MARKER)
    require_log_file(report_path, expected_logs, 'quick_gate')

    if status.get('release_gate') == PASSED or release_requested:
        require_log_file(report_path, expected_logs, 'release_gate')
    if status.get('launch_smoke') == PASSED or launch_smoke_requested:
        require_log_file(report_path, expected_logs, 'launch_smoke')
    if payload.get('mode') == 'container':
        require_log_file(report_path, expected_logs, 'image_build')


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Verify that a target-environment acceptance report is sufficient evidence for '
            'non-replay full source-tree gate closure. This verifier does not run the gate; '
            'it rejects missing, failed, not-run, or evidence-free reports so release tooling '
            'cannot treat an absent target-environment run as a pass.'
        )
    )
    parser.add_argument('report', type=Path)
    parser.add_argument('--require-release-gate', action='store_true')
    parser.add_argument('--require-launch-smoke', action='store_true')
    args = parser.parse_args()

    if not args.report.is_file():
        fail(f'report not found: {args.report}')
    try:
        payload = json.loads(args.report.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        fail(f'invalid JSON: {exc}')

    status = payload.get('status')
    if not isinstance(status, dict):
        fail('missing object field: status')

    if payload.get('success') is not True:
        fail('top-level success must be true')
    require_status(status, 'acceptance', {PASSED})
    require_status(status, 'environment_check', {PASSED})
    require_status(status, 'rosdep_install', {PASSED})
    require_status(status, 'quick_gate', {PASSED})
    require_status(status, 'full_source_gate', {PASSED})

    mode = payload.get('mode')
    if mode == 'container':
        require_status(status, 'image_build', {PASSED})
    elif mode == 'local':
        require_status(status, 'image_build', {NOT_APPLICABLE})
    else:
        fail(f'mode must be local or container, got {mode!r}')

    release_requested = bool(payload.get('release_gate_requested')) or args.require_release_gate
    launch_smoke_requested = bool(payload.get('launch_smoke_requested')) or args.require_launch_smoke
    if release_requested:
        require_status(status, 'release_gate', {PASSED})
    else:
        require_status(status, 'release_gate', {NOT_REQUESTED, PASSED})
    if launch_smoke_requested:
        require_status(status, 'launch_smoke', {PASSED})
    else:
        require_status(status, 'launch_smoke', {NOT_REQUESTED, PASSED})

    failure_reason = status.get('failure_reason', '')
    if failure_reason not in ('', None):
        fail(f'failure_reason must be empty on success, got {failure_reason!r}')

    require_target_environment(payload)
    require_evidence_logs(
        args.report,
        payload,
        status,
        release_requested=release_requested,
        launch_smoke_requested=launch_smoke_requested,
    )

    print('target acceptance report verification passed')


if __name__ == '__main__':
    main()
