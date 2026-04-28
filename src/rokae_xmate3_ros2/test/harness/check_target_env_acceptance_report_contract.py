#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import json
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []

verifier = ROOT / 'tools' / 'verify_target_env_acceptance_report.py'
if not verifier.is_file():
    FAILURES.append('missing tools/verify_target_env_acceptance_report.py')
else:
    text = verifier.read_text(encoding='utf-8')
    for token in [
        'full_source_gate',
        'environment_check',
        'rosdep_install',
        'quick_gate',
        'acceptance',
        'require_release_gate',
        'require_launch_smoke',
        'success',
        'FULL_SOURCE_SUCCESS_MARKER',
        'ENVIRONMENT_SUCCESS_MARKER',
        'require_target_environment',
        'require_evidence_logs',
        'environment.ROS_DISTRO',
    ]:
        if token not in text:
            FAILURES.append(f'target-env report verifier missing required token: {token}')

acceptance = (ROOT / 'tools' / 'run_target_env_acceptance.sh').read_text(encoding='utf-8')
if 'verify_target_env_acceptance_report.py' not in acceptance:
    FAILURES.append('target-env acceptance script must verify success reports before returning success')
if 'acceptance_report_local.json' not in acceptance or 'acceptance_report_container.json' not in acceptance:
    FAILURES.append('target-env acceptance must verify both local and container acceptance report paths')
if '--full-source-gate-status "${FULL_SOURCE_GATE_STATUS}"' not in acceptance:
    FAILURES.append('target-env acceptance report call must pass full source gate status')
if 'full_source_gate_local.log' not in acceptance or 'full_source_gate_container.log' not in acceptance:
    FAILURES.append('target-env acceptance must capture dedicated full source gate evidence logs')
if 'run_full_source_tree_build_gate.sh' not in acceptance:
    FAILURES.append('target-env acceptance must execute the full source gate explicitly')
if 'check_target_environment.sh" --quiet' in acceptance:
    FAILURES.append('target-env acceptance must not run environment check in quiet mode because report evidence needs a success marker')

writer = (ROOT / 'tools' / 'write_target_env_report.py').read_text(encoding='utf-8')
if 'full_source_gate_{args.mode}.log' not in writer:
    FAILURES.append('target-env report writer must point full_source_gate evidence at a dedicated log file')
if 'quick_gate_{args.mode}.log' not in writer:
    FAILURES.append('target-env report writer must still expose quick_gate evidence')

static_sanity = (ROOT / 'tools' / 'run_static_sanity.sh').read_text(encoding='utf-8')
if 'check_target_env_acceptance_report_contract.py' not in static_sanity:
    FAILURES.append('run_static_sanity.sh must execute check_target_env_acceptance_report_contract.py')
if 'verify_target_env_acceptance_report.py' not in static_sanity:
    FAILURES.append('run_static_sanity.sh must py_compile verify_target_env_acceptance_report.py')

targets_tests = (ROOT / 'cmake' / 'targets_tests.cmake').read_text(encoding='utf-8')
if 'NAME target_env_acceptance_report_contract' not in targets_tests:
    FAILURES.append('cmake/targets_tests.cmake must register target_env_acceptance_report_contract')
if 'check_target_env_acceptance_report_contract.py' not in targets_tests:
    FAILURES.append('target_env_acceptance_report_contract must call check_target_env_acceptance_report_contract.py')


def write_logs(tmpdir: Path, *, full_marker: bool = True) -> None:
    (tmpdir / 'environment_check_local.log').write_text('environment-lock: OK (Ubuntu 22.04, ROS 2 Humble, Gazebo available)\n', encoding='utf-8')
    (tmpdir / 'rosdep_install_local.log').write_text('rosdep install completed\n', encoding='utf-8')
    (tmpdir / 'quick_gate_local.log').write_text('quick gate completed\n', encoding='utf-8')
    full_payload = 'full-source-build-gate: passed (non-replay source build and ctest labels: quick_gate;semantic_gate)\n' if full_marker else 'colcon build completed without marker\n'
    (tmpdir / 'full_source_gate_local.log').write_text(full_payload, encoding='utf-8')


def base_report(tmpdir: Path) -> dict[str, object]:
    return {
        'success': True,
        'mode': 'local',
        'release_gate_requested': False,
        'launch_smoke_requested': False,
        'status': {
            'environment_check': 'passed',
            'rosdep_install': 'passed',
            'quick_gate': 'passed',
            'full_source_gate': 'passed',
            'release_gate': 'not_requested',
            'launch_smoke': 'not_requested',
            'image_build': 'not_applicable',
            'acceptance': 'passed',
            'failure_reason': '',
        },
        'environment': {
            'ROS_DISTRO': 'humble',
            'ROKAE_IGNORE_ENV_LOCK': '',
        },
        'tools': {
            'python3': 'Python 3.10.12',
            'colcon': 'colcon help output',
            'ros2': 'ros2 help output',
            'rosdep': 'rosdep help output',
            'gazebo': 'Gazebo multi-robot simulator, version 11.10.2',
            'pkg_gazebo_ros': '/opt/ros/humble/share/gazebo_ros',
        },
        'artifacts': {
            'expected_logs': {
                'environment_check': 'environment_check_local.log',
                'rosdep_install': 'rosdep_install_local.log',
                'quick_gate': 'quick_gate_local.log',
                'full_source_gate': 'full_source_gate_local.log',
                'release_gate': 'release_gate_local.log',
                'launch_smoke': 'launch_smoke_local.log',
                'image_build': '',
            }
        },
    }


def run_verifier(report: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run([sys.executable, str(verifier), str(report)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


if verifier.is_file():
    with tempfile.TemporaryDirectory() as tmp:
        tmpdir = Path(tmp)
        write_logs(tmpdir)
        report = tmpdir / 'report.json'
        payload = base_report(tmpdir)

        payload['status']['full_source_gate'] = 'not_run'
        report.write_text(json.dumps(payload), encoding='utf-8')
        if run_verifier(report).returncode == 0:
            FAILURES.append('target-env report verifier must reject not_run full_source_gate status')

        payload = base_report(tmpdir)
        (tmpdir / 'full_source_gate_local.log').unlink()
        report.write_text(json.dumps(payload), encoding='utf-8')
        if run_verifier(report).returncode == 0:
            FAILURES.append('target-env report verifier must reject passed reports without full source gate log evidence')

        write_logs(tmpdir, full_marker=False)
        report.write_text(json.dumps(payload), encoding='utf-8')
        if run_verifier(report).returncode == 0:
            FAILURES.append('target-env report verifier must reject full source gate logs without the success marker')

        write_logs(tmpdir)
        payload = base_report(tmpdir)
        payload['environment']['ROS_DISTRO'] = 'rolling'
        report.write_text(json.dumps(payload), encoding='utf-8')
        if run_verifier(report).returncode == 0:
            FAILURES.append('target-env report verifier must reject non-Humble ROS_DISTRO evidence')

        payload = base_report(tmpdir)
        payload['tools']['colcon'] = 'unavailable: colcon not found'
        report.write_text(json.dumps(payload), encoding='utf-8')
        if run_verifier(report).returncode == 0:
            FAILURES.append('target-env report verifier must reject unavailable required tools')

        payload = base_report(tmpdir)
        report.write_text(json.dumps(payload), encoding='utf-8')
        proc = run_verifier(report)
        if proc.returncode != 0:
            FAILURES.append(f'target-env report verifier rejected a complete success report with evidence logs: {proc.stderr or proc.stdout}')

if FAILURES:
    print('target env acceptance report contract check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    raise SystemExit(1)

print('target env acceptance report contract check passed')
