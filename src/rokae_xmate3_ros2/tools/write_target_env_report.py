#!/usr/bin/env python3
import argparse
import json
import os
import platform
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


VALID_STATUSES = {
    'not_run',
    'not_requested',
    'not_applicable',
    'passed',
    'failed',
}


def command_output(cmd):
    try:
        proc = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        return proc.stdout.strip()
    except Exception as exc:
        return f"unavailable: {exc}"


def git_revision(repo_root: Path) -> str:
    git = shutil.which("git")
    if git is None:
        return "unavailable: git not found"
    if not (repo_root / '.git').exists():
        return "unavailable: .git not present"
    return command_output([git, '-C', str(repo_root), 'rev-parse', 'HEAD'])


def validate_status(name: str, value: str) -> str:
    if value not in VALID_STATUSES:
      raise ValueError(f"{name} must be one of {sorted(VALID_STATUSES)}, got: {value}")
    return value


def main():
    parser = argparse.ArgumentParser(description='Write a target-environment acceptance report')
    parser.add_argument('--workspace-root', required=True)
    parser.add_argument('--package-root', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--mode', required=True, choices=['local', 'container'])
    parser.add_argument('--release-gate', action='store_true')
    parser.add_argument('--launch-smoke', action='store_true')
    parser.add_argument('--environment-check-status', default='not_run')
    parser.add_argument('--rosdep-install-status', default='not_run')
    parser.add_argument('--quick-gate-status', default='not_run')
    parser.add_argument('--release-gate-status', default='not_requested')
    parser.add_argument('--launch-smoke-status', default='not_requested')
    parser.add_argument('--image-build-status', default='not_run')
    parser.add_argument('--acceptance-status', default='failed')
    parser.add_argument('--failure-reason', default='')
    args = parser.parse_args()

    ws_root = Path(args.workspace_root).resolve()
    pkg_root = Path(args.package_root).resolve()
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    environment_check_status = validate_status('environment_check_status', args.environment_check_status)
    rosdep_install_status = validate_status('rosdep_install_status', args.rosdep_install_status)
    quick_gate_status = validate_status('quick_gate_status', args.quick_gate_status)
    release_gate_status = validate_status('release_gate_status', args.release_gate_status)
    launch_smoke_status = validate_status('launch_smoke_status', args.launch_smoke_status)
    image_build_status = validate_status('image_build_status', args.image_build_status)
    acceptance_status = 'passed' if args.acceptance_status == 'passed' else 'failed'

    report = {
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'mode': args.mode,
        'workspace_root': str(ws_root),
        'package_root': str(pkg_root),
        'release_gate_requested': args.release_gate,
        'launch_smoke_requested': args.launch_smoke,
        'status': {
            'environment_check': environment_check_status,
            'rosdep_install': rosdep_install_status,
            'quick_gate': quick_gate_status,
            'release_gate': release_gate_status,
            'launch_smoke': launch_smoke_status,
            'image_build': image_build_status,
            'acceptance': acceptance_status,
            'failure_reason': args.failure_reason,
        },
        'success': acceptance_status == 'passed',
        'os': {
            'platform': platform.platform(),
            'system': platform.system(),
            'release': platform.release(),
            'machine': platform.machine(),
        },
        'environment': {
            'ROS_DISTRO': os.environ.get('ROS_DISTRO', ''),
            'ROKAE_IGNORE_ENV_LOCK': os.environ.get('ROKAE_IGNORE_ENV_LOCK', ''),
            'ROKAE_PYTHON_EXECUTABLE': os.environ.get('ROKAE_PYTHON_EXECUTABLE', ''),
            'ROKAE_CONTAINER_ENGINE': os.environ.get('ROKAE_CONTAINER_ENGINE', ''),
            'ROKAE_ACCEPTANCE_IMAGE_TAG': os.environ.get('ROKAE_ACCEPTANCE_IMAGE_TAG', ''),
        },
        'tools': {
            'python3': command_output(['python3', '--version']),
            'colcon': command_output(['colcon', '--help']),
            'ros2': command_output(['ros2', '--help']),
            'rosdep': command_output(['rosdep', '--help']),
            'gazebo': command_output(['gazebo', '--version']),
            'pkg_gazebo_ros': command_output(['ros2', 'pkg', 'prefix', 'gazebo_ros']),
        },
        'artifacts': {
            'expected_logs': {
                'environment_check': f"environment_check_{args.mode}.log",
                'rosdep_install': f"rosdep_install_{args.mode}.log",
                'quick_gate': f"quick_gate_{args.mode}.log",
                'release_gate': f"release_gate_{args.mode}.log",
                'launch_smoke': f"launch_smoke_{args.mode}.log",
                'image_build': 'image_build.log' if args.mode == 'container' else '',
            },
        },
        'git_revision': git_revision(pkg_root),
    }
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding='utf-8')


if __name__ == '__main__':
    main()
