#!/usr/bin/env python3
"""Render the robot_description parameter from either a canonical URDF artifact or xacro.

This helper keeps launch logic deterministic: when the selected model path already points to a
URDF artifact we emit it verbatim, otherwise we invoke xacro with the requested launch flags.
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--package-share', required=True)
    parser.add_argument('--mesh-root', required=True)
    parser.add_argument('--enable-ros2-control', required=True)
    parser.add_argument('--enable-xcore-plugin', required=True)
    parser.add_argument('--backend-mode', required=True)
    parser.add_argument('--canonical-model', default='')
    parser.add_argument('--allow-noncanonical-model', default='false')
    return parser.parse_args()


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def main() -> int:
    args = _parse_args()
    model_path = pathlib.Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f'model path does not exist: {model_path}')

    canonical_path = pathlib.Path(args.canonical_model) if args.canonical_model else None
    allow_noncanonical = _as_bool(args.allow_noncanonical_model)
    if canonical_path and canonical_path.exists() and model_path.resolve() != canonical_path.resolve() and not allow_noncanonical:
        raise RuntimeError(
            f'non-canonical model override is disabled by default: {model_path}. '
            f'Use allow_noncanonical_model:=true only for developer-mode validation.'
        )

    if model_path.suffix.lower() != '.xacro':
        sys.stdout.write(model_path.read_text(encoding='utf-8'))
        return 0

    cmd = [
        'xacro',
        str(model_path),
        f'mesh_root:={args.mesh_root}',
        f'package_share:={args.package_share}',
        f'enable_ros2_control:={args.enable_ros2_control}',
        f'enable_xcore_plugin:={args.enable_xcore_plugin}',
        f'backend_mode:={args.backend_mode}',
    ]
    completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
    sys.stdout.write(completed.stdout)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
