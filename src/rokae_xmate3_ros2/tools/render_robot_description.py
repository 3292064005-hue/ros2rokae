#!/usr/bin/env python3
"""Render robot_description from a canonical URDF or xacro with deterministic install-tree behavior.

Behavior:
- Canonical URDF is used verbatim only when the requested description-shaping arguments match the
  build-time canonical metadata.
- When the selected model is the canonical URDF but launch requests a different backend/profile/plugin
  shape, the helper transparently re-expands the installed xacro described by canonical metadata.
- Explicit non-canonical model overrides remain developer-only and require allow_noncanonical_model:=true.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys
from typing import Dict


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--package-share', required=True)
    parser.add_argument('--mesh-root', required=True)
    parser.add_argument('--enable-ros2-control', required=True)
    parser.add_argument('--enable-xcore-plugin', required=True)
    parser.add_argument('--backend-mode', required=True)
    parser.add_argument('--service-exposure-profile', default='public_xmate6_only')
    parser.add_argument('--canonical-model', default='')
    parser.add_argument('--canonical-metadata', default='')
    parser.add_argument('--allow-noncanonical-model', default='false')
    return parser.parse_args()


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _load_metadata(path: pathlib.Path | None) -> Dict[str, object]:
    if path is None or not path.exists():
        return {}
    return json.loads(path.read_text(encoding='utf-8'))


def _requested_args(args: argparse.Namespace) -> Dict[str, str]:
    return {
        'enable_ros2_control': args.enable_ros2_control,
        'enable_xcore_plugin': args.enable_xcore_plugin,
        'backend_mode': args.backend_mode,
        'service_exposure_profile': args.service_exposure_profile,
    }


def _canonical_args(metadata: Dict[str, object]) -> Dict[str, str]:
    raw = metadata.get('xacro_args', {})
    if not isinstance(raw, dict):
        return {}
    result: Dict[str, str] = {}
    for key, value in raw.items():
        result[str(key)] = str(value)
    return result


def _resolve_xacro_from_metadata(package_share: pathlib.Path, metadata: Dict[str, object]) -> pathlib.Path:
    rel = metadata.get('source_xacro_package_relative')
    if isinstance(rel, str) and rel:
        candidate = package_share / rel
        if candidate.exists():
            return candidate
    fallback = package_share / 'urdf' / 'xMate3.xacro'
    if fallback.exists():
        return fallback
    source = metadata.get('source_xacro')
    if isinstance(source, str) and source:
        source_path = pathlib.Path(source)
        if source_path.exists():
            return source_path
    raise FileNotFoundError('unable to resolve xacro source from canonical metadata')


def _render_xacro(model_path: pathlib.Path, args: argparse.Namespace) -> str:
    cmd = [
        'xacro',
        str(model_path),
        f'mesh_root:={args.mesh_root}',
        f'package_share:={args.package_share}',
        f'enable_ros2_control:={args.enable_ros2_control}',
        f'enable_xcore_plugin:={args.enable_xcore_plugin}',
        f'backend_mode:={args.backend_mode}',
        f'service_exposure_profile:={args.service_exposure_profile}',
    ]
    completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return completed.stdout


def main() -> int:
    args = _parse_args()
    model_path = pathlib.Path(args.model)
    if not model_path.exists():
        raise FileNotFoundError(f'model path does not exist: {model_path}')

    canonical_path = pathlib.Path(args.canonical_model) if args.canonical_model else None
    canonical_metadata_path = pathlib.Path(args.canonical_metadata) if args.canonical_metadata else None
    allow_noncanonical = _as_bool(args.allow_noncanonical_model)
    if canonical_path and canonical_path.exists() and model_path.resolve() != canonical_path.resolve() and not allow_noncanonical:
        raise RuntimeError(
            f'non-canonical model override is disabled by default: {model_path}. '
            f'Use allow_noncanonical_model:=true only for developer-mode validation.'
        )

    if model_path.suffix.lower() == '.xacro':
        sys.stdout.write(_render_xacro(model_path, args))
        return 0

    metadata = _load_metadata(canonical_metadata_path)
    if canonical_path and canonical_path.exists() and model_path.resolve() == canonical_path.resolve() and metadata:
        requested = _requested_args(args)
        canonical = _canonical_args(metadata)
        if all(canonical.get(key) == value for key, value in requested.items()):
            sys.stdout.write(model_path.read_text(encoding='utf-8'))
            return 0
        xacro_path = _resolve_xacro_from_metadata(pathlib.Path(args.package_share), metadata)
        sys.stdout.write(_render_xacro(xacro_path, args))
        return 0

    sys.stdout.write(model_path.read_text(encoding='utf-8'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
