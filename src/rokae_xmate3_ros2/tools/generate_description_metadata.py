#!/usr/bin/env python3
"""Generate canonical robot-description metadata for traceability."""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
from datetime import datetime, timezone


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--source-xacro', required=True)
    parser.add_argument('--mesh-root', default='')
    parser.add_argument('--enable-ros2-control', required=True)
    parser.add_argument('--enable-xcore-plugin', required=True)
    parser.add_argument('--backend-mode', required=True)
    parser.add_argument('--service-exposure-profile', required=True)
    parser.add_argument('--source-xacro-package-relative', default='urdf/xMateER3.xacro')
    parser.add_argument('--robot-family', default='xMateER3')
    parser.add_argument('--robot-model', default='xMateER3')
    parser.add_argument('--identity-scope', default='canonical')
    parser.add_argument('--canonical-identity', default='xCoreSDK:xmate_er3')
    parser.add_argument('--canonical-source-xacro', default='urdf/xMateER3.xacro')
    parser.add_argument('--compatibility-alias-policy', default='canonical_only')
    parser.add_argument('--default-launch-profile', default='')
    parser.add_argument('--default-runtime-profile', default='')
    parser.add_argument('--default-runtime-host', default='')
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    urdf_path = pathlib.Path(args.urdf)
    xml = urdf_path.read_bytes()
    payload = {
        'schema_version': 2,
        'generated_at_utc': datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        'canonical_urdf_path': str(urdf_path),
        'robot_family': args.robot_family,
        'robot_model': args.robot_model,
        'identity_scope': args.identity_scope,
        'canonical_identity': args.canonical_identity,
        'canonical_source_xacro': args.canonical_source_xacro,
        'source_xacro': args.source_xacro,
        'source_xacro_package_relative': args.source_xacro_package_relative,
        'runtime_host_policy': {
            'default_runtime_host': args.default_runtime_host,
            'default_runtime_profile': args.default_runtime_profile,
            'default_launch_profile': args.default_launch_profile,
            'compatibility_alias_policy': args.compatibility_alias_policy,
        },
        'xacro_args': {
            'mesh_root': args.mesh_root,
            'enable_ros2_control': args.enable_ros2_control,
            'enable_xcore_plugin': args.enable_xcore_plugin,
            'backend_mode': args.backend_mode,
            'service_exposure_profile': args.service_exposure_profile,
            'compatibility_alias_policy': args.compatibility_alias_policy,
        },
        'sha256': hashlib.sha256(xml).hexdigest(),
        'size_bytes': len(xml),
    }
    output_path = pathlib.Path(args.output)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
