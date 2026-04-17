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
    parser.add_argument('--enable-ros2-control', required=True)
    parser.add_argument('--enable-xcore-plugin', required=True)
    parser.add_argument('--backend-mode', required=True)
    parser.add_argument('--service-exposure-profile', required=True)
    parser.add_argument('--source-xacro-package-relative', default='urdf/xMate3.xacro')
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    urdf_path = pathlib.Path(args.urdf)
    xml = urdf_path.read_bytes()
    payload = {
        'schema_version': 2,
        'generated_at_utc': datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        'canonical_urdf_path': str(urdf_path),
        'source_xacro': args.source_xacro,
        'source_xacro_package_relative': args.source_xacro_package_relative,
        'xacro_args': {
            'enable_ros2_control': args.enable_ros2_control,
            'enable_xcore_plugin': args.enable_xcore_plugin,
            'backend_mode': args.backend_mode,
            'service_exposure_profile': args.service_exposure_profile,
        },
        'sha256': hashlib.sha256(xml).hexdigest(),
        'size_bytes': len(xml),
    }
    output_path = pathlib.Path(args.output)
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
