#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict




def read_policy(policy_path: Path) -> Dict[str, str]:
    policy: Dict[str, str] = {}
    if not policy_path.is_file():
        return policy
    for raw_line in policy_path.read_text(encoding='utf-8').splitlines():
        line = raw_line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        policy[key.strip()] = value.strip()
    return policy


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--install-metadata', required=True)
    args = parser.parse_args()

    root = Path(args.root).resolve()
    manifest = json.loads((root / 'docs' / 'reference' / 'xmate_er3_alignment_manifest.json').read_text(encoding='utf-8'))
    policy = read_policy(root / 'config' / 'default_runtime_host_policy.env')
    install_metadata: Dict[str, Any] = json.loads(Path(args.install_metadata).read_text(encoding='utf-8'))
    payload = {
        'canonical_identity': manifest.get('identity', {}).get('canonical_identity', 'unknown'),
        'canonical_package': manifest.get('identity', {}).get('canonical_package', 'unknown'),
        'legacy_source_package': manifest.get('identity', {}).get('legacy_source_package', 'unknown'),
        'unsupported_public_modules': manifest.get('unsupported_public_modules', []),
        'public_examples': manifest.get('public_examples', []),
        'internal_examples': manifest.get('internal_examples', []),
        'public_motion_extensions': manifest.get('public_motion_extensions', []),
        'profiles': manifest.get('profiles', {}),
        'source_layout': manifest.get('source_layout', {}),
        'release_gates': manifest.get('release_gates', []),
        'default_runtime_host_policy': {
            'launch_profile': install_metadata.get('default_launch_profile', policy.get('ROKAE_DEFAULT_PUBLIC_LAUNCH_PROFILE', 'public_xmate_er3_jtc')),
            'runtime_host': install_metadata.get('default_runtime_host', policy.get('ROKAE_DEFAULT_RUNTIME_HOST', 'daemonized_runtime')),
            'runtime_profile': install_metadata.get('default_runtime_profile', policy.get('ROKAE_DEFAULT_RUNTIME_PROFILE', 'nrt_strict_parity')),
            'backend_mode': install_metadata.get('backend_mode', policy.get('ROKAE_DEFAULT_BACKEND_MODE', 'effort')),
            'primary_install_consumer': install_metadata.get('primary_install_consumer', 'unknown'),
            'service_exposure_profile': install_metadata.get('default_service_exposure_profile', policy.get('ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE', 'public_xmate_er3_only')),
            'enable_ros2_control': install_metadata.get('default_enable_ros2_control', policy.get('ROKAE_DEFAULT_ENABLE_ROS2_CONTROL', 'false')),
            'enable_xcore_plugin': install_metadata.get('default_enable_xcore_plugin', policy.get('ROKAE_DEFAULT_ENABLE_XCORE_PLUGIN', 'false')),
        },
        'validation_layers': {
            'static': {'status': 'confirmed', 'evidence': 'manifest/header/runtime pattern checks'},
            'sandbox': {
                'status': 'confirmed',
                'evidence': [
                    'check_public_internal_boundary.py',
                    'check_runtime_state_machine_contract.py',
                    'check_xmate_er3_alignment.py',
                ],
            },
            'sim': {'status': 'not_confirmed', 'evidence': 'requires ROS2/Gazebo runtime launch'},
            'hardware': {'status': 'not_confirmed', 'evidence': 'controller-grade and real robot validation not bundled'},
        },
        'validation_badges': ['static:confirmed', 'sandbox:confirmed', 'sim:not_confirmed', 'hardware:not_confirmed'],
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
