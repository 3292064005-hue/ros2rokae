#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []

doc = (ROOT / 'docs/release/ACCEPTANCE_LAYERS.md').read_text(encoding='utf-8')
manifest = json.loads((ROOT / 'docs/release/acceptance_layers_manifest.json').read_text(encoding='utf-8'))
release_gate = (ROOT / 'docs/release/RELEASE_GATE.md').read_text(encoding='utf-8')
build_release = (ROOT / 'docs/release/BUILD_RELEASE.md').read_text(encoding='utf-8')
packaging = (ROOT / 'cmake/targets_packaging.cmake').read_text(encoding='utf-8')

required_layers = ['L0', 'L1', 'L2', 'L3', 'L4', 'L5']
for layer in required_layers:
    if layer not in doc:
        FAILURES.append(f'ACCEPTANCE_LAYERS.md missing {layer}')

manifest_layers = manifest.get('layers', [])
if [entry.get('id') for entry in manifest_layers] != required_layers:
    FAILURES.append('acceptance_layers_manifest.json must enumerate L0-L5 in order')

required_entrypoints = [
    'tools/run_acceptance_layers.sh',
    'tools/run_real_dryrun_acceptance.sh',
    'tools/run_loaded_sensor_acceptance.sh',
    'tools/run_full_task_acceptance.sh',
    'tools/run_launch_smoke.sh',
    'tools/run_main_chain_smoke.sh',
    'tools/run_headless_sdk_smoke.sh',
    'tools/run_experimental_opt_in_smoke.sh',
]
for rel in required_entrypoints:
    if not (ROOT / rel).is_file():
        FAILURES.append(f'missing acceptance layer entrypoint: {rel}')
    if rel not in packaging:
        FAILURES.append(f'cmake/targets_packaging.cmake must install acceptance entrypoint token: {rel}')

if 'release gate 只能覆盖到 L0-L2' not in release_gate:
    FAILURES.append('RELEASE_GATE.md must state that release gate only covers L0-L2')

required_install_helpers = [
    'tools/check_runtime_diag_gate.py',
    'tools/derive_runtime_diag_gate.py',
    'tools/run_public_kinematics_probe.py',
]
if manifest.get('install_helpers') != required_install_helpers:
    FAILURES.append('acceptance_layers_manifest.json install_helpers mismatch')
for rel in required_install_helpers:
    if not (ROOT / rel).is_file():
        FAILURES.append(f'missing install helper: {rel}')
    if rel not in packaging:
        FAILURES.append(f'cmake/targets_packaging.cmake must install helper token: {rel}')

service_path_expectations = {
    'tools/run_real_dryrun_acceptance.sh': '${NAMESPACE}/cobot/get_runtime_diagnostics',
    'tools/run_full_task_acceptance.sh': '${NAMESPACE}/cobot/get_runtime_diagnostics',
}
for rel, expected in service_path_expectations.items():
    content = (ROOT / rel).read_text(encoding='utf-8')
    if expected not in content:
        FAILURES.append(f"{rel} must use canonical diagnostics service path: {expected}")
    if '${NAMESPACE}/internal/get_runtime_diagnostics' in content:
        FAILURES.append(f"{rel} must not call legacy internal diagnostics service path")

acceptance_scripts = [
    'tools/run_acceptance_layers.sh',
    'tools/run_launch_smoke.sh',
    'tools/run_main_chain_smoke.sh',
    'tools/run_headless_sdk_smoke.sh',
    'tools/run_experimental_opt_in_smoke.sh',
]
for rel in acceptance_scripts:
    content = (ROOT / rel).read_text(encoding='utf-8')
    if 'src/rokae_xmate3_ros2/tools/' in content:
        FAILURES.append(f'{rel} must not hard-code source-tree tool paths')

main_chain_text = (ROOT / 'tools/run_main_chain_smoke.sh').read_text(encoding='utf-8')
for token in ('SCRIPT_DIR', 'PACKAGE_ROOT', 'check_runtime_diag_gate.py'):
    if token not in main_chain_text:
        FAILURES.append(f'run_main_chain_smoke.sh missing install-tree token: {token}')
for token in ('run_public_kinematics_probe.py', 'get_end_wrench', 'get_runtime_state_snapshot', 'get_soft_limit'):
    if token not in main_chain_text:
        FAILURES.append(f'run_main_chain_smoke.sh missing public simulation completeness token: {token}')
headless_text = (ROOT / 'tools/run_headless_sdk_smoke.sh').read_text(encoding='utf-8')
for token in ('public_xmate_er3_headless_sdk_smoke', 'headless_sim', 'run_main_chain_smoke.sh'):
    if token not in headless_text:
        FAILURES.append(f'run_headless_sdk_smoke.sh missing token: {token}')
experimental_text = (ROOT / 'tools/run_experimental_opt_in_smoke.sh').read_text(encoding='utf-8')
for token in ('public_xmate_er3_experimental_rt', 'enable_drag', 'start_record_path', 'get_rt_joint_data'):
    if token not in experimental_text:
        FAILURES.append(f'run_experimental_opt_in_smoke.sh missing token: {token}')
if 'L0-L5 分层验收体系' not in build_release:
    FAILURES.append('BUILD_RELEASE.md must mention the L0-L5 acceptance system')
if 'share/rokae_xmate3_ros2/tools/' not in doc:
    FAILURES.append('ACCEPTANCE_LAYERS.md must document install-tree mirrored tool entrypoints')

if FAILURES:
    print('acceptance layers check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('acceptance layers check passed')
