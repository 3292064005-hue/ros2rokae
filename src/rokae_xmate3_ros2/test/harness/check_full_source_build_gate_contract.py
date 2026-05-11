#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []

script = ROOT / 'tools' / 'run_full_source_tree_build_gate.sh'
if not script.is_file():
    FAILURES.append('missing tools/run_full_source_tree_build_gate.sh')
else:
    text = script.read_text(encoding='utf-8')
    required = [
        'check_target_environment.sh',
        'run_static_sanity.sh',
        'colcon build',
        '--packages-select "${PKG_NAME}"',
        '-DROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF',
        'ctest --test-dir "${BUILD_PKG_DIR}" -L',
        'ROKAE_PUBLIC_SDK_REPLAY_ONLY must be OFF',
        'workspace root containing src/${PKG_NAME}, or the ${PKG_NAME} package root',
    ]
    for token in required:
        if token not in text:
            FAILURES.append(f'full source build gate missing required token: {token}')
    if '-DROKAE_PUBLIC_SDK_REPLAY_ONLY=ON' in text:
        FAILURES.append('full source build gate must not configure replay-only mode')


for gate_name in ['run_quick_gate.sh', 'run_release_gate.sh']:
    gate_text = (ROOT / 'tools' / gate_name).read_text(encoding='utf-8')
    if 'run_full_source_tree_build_gate.sh' not in gate_text:
        FAILURES.append(f'{gate_name} must execute the non-replay full source-tree build gate instead of a raw colcon-only build')
    if 'colcon build --packages-select' in gate_text:
        FAILURES.append(f'{gate_name} must delegate source build ownership to run_full_source_tree_build_gate.sh')

report_writer = (ROOT / 'tools' / 'write_target_env_report.py').read_text(encoding='utf-8')
if '--full-source-gate-status' not in report_writer or "'full_source_gate'" not in report_writer:
    FAILURES.append('target environment report must expose full_source_gate status explicitly')

target_acceptance = (ROOT / 'tools' / 'run_target_env_acceptance.sh').read_text(encoding='utf-8')
if 'FULL_SOURCE_GATE_STATUS' not in target_acceptance:
    FAILURES.append('target environment acceptance must propagate full source gate status into its report')
if '--full-source-gate-status "${FULL_SOURCE_GATE_STATUS}"' not in target_acceptance:
    FAILURES.append('target environment acceptance report call must include --full-source-gate-status')
if 'verify_target_env_acceptance_report.py' not in target_acceptance:
    FAILURES.append('target environment acceptance must verify successful reports before returning success')

if 'full_source_gate_local.log' not in target_acceptance or 'full_source_gate_container.log' not in target_acceptance:
    FAILURES.append('target environment acceptance must capture dedicated full source gate evidence logs')
if 'check_target_environment.sh" --quiet' in target_acceptance:
    FAILURES.append('target environment acceptance must not run target environment check in quiet mode because verifier requires an evidence marker')

verifier = ROOT / 'tools' / 'verify_target_env_acceptance_report.py'
if not verifier.is_file():
    FAILURES.append('missing tools/verify_target_env_acceptance_report.py')
else:
    verifier_text = verifier.read_text(encoding='utf-8')
    for token in ['full_source_gate', 'environment_check', 'rosdep_install', 'quick_gate', 'success', 'require_evidence_logs', 'require_target_environment', 'FULL_SOURCE_SUCCESS_MARKER']:
        if token not in verifier_text:
            FAILURES.append(f'target acceptance verifier missing required token: {token}')

report_contract = ROOT / 'test' / 'harness' / 'check_target_env_acceptance_report_contract.py'
if not report_contract.is_file():
    FAILURES.append('missing test/harness/check_target_env_acceptance_report_contract.py')

static_sanity = (ROOT / 'tools' / 'run_static_sanity.sh').read_text(encoding='utf-8')
for token in [
    'check_runtime_source_integrity.py',
    'check_full_source_build_gate_contract.py',
]:
    if token not in static_sanity:
        FAILURES.append(f'run_static_sanity.sh must execute {token}')

targets_tests = (ROOT / 'cmake' / 'targets_tests.cmake').read_text(encoding='utf-8')
if 'NAME full_source_build_gate_contract' not in targets_tests:
    FAILURES.append('cmake/targets_tests.cmake must register full_source_build_gate_contract')
if 'check_full_source_build_gate_contract.py' not in targets_tests:
    FAILURES.append('full_source_build_gate_contract must call check_full_source_build_gate_contract.py')

build_release = (ROOT / 'docs' / 'release' / 'BUILD_RELEASE.md').read_text(encoding='utf-8')
if 'run_full_source_tree_build_gate.sh' not in build_release:
    FAILURES.append('BUILD_RELEASE.md must document the non-replay full source-tree build gate')
if 'ROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF' not in build_release:
    FAILURES.append('BUILD_RELEASE.md must state that the full source-tree build gate runs with replay-only OFF')
if '实机' not in build_release and 'hardware' not in build_release.lower():
    FAILURES.append('BUILD_RELEASE.md must explicitly separate source build validation from real hardware validation')

quickstart = (ROOT / 'docs' / 'public' / 'QUICKSTART.md').read_text(encoding='utf-8')
if 'install_tree_core_only' not in quickstart:
    FAILURES.append('QUICKSTART.md must mention the split core-only install-tree consumer sample')
if 'install_tree_runtime_components' not in quickstart:
    FAILURES.append('QUICKSTART.md must mention the split runtime-components install-tree consumer sample')

if FAILURES:
    print('full source build gate contract check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('full source build gate script contract check passed')
