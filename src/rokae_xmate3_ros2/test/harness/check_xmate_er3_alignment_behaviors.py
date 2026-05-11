#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []
manifest = json.loads((ROOT / 'docs/reference/xmate_er3_alignment_manifest.json').read_text(encoding='utf-8'))
doc = (ROOT / 'docs/reference/SDK_ALIGNMENT.md').read_text(encoding='utf-8')
cmake_text = (ROOT / 'cmake/targets_tests.cmake').read_text(encoding='utf-8')
quick_gate_text = (ROOT / 'tools' / 'run_quick_gate.sh').read_text(encoding='utf-8')
release_gate_text = (ROOT / 'tools' / 'run_release_gate.sh').read_text(encoding='utf-8')
acceptance_layers_text = (ROOT / 'tools' / 'run_acceptance_layers.sh').read_text(encoding='utf-8')

behavior = manifest.get('behavior_evidence', {})
unit_tests = behavior.get('unit_tests', [])
if len(unit_tests) < 3:
    FAILURES.append('behavior_evidence.unit_tests must list the runtime semantic evidence set')
for rel in unit_tests:
    path = ROOT / rel
    if not path.is_file():
        FAILURES.append(f'missing behavior evidence test file: {rel}')

for token in [
    'test_move_queue_semantics.cpp',
    'test_service_facade.cpp',
    'test_runtime_state_machine.cpp',
    'check_xmate_er3_alignment_behaviors.py',
    'runtime_publish_bridge.cpp',
    'robot_motion.cpp',
    'robot_model.cpp',
]:
    if token not in doc:
        FAILURES.append(f'SDK_ALIGNMENT.md missing behavior evidence token: {token}')

required_test_tokens = behavior.get('required_test_tokens', {})
for rel, tokens in required_test_tokens.items():
    path = ROOT / rel
    if not path.is_file():
        FAILURES.append(f'missing required test token file: {rel}')
        continue
    content = path.read_text(encoding='utf-8')
    for token in tokens:
        if token not in content:
            FAILURES.append(f'{rel} missing required behavior token: {token}')

execution_gate = behavior.get('execution_gate', '')
if execution_gate != 'tools/run_xmate_er3_alignment_behavior_gate.sh':
    FAILURES.append('behavior_evidence.execution_gate mismatch')
if execution_gate and not (ROOT / execution_gate).is_file():
    FAILURES.append(f'missing xmate_er3 alignment execution gate: {execution_gate}')
required_cmake_labels = behavior.get('required_cmake_labels', {})
for target_name, labels in required_cmake_labels.items():
    if f'set_tests_properties({target_name} PROPERTIES LABELS "' not in cmake_text:
        FAILURES.append(f'cmake/targets_tests.cmake missing label registration for {target_name}')
        continue
    for label in labels:
        if label not in cmake_text:
            FAILURES.append(f'cmake/targets_tests.cmake missing label token {label} for {target_name}')

for anchor in behavior.get('source_anchors', []):
    rel = anchor.get('file', '')
    token = anchor.get('contains', '')
    path = ROOT / rel
    if not path.is_file():
        FAILURES.append(f'missing source anchor file: {rel}')
        continue
    content = path.read_text(encoding='utf-8')
    if token not in content:
        FAILURES.append(f'{rel} missing source anchor token: {token}')

if behavior.get('static_gate') != 'test/harness/check_xmate_er3_alignment_behaviors.py':
    FAILURES.append('behavior_evidence.static_gate mismatch')
if execution_gate:
    gate_text = (ROOT / execution_gate).read_text(encoding='utf-8')
    for token in ('ctest -L xmate_er3_alignment', 'usage: $0 <workspace-root>'):
        if token not in gate_text:
            FAILURES.append(f'{execution_gate} missing execution token: {token}')
for gate_name, gate_text in [('run_quick_gate.sh', quick_gate_text), ('run_release_gate.sh', release_gate_text), ('run_acceptance_layers.sh', acceptance_layers_text)]:
    if 'run_xmate_er3_alignment_behavior_gate.sh' not in gate_text and 'ctest -L xmate_er3_alignment' not in gate_text:
        FAILURES.append(f'{gate_name} must bind the xmate_er3 alignment execution gate into the default chain')

if FAILURES:
    print('xmate_er3 alignment behavior check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('xmate_er3 alignment behavior check passed')
