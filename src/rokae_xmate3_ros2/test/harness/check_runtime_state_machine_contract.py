#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []

hpp = (ROOT / 'src/runtime/runtime_state_machine.hpp').read_text(encoding='utf-8')
cpp = (ROOT / 'src/runtime/runtime_state_machine.cpp').read_text(encoding='utf-8')
rt_types = (ROOT / 'src/runtime/runtime_types.hpp').read_text(encoding='utf-8')
execution_switch = (ROOT / 'src/runtime/runtime_execution_switch.cpp').read_text(encoding='utf-8')
doc = (ROOT / 'docs/reference/RUNTIME_STATE_MACHINE.md').read_text(encoding='utf-8')
manifest = json.loads((ROOT / 'docs/reference/runtime_state_machine_manifest.json').read_text(encoding='utf-8'))

for token in [
    'RuntimeStateMachine',
    'request_queued',
    'planning_requested',
    'planning_rejected',
    'plan_queued',
    'execution_started',
    'progress_updated',
    'trajectory_retimed',
    'paused',
    'watchdog_triggered',
    'completed',
    'completed_relaxed',
    'failed',
    'stopped',
    'owner_changed',
    'phase_override',
    'has_observed_state',
    'observed_state',
]:
    if token not in cpp and token not in hpp:
        FAILURES.append(f'runtime state machine source missing token: {token}')

for token in ['ExecutionState', 'idle,', 'planning,', 'queued,', 'executing,', 'paused,', 'settling,', 'completed,', 'completed_relaxed,', 'failed,', 'stopped,']:
    if token not in rt_types:
        FAILURES.append(f'runtime_types.hpp missing token: {token}')

for token in [
    'request_queued -> planning_requested -> plan_queued -> execution_started',
    'observed execution state',
    'settling',
    'RuntimeStateMachine',
]:
    if token not in doc:
        FAILURES.append(f'docs/reference/RUNTIME_STATE_MACHINE.md missing token: {token}')

if manifest.get('authority') != 'RuntimeStateMachine':
    FAILURES.append('runtime_state_machine_manifest.json authority mismatch')
if 'execution_started' not in manifest.get('events', []):
    FAILURES.append('runtime_state_machine_manifest.json missing execution_started')
if 'completed_relaxed' not in manifest.get('states', []):
    FAILURES.append('runtime_state_machine_manifest.json missing completed_relaxed state')
if not any('observed_state=queued|executing|settling' in item for item in manifest.get('main_chain', [])):
    FAILURES.append('runtime_state_machine_manifest.json must encode observed_state progression')

forbidden_patterns = [
    r'active_status_\.state\s*=(?!=)',
    r'active_status_\.message\s*=(?!=)',
    r'active_status_\.completed_segments\s*=(?!=)',
    r'active_status_\.current_segment_index\s*=(?!=)',
]
for pattern in forbidden_patterns:
    if re.search(pattern, execution_switch):
        FAILURES.append(f'runtime_execution_switch.cpp bypasses RuntimeStateMachine with direct status mutation: {pattern}')

if 'progress_event.has_observed_state = true;' not in execution_switch:
    FAILURES.append('runtime_execution_switch.cpp must emit observed-state progress events')

if FAILURES:
    print('runtime state machine contract check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('runtime state machine contract check passed')
