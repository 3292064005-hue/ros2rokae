#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []

program_header = (ROOT / 'src/runtime/program_state.hpp').read_text(encoding='utf-8')
program_cpp = (ROOT / 'src/runtime/program_state.cpp').read_text(encoding='utf-8')
controller_header = (ROOT / 'src/runtime/controller_state.hpp').read_text(encoding='utf-8')
controller_cpp = (ROOT / 'src/runtime/controller_state.cpp').read_text(encoding='utf-8')
path_facade_cpp = (ROOT / 'src/runtime/path_facade.cpp').read_text(encoding='utf-8')

header_pattern = r'\[\[nodiscard\]\]\s+bool\s+saveRecordedPath\(const std::string &name, std::string \*error_message = nullptr\);'
impl_pattern = r'bool\s+ProgramState::saveRecordedPath\(const std::string &name, std::string \*error_message\)'
controller_header_pattern = r'\[\[nodiscard\]\]\s+bool\s+saveRecordedPath\(const std::string &name, std::string \*error_message = nullptr\);'
controller_impl_pattern = r'bool\s+ControllerState::saveRecordedPath\(const std::string &name, std::string \*error_message\)'

if not re.search(header_pattern, program_header):
    FAILURES.append('ProgramState::saveRecordedPath declaration must expose bool return and optional error_message')
if not re.search(impl_pattern, program_cpp):
    FAILURES.append('ProgramState::saveRecordedPath implementation signature mismatch')
if not re.search(controller_header_pattern, controller_header):
    FAILURES.append('ControllerState::saveRecordedPath declaration must expose bool return and optional error_message')
if not re.search(controller_impl_pattern, controller_cpp):
    FAILURES.append('ControllerState::saveRecordedPath implementation signature mismatch')
if 'program_state_.saveRecordedPath(target_name, &save_error)' not in path_facade_cpp:
    FAILURES.append('PathFacade must pass the saveRecordedPath error sink explicitly')

if FAILURES:
    print('cpp signature sync check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('cpp signature sync check passed')
