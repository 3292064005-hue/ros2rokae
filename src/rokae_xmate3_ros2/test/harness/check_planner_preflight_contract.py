#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SERVICE = ROOT / 'internal_interfaces' / 'srv' / 'PlannerPreflightReport.srv'
MANIFEST = (ROOT / 'src' / 'runtime' / 'service_contract_manifest.hpp').read_text(encoding='utf-8') + (ROOT / 'src' / 'runtime' / 'service_contract_manifest.cpp').read_text(encoding='utf-8')
FACADE = (ROOT / 'src' / 'runtime' / 'service_facade.hpp').read_text(encoding='utf-8')
QUERY = (ROOT / 'src' / 'runtime' / 'query_kinematics_service.cpp').read_text(encoding='utf-8')
BACKLOG = (ROOT / 'docs' / 'release' / 'HARDENING_BACKLOG.md').read_text(encoding='utf-8')
FAILURES: list[str] = []

if not SERVICE.is_file():
    FAILURES.append('PlannerPreflightReport.srv missing')
else:
    service_text = SERVICE.read_text(encoding='utf-8')
    for token in [
        'string request_profile',
        'string primary_backend',
        'string reject_reason',
        'string[] notes',
    ]:
        if token not in service_text:
            FAILURES.append(f'PlannerPreflightReport.srv missing field: {token}')

for token in [
    'handlePlannerPreflightReport',
    '/xmate_er3/internal/planner_preflight_report',
    '/xmate3/internal/planner_preflight_report',
]:
    if token not in (MANIFEST + FACADE + QUERY):
        FAILURES.append(f'planner preflight contract missing token: {token}')

if 'added internal `PlannerPreflightReport` service-level payload' not in BACKLOG:
    FAILURES.append('HARDENING_BACKLOG.md not updated to reflect planner preflight report landing')

if FAILURES:
    print('planner preflight contract check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('planner preflight contract check passed')
