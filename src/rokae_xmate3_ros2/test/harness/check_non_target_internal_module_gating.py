#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CMAKE = (ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
TARGETS_RUNTIME = (ROOT / 'cmake' / 'targets_runtime.cmake').read_text(encoding='utf-8')
CONTROL = (ROOT / 'src' / 'runtime' / 'control_facade.cpp').read_text(encoding='utf-8')
QUERY = (ROOT / 'src' / 'runtime' / 'query_state_service.cpp').read_text(encoding='utf-8')
CLIENTS = (ROOT / 'src' / 'sdk' / 'robot_clients.cpp').read_text(encoding='utf-8')
MANIFEST = (ROOT / 'src' / 'runtime' / 'service_contract_manifest.hpp').read_text(encoding='utf-8') + (ROOT / 'src' / 'runtime' / 'service_contract_manifest.cpp').read_text(encoding='utf-8')
FAILURES: list[str] = []

if 'option(ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES' not in CMAKE:
    FAILURES.append('CMakeLists.txt must define ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES gate')
if 'option(ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES "Build RL/IO/register/avoid-singularity internal modules' not in CMAKE:
    FAILURES.append('non-target module gate must describe RL/IO/register/avoid-singularity scope')
if 'option(ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES "Build RL/IO/register/avoid-singularity internal modules that are intentionally outside the xMateER3 public lane" OFF)' not in CMAKE:
    FAILURES.append('non-target internal module gate must default OFF')

for token in [
    '#if ROKAE_ENABLE_INTERNAL_SURFACE && ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES',
    'handleSetSimulationMode',
    'handleSetAvoidSingularity',
]:
    if token not in CONTROL:
        FAILURES.append(f'control_facade.cpp missing gated token: {token}')

for token in [
    '#if ROKAE_ENABLE_INTERNAL_SURFACE && ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES',
    'handleGetAvoidSingularity',
]:
    if token not in QUERY:
        FAILURES.append(f'query_state_service.cpp missing gated token: {token}')

for token in [
    '#if !ROKAE_ENABLE_INTERNAL_SURFACE || !ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES',
    'publishCatalogProvenance',
    'ensureProjectClients',
]:
    if token not in CLIENTS:
        FAILURES.append(f'robot_clients.cpp missing non-target gating token: {token}')

for token in [
    'query_catalog_service.cpp',
    'io_program_facade.cpp',
    'ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES',
]:
    if token not in TARGETS_RUNTIME:
        FAILURES.append(f'targets_runtime.cmake missing non-target gating token: {token}')

if 'buildInternalPrimaryServiceContractManifest' not in MANIFEST or 'PlannerPreflightReport' not in MANIFEST:
    FAILURES.append('service manifest must preserve internal preflight contracts')
if 'GetRlProjectInfo' not in MANIFEST or '#if ROKAE_ENABLE_INTERNAL_SURFACE && ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES' not in MANIFEST:
    FAILURES.append('service manifest must gate RL/IO/register compatibility branches behind non-target module flag')

if FAILURES:
    print('non-target internal module gating check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('non-target internal module gating check passed')
