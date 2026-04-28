#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
failures: list[str] = []

def read(rel: str) -> str:
    p = ROOT / rel
    if not p.is_file():
        failures.append(f'missing required file: {rel}')
        return ''
    return p.read_text(encoding='utf-8')

hpp = read('src/runtime/backend_provider.hpp')
cpp = read('src/runtime/backend_provider.cpp')
bootstrap = read('src/gazebo/runtime_bootstrap.cpp')
sim_main = read('src/runtime/sim_runtime_main.cpp')
arch = read('docs/architecture/ARCHITECTURE.md')
provider_doc = read('docs/architecture/PROVIDER_BOUNDARY.md')
manifest = read('docs/reference/xmate_er3_alignment_manifest.json')
for legacy in ['RuntimeBackendHostFlavor','createSimulationBackend(','createHeadlessMockBackend(','host.hostFlavor()']:
    if legacy in hpp or legacy in cpp:
        failures.append(f'provider boundary still exposes legacy symbol: {legacy}')
for token in ['struct RuntimeBackendFactoryRequest','supportsFactory(const std::string &factory_key)','advertisedFactoryKeys() const','createBackend(','RuntimeBackendFactoryRequest &request']:
    if token not in hpp:
        failures.append(f'backend_provider.hpp missing token: {token}')
if 'factory_key == "gazebo_runtime"' not in bootstrap:
    failures.append('runtime_bootstrap.cpp missing generic gazebo_runtime factory handling')
if 'factory_key == "headless_mock"' not in sim_main:
    failures.append('sim_runtime_main.cpp missing generic headless_mock factory handling')
if 'RuntimeBackendProviderHost -> RuntimeBackendProvider -> BackendInterface' not in provider_doc:
    failures.append('PROVIDER_BOUNDARY.md missing provider boundary chain')
if 'generic backend factory request' not in provider_doc:
    failures.append('PROVIDER_BOUNDARY.md missing generic backend factory request wording')
if 'PROVIDER_BOUNDARY.md' not in arch:
    failures.append('ARCHITECTURE.md must link to PROVIDER_BOUNDARY.md')
if 'RuntimeBackendFactoryRequest' not in manifest:
    failures.append('alignment manifest must pin RuntimeBackendFactoryRequest')
if failures:
    print('provider boundary check failed:')
    for f in failures:
        print(f'- {f}')
    sys.exit(1)
print('provider boundary check passed')
