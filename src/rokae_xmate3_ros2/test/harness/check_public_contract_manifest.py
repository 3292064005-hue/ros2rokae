#!/usr/bin/env python3
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []
manifest = json.loads((ROOT / 'docs' / 'reference' / 'xmate_er3_alignment_manifest.json').read_text(encoding='utf-8'))
public_examples = manifest.get('public_examples', [])
internal_examples = manifest.get('internal_examples', [])
identity = manifest.get('identity', {})
cmake_text = (ROOT / 'cmake' / 'targets_examples.cmake').read_text(encoding='utf-8')
examples_text = (ROOT / 'docs' / 'public' / 'EXAMPLES.md').read_text(encoding='utf-8')
compat_text = (ROOT / 'docs' / 'public' / 'COMPATIBILITY.md').read_text(encoding='utf-8')
config_text = (ROOT / 'cmake' / 'xCoreSDKConfig.cmake.in').read_text(encoding='utf-8')
readme_text = (ROOT / 'README.md').read_text(encoding='utf-8')
packaging_text = (ROOT / 'cmake' / 'targets_packaging.cmake').read_text(encoding='utf-8')

def parse_cmake_list(name: str) -> list[str]:
    m = re.search(rf"set\({name}\n(.*?)\n\)", cmake_text, re.S)
    if not m:
        FAILURES.append(f'missing CMake list: {name}')
        return []
    return [line.strip() for line in m.group(1).splitlines() if line.strip() and not line.strip().startswith('#')]

def example_bullets(kind: str) -> list[str]:
    pattern = r"## 2\. Public examples\n(.*?)\n## 3\. Internal/backend examples" if kind == 'public' else r"## 3\. Internal/backend examples\n(.*?)\n## 4\. Run"
    m = re.search(pattern, examples_text, re.S)
    if not m:
        FAILURES.append(f'failed to locate {kind} examples section')
        return []
    return [line.strip().removeprefix('- `example_').removesuffix('`') for line in m.group(1).splitlines() if line.strip().startswith('- `example_')]

if parse_cmake_list('ROKAE_PUBLIC_COMPAT_EXAMPLES') != public_examples:
    FAILURES.append('public examples drift between cmake and manifest')
if parse_cmake_list('ROKAE_INTERNAL_BACKEND_EXAMPLES') != internal_examples:
    FAILURES.append('internal examples drift between cmake and manifest')
if example_bullets('public') != public_examples:
    FAILURES.append('public examples drift between docs and manifest')
if example_bullets('internal') != internal_examples:
    FAILURES.append('internal examples drift between docs and manifest')

for token in ['MoveSP', '路径录制/回放', 'public xMateER3 lane']:
    if token not in compat_text:
        FAILURES.append(f'COMPATIBILITY.md missing token: {token}')
for required in ['xCoreSDK_CANONICAL_PACKAGE', 'xCoreSDK_LEGACY_SOURCE_PACKAGE', 'xCoreSDK_CANONICAL_IDENTITY']:
    if required not in config_text:
        FAILURES.append(f'xCoreSDKConfig.cmake.in missing {required}')
if 'canonical install-facing identity: `xCoreSDK`' not in readme_text:
    FAILURES.append('README.md missing canonical install-facing identity')
if identity.get('canonical_package') != 'xCoreSDK' or identity.get('legacy_source_package') != 'rokae_xmate3_ros2' or identity.get('canonical_identity') != 'xCoreSDK:xmate_er3':
    FAILURES.append('manifest identity block drift')
layout = manifest.get('source_layout', {})
expected_layout = {'public_rosidl_root':'srv/','internal_rosidl_root':'internal_interfaces/srv/','public_examples_root':'examples/cpp/','internal_examples_root':'examples/internal/cpp/'}
for k,v in expected_layout.items():
    if layout.get(k) != v:
        FAILURES.append(f'manifest source_layout.{k} must be {v}')
for required in ['GetDI','ReadRegister','GetRlProjectInfo','SetXPanelVout']:
    if (ROOT/'srv'/f'{required}.srv').exists():
        FAILURES.append(f'internal service leaked into public srv/: {required}.srv')
    if not (ROOT/'internal_interfaces'/'srv'/f'{required}.srv').is_file():
        FAILURES.append(f'internal service missing from internal_interfaces/srv/: {required}.srv')
if 'rosidl_generator_cpp/${PROJECT_NAME}' in packaging_text:
    FAILURES.append('targets_packaging.cmake must not mirror generated rosidl headers into the public SDK include tree')
for token in ['docs/public/PUBLIC_SDK_ARTIFACT.md','examples/README.md']:
    if token not in packaging_text:
        FAILURES.append(f'targets_packaging.cmake missing install reference: {token}')
if FAILURES:
    print('public contract manifest check failed:')
    for f in FAILURES:
        print(f'- {f}')
    sys.exit(1)
print('public contract manifest check passed')
