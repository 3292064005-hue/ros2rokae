#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ORACLE = json.loads((ROOT / 'docs' / 'reference' / 'official_cpp_sdk_oracle.json').read_text(encoding='utf-8'))
FAILURES: list[str] = []


def normalize_cpp(text: str) -> str:
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    text = re.sub(r'//.*', ' ', text)
    text = re.sub(r'\s+', ' ', text)
    return text.strip()


def require_normalized_token(rel: str, text: str, token: str, *, label: str) -> None:
    if normalize_cpp(token) not in normalize_cpp(text):
        FAILURES.append(f'{label} {rel} missing SDK oracle signature pattern: {token}')


tiers = ORACLE.get('verification_tiers', {})
for key, expected in {
    'token_presence': 'enforced_by_check_official_cpp_sdk_oracle_py',
    'local_header_signature_tokens': 'enforced_for_xmate_er3_public_subset',
    'official_header_presence': 'enforced_only_when_ROKAE_OFFICIAL_SDK_INCLUDE_is_set',
    'full_signature_parity': 'not_claimed',
    'abi_parity': 'not_claimed',
    'hardware_behavior_parity': 'not_claimed',
}.items():
    if tiers.get(key) != expected:
        FAILURES.append(f'official SDK oracle verification_tiers.{key} must be {expected}')

if 'not full official SDK ABI/API/behavior parity' not in ORACLE.get('allowed_claim', ''):
    FAILURES.append('official SDK oracle must explicitly limit the compatibility claim')
for forbidden in ['full official SDK parity', 'ABI parity', 'hardware behavior parity']:
    if forbidden not in ORACLE.get('forbidden_claims', []):
        FAILURES.append(f'official SDK oracle missing forbidden claim guard: {forbidden}')

for rel in ORACLE.get('required_headers', []):
    if not (ROOT / rel).is_file():
        FAILURES.append(f'missing required compatibility header: {rel}')

for rel, tokens in ORACLE.get('local_source_compatibility_signatures', {}).items():
    path = ROOT / rel
    if not path.is_file():
        FAILURES.append(f'missing local SDK compatibility header: {rel}')
        continue
    text = path.read_text(encoding='utf-8')
    for token in tokens:
        require_normalized_token(rel, text, token, label='local')

if 'local_source_compatibility_signatures' not in ORACLE:
    FAILURES.append('official SDK oracle must contain local_source_compatibility_signatures')
if 'official_signature_patterns' not in ORACLE:
    FAILURES.append('official SDK oracle must contain official_signature_patterns')

official_root = os.environ.get('ROKAE_OFFICIAL_SDK_INCLUDE', '').strip()
if official_root:
    official = Path(official_root)
    for header_name, patterns in ORACLE.get('official_signature_patterns', {}).items():
        candidate = official / 'rokae' / header_name
        if not candidate.is_file():
            FAILURES.append(f'official SDK include root missing rokae/{header_name}')
            continue
        text = candidate.read_text(encoding='utf-8', errors='ignore')
        for pattern in patterns:
            require_normalized_token(f'rokae/{header_name}', text, pattern, label='official')

alignment_doc = (ROOT / 'docs' / 'reference' / 'SDK_ALIGNMENT.md').read_text(encoding='utf-8')
if 'token/source-scope oracle' not in alignment_doc:
    FAILURES.append('SDK_ALIGNMENT.md must describe the oracle as token/source-scope, not full parity')
if 'not full official SDK ABI/API/behavior parity' not in alignment_doc:
    FAILURES.append('SDK_ALIGNMENT.md must state that full official SDK parity is not claimed')
if 'ROKAE_OFFICIAL_SDK_INCLUDE' not in alignment_doc:
    FAILURES.append('SDK_ALIGNMENT.md must document the optional official-header oracle mode')

if 'MoveSP' not in ORACLE.get('experimental_modules', []):
    FAILURES.append('oracle must classify MoveSP as experimental')

if FAILURES:
    print('official C++ SDK token/source-scope oracle check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

if official_root:
    print('official C++ SDK token/source-scope oracle check passed with official header pattern validation')
else:
    print('official C++ SDK token/source-scope oracle check passed; official header pattern validation not run because ROKAE_OFFICIAL_SDK_INCLUDE is unset')
