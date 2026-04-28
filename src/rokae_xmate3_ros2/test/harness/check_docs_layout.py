#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []
required = [
    'docs/INDEX.md','docs/public/QUICKSTART.md','docs/public/COMPATIBILITY.md','docs/public/EXAMPLES.md','docs/public/PUBLIC_SDK_ARTIFACT.md','docs/public/KINEMATICS_AND_MODEL.md','docs/public/RUNTIME_PROFILES.md',
    'docs/architecture/ARCHITECTURE.md','docs/architecture/PROVIDER_BOUNDARY.md',
    'docs/release/BUILD_RELEASE.md','docs/release/ENVIRONMENT_LOCK.md','docs/release/RELEASE_GATE.md','docs/release/ACCEPTANCE_LAYERS.md','docs/release/acceptance_layers_manifest.json','docs/release/HARDENING_BACKLOG.md',
    'docs/reference/SDK_ALIGNMENT.md','docs/reference/xmate_er3_alignment_manifest.json','docs/reference/RUNTIME_STATE_MACHINE.md','docs/reference/runtime_state_machine_manifest.json','docs/reference/RECORDED_PATH_SCHEMA.md','docs/reference/recorded_path_schema_manifest.json','docs/archive/audits/IMPLEMENTATION_AUDIT.md','examples/README.md']
for rel in required:
    if not (ROOT/rel).is_file():
        FAILURES.append(f'missing required docs/layout file: {rel}')
removed = [
    'docs/COMPAT_ABI.md','docs/API_ALIGNMENT_MATRIX.md','docs/XMATE_ER3_OFFICIAL_ALIGNMENT_MATRIX.md','docs/RT_PROFILE_GUIDE.md','docs/RT_HARDENING_PROFILE.md','docs/PROFILE_CAPABILITY_MATRIX.md','docs/PROFILE_QUERY_POLICY.md','docs/RUNTIME_CATALOG_POLICY.md','docs/KINEMATICS_POLICY.md','docs/FIDELITY_POLICY.md','docs/MODEL_TRACEABILITY.md','docs/EXTENSION_FRAMEWORK.md','docs/P0_1_DEEP_REVIEW_REPORT.md','docs/P0_1_IMPLEMENTATION_SUMMARY.md','docs/P0_1_ISSUE_REMEDIATION_SUMMARY.md','docs/P0_1_SECOND_REFINEMENT_SUMMARY.md','docs/HARDENING_BACKLOG.md','docs/IMPLEMENTATION_AUDIT.md','docs/maintenance/HARDENING_BACKLOG.md','examples/PUBLIC_SDK_README.md']
for rel in removed:
    if (ROOT/rel).exists():
        FAILURES.append(f'legacy/duplicate doc should be removed: {rel}')
readme = (ROOT/'README.md').read_text(encoding='utf-8')
index = (ROOT/'docs'/'INDEX.md').read_text(encoding='utf-8')
for token in ['docs/public/QUICKSTART.md','docs/architecture/PROVIDER_BOUNDARY.md','docs/release/BUILD_RELEASE.md','docs/release/ACCEPTANCE_LAYERS.md','docs/reference/SDK_ALIGNMENT.md']:
    if token not in readme:
        FAILURES.append(f'README.md missing entry: {token}')
for token in ['public/','architecture/','release/','reference/','archive/']:
    if token not in index:
        FAILURES.append(f'docs/INDEX.md missing group token: {token}')

TARGET_DOCS = [ROOT/'README.md', ROOT/'docs'/'INDEX.md']
TARGET_DOCS.extend(sorted((ROOT/'docs'/'public').glob('*.md')))

LINK_RE = re.compile(r'(?<!!)\[[^\]]+\]\(([^)]+)\)')
CODE_FENCE_RE = re.compile(r'```.*?```', re.DOTALL)
HTML_LINK_RE = re.compile(r'<a\s+[^>]*href=["\']([^"\']+)["\']', re.IGNORECASE)


def strip_code_fences(text: str) -> str:
    return CODE_FENCE_RE.sub('', text)


def normalize_target(raw_target: str) -> str:
    target = raw_target.strip()
    if target.startswith('<') and target.endswith('>'):
        target = target[1:-1].strip()
    return target


def should_skip(target: str) -> bool:
    lowered = target.lower()
    return (
        not target
        or target.startswith('#')
        or lowered.startswith('http://')
        or lowered.startswith('https://')
        or lowered.startswith('mailto:')
        or lowered.startswith('file://')
        or lowered.startswith('data:')
    )


def check_target(doc: Path, target: str) -> None:
    target = normalize_target(target)
    if should_skip(target):
        return
    path_part = target.split('#', 1)[0].split('?', 1)[0]
    if not path_part:
        return
    if path_part.startswith('/'):
        resolved = ROOT / path_part.lstrip('/')
    else:
        resolved = (doc.parent / path_part).resolve()
    if not resolved.exists():
        try:
            rel = resolved.relative_to(ROOT)
            shown = str(rel)
        except ValueError:
            shown = str(resolved)
        FAILURES.append(f'broken relative link in {doc.relative_to(ROOT)} -> {target} (resolved: {shown})')

for doc in TARGET_DOCS:
    raw = doc.read_text(encoding='utf-8')
    text = strip_code_fences(raw)
    for match in LINK_RE.finditer(text):
        check_target(doc, match.group(1))
    for match in HTML_LINK_RE.finditer(text):
        check_target(doc, match.group(1))

if FAILURES:
    print('docs layout check failed:')
    for f in FAILURES:
        print(f'- {f}')
    sys.exit(1)
print('docs layout check passed')
