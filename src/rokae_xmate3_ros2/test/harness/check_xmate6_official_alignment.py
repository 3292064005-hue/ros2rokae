#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
failures: list[str] = []


def require_contains(path: Path, needle: str, label: str) -> None:
    text = path.read_text(encoding="utf-8")
    if needle not in text:
        failures.append(f"{label}: missing '{needle}' in {path.relative_to(ROOT)}")


manifest_path = ROOT / "docs" / "xmate6_official_alignment_manifest.json"
if not manifest_path.is_file():
    failures.append(f"missing alignment manifest: {manifest_path}")
else:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("target_family") != "xmate6":
        failures.append("manifest target_family must be xmate6")
    if manifest.get("alignment_scope") != "call_build_only":
        failures.append("manifest alignment_scope must be call_build_only")
    if manifest.get("contract_source") != "single_source_manifest":
        failures.append("manifest contract_source must be single_source_manifest")

    unsupported = manifest.get("unsupported_public_modules", [])
    if unsupported != ["io", "rl", "calibration"]:
        failures.append("manifest unsupported_public_modules must be exactly ['io', 'rl', 'calibration']")

    unsupported_contract = manifest.get("unsupported_contract", {})
    if unsupported_contract.get("runtime_policy") != "deterministic_not_implemented":
        failures.append("manifest unsupported_contract.runtime_policy must be deterministic_not_implemented")
    if unsupported_contract.get("error_code_symbol") != "SdkError::not_implemented":
        failures.append("manifest unsupported_contract.error_code_symbol must be SdkError::not_implemented")

    profiles = manifest.get("profiles", {})
    if profiles.get("nrt") != "nrt_strict_parity":
        failures.append("manifest profiles.nrt must be nrt_strict_parity")
    if profiles.get("rt") != "hard_1khz":
        failures.append("manifest profiles.rt must be hard_1khz")
    if profiles.get("rt_policy") != "strict_1khz_fail_fast":
        failures.append("manifest profiles.rt_policy must be strict_1khz_fail_fast")

    checks = manifest.get("checks", {})
    for key in ("header_patterns", "compat_patterns", "runtime_patterns", "docs_patterns", "cmake_patterns"):
        entries = checks.get(key, [])
        if not isinstance(entries, list) or not entries:
            failures.append(f"manifest checks.{key} must be a non-empty list")
            continue
        for entry in entries:
            rel_file = entry.get("file", "")
            contains = entry.get("contains", "")
            if not rel_file or not contains:
                failures.append(f"manifest checks.{key} entries require file+contains")
                continue
            path = ROOT / rel_file
            if not path.is_file():
                failures.append(f"manifest checks.{key}: missing file {rel_file}")
                continue
            require_contains(path, contains, f"manifest checks.{key}")

if failures:
    print("xmate6 official alignment check failed:")
    for item in failures:
        print(f"- {item}")
    sys.exit(1)

print("xmate6 official alignment check passed")
