#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
PYTHON = sys.executable
SAMPLE_LOG = ROOT / "test" / "harness" / "data" / "runtime_diagnostics_sample.log"
DEFAULT_LIMITS = ROOT / "config" / "runtime_diag_gate.default.json"
CHECK_TOOL = ROOT / "tools" / "check_runtime_diag_gate.py"
DERIVE_TOOL = ROOT / "tools" / "derive_runtime_diag_gate.py"


def run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, capture_output=True, check=False)


def main() -> int:
    check_proc = run(
        PYTHON,
        str(CHECK_TOOL),
        str(SAMPLE_LOG),
        "--limits-file",
        str(DEFAULT_LIMITS),
        "--print-effective-limits",
    )
    if check_proc.returncode != 0:
        print(check_proc.stdout)
        print(check_proc.stderr, file=sys.stderr)
        return check_proc.returncode
    if "effective_limits" not in check_proc.stdout:
        print("check_runtime_diag_gate did not print effective limits", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="rokae_rt_gate_") as temp_dir:
        output_path = pathlib.Path(temp_dir) / "derived_limits.json"
        derive_proc = run(
            PYTHON,
            str(DERIVE_TOOL),
            str(SAMPLE_LOG),
            "--output",
            str(output_path),
        )
        if derive_proc.returncode != 0:
            print(derive_proc.stdout)
            print(derive_proc.stderr, file=sys.stderr)
            return derive_proc.returncode
        payload = json.loads(output_path.read_text(encoding="utf-8"))
        for key in ("require_no_deadline_miss", "max_gap_ms", "max_rx_latency_us", "max_queue_depth", "metadata"):
            if key not in payload:
                print(f"derived limits missing key: {key}", file=sys.stderr)
                return 1
        if payload["max_gap_ms"] < 12.5:
            print("derived max_gap_ms is below observed sample", file=sys.stderr)
            return 1
        if payload["max_queue_depth"] < 1:
            print("derived max_queue_depth is below observed sample", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
