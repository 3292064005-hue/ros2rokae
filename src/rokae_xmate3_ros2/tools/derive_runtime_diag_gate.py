#!/usr/bin/env python3
"""Derive a reusable runtime-diagnostics gate profile from one or more diagnostics logs.

The script reads diagnostics logs produced by `tools/run_main_chain_smoke.sh` and emits a
JSON limits file consumable by `tools/check_runtime_diag_gate.py` via `--limits-file`.

By default the script fails if any input sample reports `rt_deadline_miss=true`, because a
threshold profile should not be calibrated from already-invalid RT samples unless the caller
explicitly permits that behavior.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import re
import statistics
import sys
from typing import Iterable


FIELD_PATTERNS = {
    "rt_deadline_miss": re.compile(r"(?m)^\s*rt_deadline_miss:\s*(.+?)\s*$"),
    "rt_max_gap_ms": re.compile(r"(?m)^\s*rt_max_gap_ms:\s*(.+?)\s*$"),
    "rt_rx_latency_us": re.compile(r"(?m)^\s*rt_rx_latency_us:\s*(.+?)\s*$"),
    "rt_queue_depth": re.compile(r"(?m)^\s*rt_queue_depth:\s*(.+?)\s*$"),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Derive a runtime diagnostics gate JSON profile from diagnostics logs"
    )
    p.add_argument("log_paths", nargs="+", type=pathlib.Path, help="runtime diagnostics log files")
    p.add_argument(
        "--output",
        type=pathlib.Path,
        default=None,
        help="output JSON file path; prints to stdout when omitted",
    )
    p.add_argument(
        "--gap-scale",
        type=float,
        default=1.5,
        help="multiplier applied to the maximum observed rt_max_gap_ms (default: 1.5)",
    )
    p.add_argument(
        "--rx-latency-scale",
        type=float,
        default=2.0,
        help="multiplier applied to the maximum observed rt_rx_latency_us (default: 2.0)",
    )
    p.add_argument(
        "--queue-depth-headroom",
        type=int,
        default=2,
        help="extra queue depth headroom added above the maximum observed depth (default: 2)",
    )
    p.add_argument(
        "--min-gap-ms",
        type=float,
        default=10.0,
        help="minimum emitted max_gap_ms floor (default: 10.0)",
    )
    p.add_argument(
        "--min-rx-latency-us",
        type=float,
        default=5000.0,
        help="minimum emitted max_rx_latency_us floor (default: 5000.0)",
    )
    p.add_argument(
        "--min-queue-depth",
        type=int,
        default=1,
        help="minimum emitted max_queue_depth floor (default: 1)",
    )
    p.add_argument(
        "--allow-observed-deadline-miss",
        action="store_true",
        help="allow calibration to proceed even when a sample reports rt_deadline_miss=true",
    )
    return p.parse_args()


def extract_field(text: str, name: str) -> str:
    match = FIELD_PATTERNS[name].search(text)
    if match is None:
        raise ValueError(f"missing field {name}")
    return match.group(1).strip().strip('"')


def parse_bool(text: str) -> bool:
    lowered = text.lower()
    if lowered in {"true", "1"}:
        return True
    if lowered in {"false", "0"}:
        return False
    raise ValueError(f"invalid boolean: {text}")


def parse_float(text: str) -> float:
    return float(text)


def parse_int(text: str) -> int:
    return int(float(text))


def ceil_to_reasonable(value: float) -> float:
    if value <= 0:
        return 0.0
    if value >= 1000:
        return float(int(math.ceil(value / 100.0) * 100))
    if value >= 100:
        return float(int(math.ceil(value / 10.0) * 10))
    if value >= 10:
        return float(int(math.ceil(value)))
    return round(value, 3)


def load_sample(path: pathlib.Path) -> dict[str, float | bool | str]:
    if not path.is_file():
        raise FileNotFoundError(f"log file not found: {path}")
    text = path.read_text(encoding="utf-8", errors="replace")
    return {
        "path": str(path),
        "rt_deadline_miss": parse_bool(extract_field(text, "rt_deadline_miss")),
        "rt_max_gap_ms": parse_float(extract_field(text, "rt_max_gap_ms")),
        "rt_rx_latency_us": parse_float(extract_field(text, "rt_rx_latency_us")),
        "rt_queue_depth": parse_int(extract_field(text, "rt_queue_depth")),
    }


def main() -> int:
    args = parse_args()
    try:
        samples = [load_sample(path) for path in args.log_paths]
    except Exception as exc:
        print(f"derive_runtime_diag_gate: {exc}", file=sys.stderr)
        return 66

    observed_deadline_miss = [s for s in samples if bool(s["rt_deadline_miss"])]
    if observed_deadline_miss and not args.allow_observed_deadline_miss:
        sample_list = ", ".join(str(s["path"]) for s in observed_deadline_miss)
        print(
            "derive_runtime_diag_gate: refusing to calibrate from samples with rt_deadline_miss=true: "
            f"{sample_list}",
            file=sys.stderr,
        )
        return 1

    max_gap_ms = max(float(s["rt_max_gap_ms"]) for s in samples)
    max_rx_latency_us = max(float(s["rt_rx_latency_us"]) for s in samples)
    max_queue_depth = max(int(s["rt_queue_depth"]) for s in samples)

    derived = {
        "require_no_deadline_miss": not args.allow_observed_deadline_miss,
        "max_gap_ms": ceil_to_reasonable(max(args.min_gap_ms, max_gap_ms * args.gap_scale)),
        "max_rx_latency_us": ceil_to_reasonable(
            max(args.min_rx_latency_us, max_rx_latency_us * args.rx_latency_scale)
        ),
        "max_queue_depth": max(args.min_queue_depth, max_queue_depth + args.queue_depth_headroom),
        "metadata": {
            "sample_count": len(samples),
            "max_observed_gap_ms": max_gap_ms,
            "max_observed_rx_latency_us": max_rx_latency_us,
            "max_observed_queue_depth": max_queue_depth,
            "mean_observed_gap_ms": statistics.fmean(float(s["rt_max_gap_ms"]) for s in samples),
            "mean_observed_rx_latency_us": statistics.fmean(float(s["rt_rx_latency_us"]) for s in samples),
            "mean_observed_queue_depth": statistics.fmean(int(s["rt_queue_depth"]) for s in samples),
            "sources": [str(s["path"]) for s in samples],
            "gap_scale": args.gap_scale,
            "rx_latency_scale": args.rx_latency_scale,
            "queue_depth_headroom": args.queue_depth_headroom,
        },
    }

    payload = json.dumps(derived, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        sys.stdout.write(payload)
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
        print(f"derive_runtime_diag_gate: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
