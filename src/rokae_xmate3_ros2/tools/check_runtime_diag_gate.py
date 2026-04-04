#!/usr/bin/env python3
import argparse
import json
import pathlib
import re
import sys
from typing import Any


FIELD_MAP = {
    "require_no_deadline_miss": "rt_deadline_miss",
    "max_gap_ms": "rt_max_gap_ms",
    "max_rx_latency_us": "rt_rx_latency_us",
    "max_queue_depth": "rt_queue_depth",
}


def parse_args():
    p = argparse.ArgumentParser(description="Validate runtime diagnostics against threshold gates")
    p.add_argument("log_path", type=pathlib.Path)
    p.add_argument("--limits-file", type=pathlib.Path, default=None,
                   help="JSON limits file with keys: require_no_deadline_miss, max_gap_ms, max_rx_latency_us, max_queue_depth")
    p.add_argument("--require-no-deadline-miss", action="store_true")
    p.add_argument("--max-gap-ms", type=float, default=None)
    p.add_argument("--max-rx-latency-us", type=float, default=None)
    p.add_argument("--max-queue-depth", type=int, default=None)
    p.add_argument("--print-effective-limits", action="store_true",
                   help="Print the merged limits after file + CLI override resolution")
    return p.parse_args()


def extract_field(text: str, name: str):
    m = re.search(rf"(?m)^\s*{re.escape(name)}:\s*(.+?)\s*$", text)
    return None if m is None else m.group(1).strip()


def parse_bool(text: str):
    lowered = text.lower()
    if lowered in {"true", "1"}:
        return True
    if lowered in {"false", "0"}:
        return False
    raise ValueError(f"invalid boolean: {text}")


def parse_float(text: str):
    cleaned = text.strip().strip('"')
    return float(cleaned)


def parse_int(text: str):
    cleaned = text.strip().strip('"')
    return int(float(cleaned))


def load_limits_file(path: pathlib.Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"limits file not found: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("limits file must contain a JSON object")
    unknown_keys = set(payload.keys()) - set(FIELD_MAP.keys()) - {"metadata"}
    if unknown_keys:
        raise ValueError(f"limits file contains unsupported keys: {sorted(unknown_keys)}")
    return payload


def resolve_limits(args) -> dict[str, Any]:
    merged: dict[str, Any] = {
        "require_no_deadline_miss": False,
        "max_gap_ms": None,
        "max_rx_latency_us": None,
        "max_queue_depth": None,
    }
    if args.limits_file is not None:
        merged.update(load_limits_file(args.limits_file))
    if args.require_no_deadline_miss:
        merged["require_no_deadline_miss"] = True
    if args.max_gap_ms is not None:
        merged["max_gap_ms"] = args.max_gap_ms
    if args.max_rx_latency_us is not None:
        merged["max_rx_latency_us"] = args.max_rx_latency_us
    if args.max_queue_depth is not None:
        merged["max_queue_depth"] = args.max_queue_depth
    return merged


def main():
    args = parse_args()
    if not args.log_path.is_file():
        print(f"runtime_diag_gate: log file not found: {args.log_path}", file=sys.stderr)
        return 66
    text = args.log_path.read_text(encoding="utf-8", errors="replace")

    try:
        limits = resolve_limits(args)
    except Exception as exc:
        print(f"runtime_diag_gate: failed to resolve limits: {exc}", file=sys.stderr)
        return 66

    if args.print_effective_limits:
        printable = {k: v for k, v in limits.items() if k != "metadata"}
        print("runtime_diag_gate: effective_limits=" + json.dumps(printable, sort_keys=True))

    failures = []
    if limits.get("require_no_deadline_miss", False):
        raw = extract_field(text, "rt_deadline_miss")
        if raw is None:
            failures.append("missing rt_deadline_miss")
        else:
            try:
                if parse_bool(raw):
                    failures.append(f"rt_deadline_miss must be false, got {raw}")
            except ValueError as exc:
                failures.append(str(exc))

    numeric_checks = [
        ("rt_max_gap_ms", limits.get("max_gap_ms"), parse_float, lambda value, limit: value <= limit),
        ("rt_rx_latency_us", limits.get("max_rx_latency_us"), parse_float, lambda value, limit: value <= limit),
        ("rt_queue_depth", limits.get("max_queue_depth"), parse_int, lambda value, limit: value <= limit),
    ]
    for field, limit, parser, predicate in numeric_checks:
        if limit is None:
            continue
        raw = extract_field(text, field)
        if raw is None:
            failures.append(f"missing {field}")
            continue
        try:
            value = parser(raw)
        except Exception as exc:
            failures.append(f"invalid {field}: {exc}")
            continue
        if not predicate(value, limit):
            failures.append(f"{field}={value} exceeds limit {limit}")

    if failures:
        for failure in failures:
            print(f"runtime_diag_gate: {failure}", file=sys.stderr)
        return 1

    print("runtime_diag_gate: diagnostics thresholds satisfied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
