#!/usr/bin/env python3
"""Static contract checks for the opt-in hard_1khz stress gate."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = PACKAGE_ROOT / "tools" / "run_rt_1khz_stress.sh"


def main() -> int:
    text = SCRIPT.read_text(encoding="utf-8")
    required_snippets = {
        'DURATION_SEC="${2:-60}"': "default development duration must be 60 seconds",
        "-p runtime_profile:=hard_1khz": "daemon launch must select hard_1khz",
        "-p service_exposure_profile:=internal_full": "daemon launch must use internal_full exposure",
        "-p compatibility_alias_policy:=canonical_plus_compat": "daemon launch must use compat aliases",
        "-p rt_scheduler.priority:=90": "daemon launch must request priority 90",
        "-p rt_scheduler.cpu_affinity:=": "daemon runtime must be pinned for the 1kHz stress gate",
        "-p rt_memory.lock_all:=true": "daemon launch must request memlock",
        "chrt -f 80": "stress client must run below daemon RT priority when root is available",
        "taskset -c": "stress client must be CPU-pinned for the 1kHz stress gate",
        "ROKAE_RT_TRANSPORT_MODE must be shm_only": "non-shm RT transport must fail fast",
        'export ROKAE_RT_TRANSPORT_MODE="shm_only"': "stress gate must force shm_only",
        "user rtprio/memlock limits allow strict RT scheduling": "user RT limits must be accepted without file caps",
        "runtime_executable_on_nosuid_mount": "nosuid mounts must be classified as host readiness failures",
        'stress_mode == "daemon"': "only daemon mode may pass the 1kHz gate",
        'rt_scheduler_state.lower() == "active"': "scheduler active state must be a pass condition",
        'rt_transport_source == "shm_ring"': "shm_ring ingress must be a pass condition",
        "RT_DIAGNOSTICS_JSON": "stress gate must consume diagnostics captured by the stress client",
        "rt_deadline_miss == 0.0": "deadline misses must be a pass failure",
        "rt_max_gap_ms <= 1.2": "max RT gap must be bounded",
        'classification = "non_strict_diagnostic"': "simulation mode must be diagnostic-only",
    }

    failures = [
        f"{description}: missing `{snippet}`"
        for snippet, description in required_snippets.items()
        if snippet not in text
    ]
    if failures:
        print("check_rt_1khz_stress_contract: FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("check_rt_1khz_stress_contract: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
