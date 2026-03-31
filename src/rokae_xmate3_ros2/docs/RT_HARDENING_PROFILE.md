# RT Hardening Profile

This document records the current simulation-grade RT hardening baseline.
It does **not** claim controller-grade parity.

## Implemented in Pass 5

- RT field registry (`rt_field_registry`)
- RT subscription plan builder (`rt_subscription_plan`)
- RT watchdog (`rt_watchdog`)
- RT pre-arm checks (`rt_prearm_checks`)
- Runtime diagnostics fields:
  - `rt_subscription_plan`
  - `rt_prearm_status`
  - `rt_watchdog_summary`
  - `rt_late_cycle_count`
  - `rt_max_gap_ms`
- SDK wrapper RT state stream now accepts a wider field set and derives:
  - joint acceleration
  - torque derivative
  - TCP pose matrix / abc pose
  - TCP velocity / acceleration
  - motor mirrored signals
  - external torque estimates

## What remains intentionally simulation-grade

- no controller-native UDP state transport
- no hard real-time scheduler guarantee
- no controller-grade watchdog semantics
- no controller-side field bandwidth enforcement beyond local budget checks

## Current default RT subscription plan

The runtime bridge defaults to:

- `q_m`
- `dq_m`
- `tau_m`

with `use_state_data_in_loop=true` and a 1 ms expectation.

## Release rule

Do not upgrade RT from `Experimental` / `SimApprox` unless:

1. request-level RT command semantics are tested,
2. diagnostics fields are stable,
3. watchdog/prearm semantics are test-backed,
4. docs stop using simulation-grade caveats.
