# RT Hardening Profile

This document records the current runtime-side RT hardening baseline.
It does **not** claim controller-grade parity or hardware RT equivalence.

## Implemented in Pass 5

- single runtime-owned authoritative servo profile resolver (`rt_runtime_profile`)
- shared RT scheduler contract helper (`rt_scheduler`) for daemon and Gazebo-hosted runtime
- Gazebo-hosted runtime reports scheduler state as `host_managed(gazebo_update_thread)` instead of pretending the plugin can retune the world-update thread at runtime
- Gazebo-hosted observability is published from ROS executor timers instead of the `OnUpdate()` control path
- decoupled observability cadence: joint-state / operation-state / diagnostics now publish independently from the servo tick
- hard-profile transport contract: `hard_1khz` rejects non-SHM RT ingress and fails fast when the scheduler contract degrades
- Gazebo-hosted runtime no longer claims hard 1 kHz support; `hard_1khz` is daemon-only by contract
- runtime-owned direct RT command channel for joint/cartesian/torque surfaces
- hot-path RT snapshot consumption: `RuntimeControlBridge` now reads pre-parsed `DataStoreState::RtControlSnapshot` values instead of reparsing strings inside the servo tick
- RT semantic diagnostics: `last_api_surface`, `last_result_source`, `rt_dispatch_mode`, `catalog_provenance_summary`
- strict RT state reads: rejected subscription downgrades, 1 ms in-loop subscriptions limited to controller-native/local RT fields, and no silent NRT fallback during `updateRobotState()`
- native SDK lifecycle/mode entrypoints (`connectToRobot`, `disconnectFromRobot`, `setPowerState`, `setOperateMode`, `setMotionControlMode`) now short-circuit as idempotent success when the aggregated runtime snapshot already matches the requested state
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
- SDK wrapper RT state stream now splits field families explicitly:
  - strict 1 ms in-loop fields: joint position / velocity / torque, joint acceleration, torque derivative, motor mirrored signals
  - polled-only simulation-grade fields: TCP pose matrix / abc pose, TCP velocity / acceleration, external torque estimates
  - xMate3 六轴 shim no longer advertises `psi_*` / `elbow*` as RT fields at all

## What remains intentionally simulation-grade

- direct RT commands are still simulation-side servo approximations, not controller-native UDP servo traffic
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

## Field policy source

RT field admissibility is now summarized from `rt_field_registry` and surfaced as `policy_source=rt_field_registry; policies=...` in diagnostics summaries. This makes strict-in-loop vs polled-only behavior an executable contract instead of a purely descriptive note.

## Current runtime profile contract

- `nrt_strict_parity`: queued NRT public lane, non-authoritative servo timing, slower observability cadence
- `rt_sim_experimental_best_effort`: authoritative 1 ms runtime tick, decoupled observability, topic+SHM RT ingress allowed
- `rt_hardened`: authoritative 1 ms runtime tick, decoupled observability, legacy custom-data RT fallback disabled
- `hard_1khz`: daemon-only strict profile, authoritative 1 ms runtime tick, SHM-only ingress, strict scheduler contract

`hard_1khz` still does **not** imply controller-native UDP or safety-certified hard realtime. It means the project now exposes an explicit fail-fast profile whose runtime contract is stricter than the default simulation-grade lane.

- Release-gate now runs `tools/run_main_chain_smoke.sh`, which applies threshold checks over `rt_deadline_miss`, `rt_max_gap_ms`, `rt_rx_latency_us`, and `rt_queue_depth` via `tools/check_runtime_diag_gate.py`.
- Threshold resolution is now externalized: `tools/run_main_chain_smoke.sh` prefers `config/runtime_diag_gate.default.json`, accepts `ROKAE_RT_GATE_LIMITS_FILE` for target-specific calibration, and still allows one-off env-var overrides.
- `tools/derive_runtime_diag_gate.py` converts successful runtime diagnostics logs into a reusable JSON gate profile so target-machine baselines can be versioned instead of copied as ad-hoc shell env vars.
