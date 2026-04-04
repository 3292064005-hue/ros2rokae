# Profile capability matrix

This document hardens the runtime profile model introduced after Pass 5.

## Profiles

- `nrt_strict_parity`: public xMate6 queued non-realtime strict-parity lane
- `rt_sim_experimental_best_effort`: simulation-grade RT facade with authoritative runtime servo ownership and decoupled observability
- `rt_hardened`: authoritative runtime servo profile with legacy RT custom-data fallback disabled
- `hard_1khz`: daemon-only strict RT contract with SHM-only ingress and fail-fast scheduler requirements
- `hybrid_bridge`: trajectory + effort arbitration profile
- `effort_direct`: effort-owner retreat / direct effort profile
- `jtc_profile`: joint-trajectory-controller backed execution profile

## Rules

1. Public/runtime profiles are runtime-backed and exposed through diagnostics summaries.
2. `hard_1khz` is only valid on the daemonized runtime host; Gazebo plugin hosting must reject it at startup.
3. Strict RT profiles may fail fast instead of silently degrading when scheduler or transport contracts are not met.
4. Profile declarations are test-gated, not README-only claims.

4. On the Gazebo plugin host, `rt_scheduler_state` should report `host_managed(gazebo_update_thread)`; strict scheduler fail-fast only applies to the daemonized runtime host.
5. Gazebo-hosted observability is timer-driven on the ROS executor and is no longer published directly from the `OnUpdate()` control path.
