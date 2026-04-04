# Profile capability matrix

This document hardens the runtime profile model introduced after Pass 5.

## Profiles

- `nrt_strict_parity`: public xMate6 queued non-realtime strict-parity lane
- `rt_sim_experimental_best_effort`: simulation-grade RT facade with watchdog/prearm diagnostics and explicit non-controller-grade risk marker
- `hybrid_bridge`: trajectory + effort arbitration profile
- `effort_direct`: effort-owner retreat / direct effort profile
- `jtc_profile`: joint-trajectory-controller backed execution profile

## Rules

1. Exactly one profile is active for a runtime session.
2. Profiles are exposed through diagnostics summaries and are runtime-backed.
3. Profile declarations are test-gated, not README-only claims.
