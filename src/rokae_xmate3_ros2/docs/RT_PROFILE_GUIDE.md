# RT / NRT Profile Guide

The project now keeps non-realtime and realtime semantics in separate profiles.

## NRT profile
File: `config/ros2_control_nrt.yaml`

Intended for:
- MoveAbsJ / MoveJ / MoveL / MoveC / MoveCF / MoveSP
- queued execution
- blend / zone behaviour
- replay and RL coordination

## RT profile
File: `config/ros2_control_rt.yaml`

Intended for:
- simulated realtime loop facade
- 1 kHz loop bookkeeping
- streamed state + owner arbitration
- loop diagnostics / timing tolerance

## Observability
`RuntimeDiagnostics` now exposes:
- `motion_mode`
- `rt_mode`
- `active_profile`
- `loop_hz`
- `state_stream_hz`
- `command_latency_ms`
