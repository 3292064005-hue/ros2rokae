# xMateER3 Capability Matrix

`config/xmate_er3_capability_matrix.json` is the runtime capability source for the xMateER3 six-axis simulation lane. Launch profile resolution and launch-time preflight consume this matrix through `launch/_launch_profile.py` and `launch/_simulation_support.py`; backend-specific JTC readiness is probed twice for the default public path: once by the launch readiness probe after controller spawn, and again by the runtime backend immediately before trajectory dispatch.

## Default contract

- Default launch profile: `public_xmate_er3_jtc`
- Default runtime host: Gazebo plugin
- Default backend mode: `jtc`
- Default service exposure: `public_xmate_er3_only`
- Default compatibility alias policy: `canonical_only`

## Execution policy

`jtc` uses `joint_trajectory_controller`. `headless_sim` and `effort` use the effort executor. `hybrid` can use JTC first and fall back to the effort executor when trajectory execution is unavailable.

## Service exposure

The default public profile registers only xMateER3 six-axis connectivity, state/query, toolset, kinematics, NRT motion, and field-tagged simulation diagnostics. RT, drag, path record/replay services, and the `MoveSP` motion extension require `public_xmate_er3_experimental` or `internal_full`. The ROSIDL types may still be installed for source compatibility; registration/execution is gated by service exposure and runtime request validation.

The default public smoke proves this surface by calling connect/disconnect, power, operate mode, toolset, soft limit, FK/IK, NRT motion, wrench diagnostics, runtime snapshot, and by asserting that IO/RL/register and experimental services are absent from `public_xmate_er3_only`. Headless smoke runs the same public semantics through `public_xmate_er3_headless_sdk_smoke`; experimental smoke is explicit opt-in only.

## Field validity

Wrench and torque diagnostics explicitly distinguish valid, approximate, and unsupported response fields. Consumers must check the response validity fields before using simulation diagnostics as control inputs.
