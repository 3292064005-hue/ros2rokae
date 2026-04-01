# Implementation Audit

This document is the **truth table** for what is implemented today in the current package tree.
It is intentionally stricter than the README chapter summary.

## Reading guide

- **Implemented**: the public surface exists and the runtime path is wired through the package.
- **Simulation-grade**: the public surface is usable, but the semantics are still Gazebo/runtime approximations.
- **Not fully closed**: the package surface exists, but one or more pieces are still incomplete for claiming parity.

## Summary

The package is **not missing its entire feature set**. The current state is more nuanced:

- Base robot operations and non-realtime motion are largely implemented.
- Runtime architecture is now strong enough to be treated as a platform skeleton.
- Realtime control, RL, communication, cobot wrench/torque, planner continuity and model fidelity remain simulation-grade.
- The right next step is **hardening**, not another uncontrolled API expansion.

## Module-by-module status

### 4.3 Robot basic operations

Status: **Implemented**

What is in place:
- connect / disconnect / power / operate mode / posture / joint state
- toolset and soft-limit surfaces
- runtime-backed diagnostics service

What is still approximate:
- controller logs and alarm semantics are simulated

### 4.4 Non-realtime motion control

Status: **Implemented**

What is in place:
- MoveAbsJ / MoveJ / MoveL / MoveC / MoveCF / MoveSP
- queued execution, move reset / move start / stop
- jog / record / replay / validation / retiming hooks

What is still approximate:
- planner and retimer remain simulation-grade rather than controller-grade

### 4.5 Realtime control

Status: **Not fully closed**

What is in place:
- separate RT and NRT profiles
- RT mode state machine repaired to use `RtCommand`
- runtime diagnostics now expose `motion_mode`, `rt_mode`, `active_profile`, `loop_hz`, `state_stream_hz`, `command_latency_ms`

What is still missing for a stronger claim:
- richer loop statistics and capability matrix exposure
- stronger request-level tests around RT command semantics
- controller-grade network / watchdog semantics

Pass 5 progress:
- field-driven RT registry and subscription plan scaffolding now exist
- pre-arm checks and watchdog summaries now feed runtime diagnostics

### 4.6 IO and communication

Status: **Simulation-grade**

What is in place:
- DI / DO / AI / AO surfaces
- preferred `ReadRegisterEx` / `WriteRegisterEx`
- legacy `ReadRegister` / `WriteRegister` facades
- runtime-backed `SetXPanelVout`

What is still approximate:
- register and xPanel semantics are runtime/Gazebo models, not controller-native transport
- register descriptor scaffolding exists but is not yet exposed as a preferred query contract

### 4.7 RL project interfaces

Status: **Simulation-grade**

What is in place:
- load / start / stop
- `GetRlProjectInfo`
- `PauseRLProject`
- `SetProjectRunningOpt`
- `GetToolCatalog` / `GetWobjCatalog`
- query-domain catalog normalization via `runtime_catalog_service`

What is still approximate:
- project catalogs are runtime-backed normalized catalogs
- execution semantics are not controller-native RL project execution
- `ppToMain()` is not implemented in the Gazebo/runtime facade and now returns an explicit unsupported error instead of a fake success

### 4.8 Cobot-specific interfaces

Status: **Simulation-grade**

What is in place:
- drag / replay / record / singularity avoid flag
- preferred `GetEndWrench`
- legacy `GetEndEffectorTorque`

What is still approximate:
- `GetEndWrench` still derives from simulation/runtime estimates
- external joint torque remains simplified and may be zero-filled in fallback paths
- fidelity must still be treated as `SimApprox`

### 8.3.7 Planner

Status: **Not fully closed**

What is in place:
- planner runtime core and validation entrypoints
- blended motion and retiming scaffolding
- motion validation and structured preflight hooks

What still needs hardening:
- richer request-visible preflight reports
- stronger continuity and branch-stability enforcement
- clearer failure reporting for fallback and branch switches

### 8.3.8 Model library

Status: **Simulation-grade**

What is in place:
- FK / IK / Jacobian / torque mapping surfaces
- KDL primary backend
- improved DH auxiliary backend
- policy layer with single-primary-backend intent

What still needs hardening:
- request-level tests that prove a single request does not silently mix primary backends
- richer singularity and continuity reporting
- stronger planner ↔ model integration reports

## Preferred vs legacy contract rule

The package is now expected to follow this rule everywhere:

- `ReadRegisterEx` / `WriteRegisterEx` are preferred.
- `ReadRegister` / `WriteRegister` are legacy facades.
- `GetEndWrench` is preferred.
- `GetEndEffectorTorque` is a legacy facade.
- RL catalog and runtime-option services are preferred runtime surfaces.

## Final engineering judgement

This package should now be treated as:

> a strong simulation SDK platform skeleton with broad interface coverage,
> not yet a controller-grade parity implementation.

That is the correct baseline for future work.


## Pass 6 additions

- Runtime profile capability matrix
- Runtime option catalog descriptors
- Diagnostics exposure for profile/runtime-option/catalog summaries


## Pass 7 additions

- profile capability query surface
- runtime diagnostics now expose active request id and execution backend
- SDK wrapper can fetch profile and runtime-option capability snapshots


## 2026-04 full-coverage follow-up
- `rokae::ros2::xMateRobot` and SDK shim entrypoints now support externally injected ROS ownership via `RosClientOptions`.
- planner / query IK requests now enforce a request-level single-primary-backend contract rather than only exposing trace metadata.
- `rokae/sdk_shim.hpp` is now an umbrella over `detail/sdk_shim_core.hpp` and `detail/sdk_shim_planner.hpp`; `rokae/planner.h` is no longer transitively pulled by `rokae/robot.h`.


SDK-compatible wrappers (`rokae::Robot_T`, `rokae::Cobot`, `rokae::xMateRobot`) now default to the legacy catalog fallback policy across both their historical constructors and the `RosClientOptions` injection path so that historical SDK call sites keep their prior behavior unless they explicitly opt into strict runtime authority. The native ROS facade (`rokae::ros2::xMateRobot`) still defaults to strict runtime authority.
