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
- control/query split now treats `SetToolset` / `SetToolsetByName` as control-domain mutators rather than query operations
- base-frame state is no longer implicitly overwritten by work-object selection

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

Recent hardening now in place:
- `setEndEffectorFrame()` and `setLoad()` propagate into both `xMateModel` and the runtime RT bridge instead of only updating one side.
- `setFcCoor()` is now consumed by Cartesian impedance desired-wrench handling through a simulation-grade frame transform path.
- `setRtNetworkTolerance()` now affects the direct-RT command timeout window.
- compat `MoveJ/MoveL/MoveC` and the source-tree shim now share one unified RT semantic command bridge instead of each formatting/publishing RT topics independently.
- RT Cartesian start/target validation now uses translation tolerance plus quaternion angular distance, avoiding false negatives around ±π Euler wrapping.
- `useRciClient(true)` now blocks the simulated direct-RT dispatch path instead of silently coexisting with it.
- RT direct-control hot paths now consume pre-parsed runtime snapshots rather than performing repeated string/CSV parsing inside the servo tick.
- SDK wrapper client initialization was extended beyond tooling/safety: RL project, path-recording, and advanced dynamics clients now initialize lazily on first use.

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
- `ppToMain()` now provides a simulation-grade reset-to-main path by reloading the last successfully loaded project path; this is still not controller-native RL pointer execution

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

Recent hardening now in place:
- session-scoped TCP/load context is shared between `setToolset()`, `setEndEffectorFrame()`, runtime RT bridge, and `xMateModel`
- runtime diagnostics now expose `rt_state_source`, `model_primary_backend`, and `model_fallback_used`

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
- `rokae/sdk_shim.hpp` remains the umbrella over `detail/sdk_shim_core.hpp` and `detail/sdk_shim_planner.hpp`; to preserve six-axis source compatibility `rokae/robot.h` again transitively includes `rokae/planner.h`.


SDK-compatible wrappers (`rokae::Robot_T`, `rokae::Cobot`, `rokae::xMateRobot`) and the native ROS facade (`rokae::ros2::xMateRobot`) now both default to strict runtime authority. Legacy catalog fallback remains available only through an explicit policy override, `ROKAE_SDK_LEGACY_CATALOG_FALLBACK=true`, or the compatibility alias `ROKAE_SDK_STRICT_RUNTIME_AUTHORITY=false`.

- SDK RT state cache updates now enforce strict RT state semantics: rejected subscription downgrades and no silent fallback to `GetJointPos/GetJointVel/GetJointTorques` during `updateRobotState()`.
- Install-facing compatibility headers now expose `rokae::Robot_T<collaborative, 6>`, the official `connectToRobot(remoteIP, localIP="")` overload, the official soft-limit array type, and direct `executeCommand(MoveC)` in the public ABI lane.
- RT compatibility semantics were hardened so `MoveC` now executes a geometric circular interpolation path with explicit start validation, degenerate-circle rejection, spherical orientation interpolation, and timeout-backed target wait; `setFilterLimit()` publishes a dedicated runtime config topic, collision thresholds no longer reuse the Cartesian desired-wrench channel, and the affected RT config setters now reject NaN/range-invalid inputs before publishing.


- Runtime unified retimer and SDK planner now share the same strict `jerk-limited` scalar trajectory kernel.


## 2026-04 RT metadata follow-up

- Added compat-only RT metadata fields `rokae::RtCompatFields::samplePeriod_s` and `rokae::RtCompatFields::sampleFresh` so the native state cache now produces one scalar and one boolean RT field alongside the existing array6/matrix16 fields.
- `src/runtime/rt_field_registry.cpp`, `src/sdk/robot_rt.cpp`, and `include/rokae/detail/sdk_shim_core.hpp` now keep these fields on the same `startReceiveRobotState()/updateRobotState()/getStateData()` path instead of exposing `getStateData<double/bool>()` without any six-axis field source.

## 2026-04 RT state follow-up

- strict in-loop field classification now rejects service-backed cartesian / external-torque fields from 1 ms subscriptions; xMate3 六轴 shim 不再把 `psi_*` / `elbow*` 暴露成任何 RT 可订阅字段。
- spec/xacro contract checks were added so joint names, limits, axes and origins cannot silently drift from `xmate3_spec.hpp`.


- 2026-04-02 follow-up hardening: fixed SDK wrapper semantics for `setSoftLimit(false, ...)`, moved remaining servo-loop metadata reads to `DataStoreState::RtSemanticSnapshot`, guarded `RuntimeBootstrap` executor release with ownership tracking, and unified native SDK facade `lastErrorCode()` recording across all public `std::error_code& ec` entrypoints.


## Recently closed implementation gaps

- Added `GetRuntimeStateSnapshot` as a runtime-owned aggregated read surface so the native SDK can reuse one coherent snapshot for multiple high-frequency read APIs.
- Collapsed primary service descriptors and compatibility aliases into `service_contract_manifest.hpp` to reduce multi-file manual drift.
- Added canonical robot-description provenance metadata (`xMate3.description.json`) so build-time and launch-time description consumption can be audited against one canonical artifact family.
- Replaced detached `MoveAppend` execution threads with tracked async workers drained by `RosBindings` teardown.


- 2026-04-03 review fix: hardened `tools/run_target_env_acceptance.sh --local-target-env` by resolving `PKG_ROOT` before the local branch, added cleanup for the temporary workspace, required `rosdep` / `ros2` in `check_target_environment.sh`, and aligned the nested-array `setSoftLimit()` overload with the official sentinel semantics so direct backend callers no longer fall back to an all-zero limit payload when omitting explicit limits.

- 2026-04-03 review fix: target-environment acceptance now emits a machine-readable report (`tools/write_target_env_report.py`) into `artifacts/target_env_acceptance/`, the acceptance workflow uploads that artifact, and `check_target_environment.sh` now fail-fast checks `gazebo_ros`, `gazebo` on PATH, and a usable `rosdep` database so environment drift is surfaced before gate execution.

- 2026-04-04 review fix: `tools/run_target_env_acceptance.sh` now preserves per-stage logs and always emits an acceptance report, even when the environment check, image build, rosdep install, quick gate, release gate, or launch smoke stage fails. `tools/write_target_env_report.py` now records stage-by-stage status so target-environment evidence is no longer lost on first failure.
