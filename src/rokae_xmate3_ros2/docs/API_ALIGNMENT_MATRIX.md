# API Alignment Matrix

This project tracks alignment at the **individual-interface level**. Do not interpret a chapter label as blanket parity.

## Conventions

Runtime service registration and compatibility aliases are now derived from `src/runtime/service_contract_manifest.hpp`, which is treated as the single contract manifest for the public ROS surface.

- **Preferred**: the main contract users should build against.
- **Legacy facade**: compatibility surface kept for older examples or SDK-shaped code.
- **StrictAligned**: public shape and intended simulation semantics are aligned.
- **SimApprox**: public shape is present, but the implementation is simulation-grade.
- **Experimental**: usable for validation, but not stable enough for parity claims.
- **Unsupported**: intentionally absent.

## 4.3 Robot basic operations and state

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| connect / disconnect | `Connect`, `Disconnect`, wrapper connect APIs | Preferred | StrictAligned | Runtime and wrapper surfaces share one session state; repeated SDK calls are treated as idempotent success so teardown/setup code can mirror official-style call flows. |
| power / operate mode | `GetPowerState`, `SetPowerState`, `GetOperateMode`, `SetOperateMode` | Preferred | StrictAligned | Runtime truth is authoritative and the native SDK wrapper now prefers the aggregated runtime snapshot before falling back to dedicated legacy query services. |
| joint / posture / toolset reads | `GetJointPos`, `GetJointVel`, `GetPosture`, `GetCartPosture`, `GetToolset`, `GetRuntimeStateSnapshot` | Preferred | StrictAligned | Simulation-grade state values, stable interface shape. Native SDK now prefers one runtime-owned aggregated snapshot before falling back to individual legacy query services. |
| soft limit | `GetSoftLimit`, `SetSoftLimit` | Preferred | StrictAligned | Runtime-backed defaults, mechanical-limit validation, manual-mode/off-power preconditions, current-joint range checks, and authoritative backend-snapshot validation are enforced before enable. |
| clear alarm / logs | `ClearServoAlarm`, `QueryControllerLog` | Preferred | SimApprox | Alarm and controller logs are simulated, but `QueryControllerLog` now returns latest-first windows with level filtering semantics. |

## 4.4 Non-realtime motion control

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| motion mode / move reset / move start / stop | `SetMotionControlMode`, `MoveReset`, `MoveStart`, `Stop` | Preferred | StrictAligned | Ownership and queue semantics are runtime-backed; repeated mode-change requests that already match the active runtime mode now succeed without redundant transport work, and `moveStart()` now rejects disconnected/Idle sessions before flushing cached commands. |
| MoveAbsJ / MoveJ / MoveL / MoveC / MoveCF / MoveSP | actions + planner runtime | Preferred | StrictAligned | Simulated execution with stable public shape. |
| validation / retiming | `ValidateMotion` | Preferred | SimApprox | Preflight is simulation-grade but already exposes branch and retimer notes. |
| blend / default speed / default zone / conf | `SetDefaultSpeed`, `SetDefaultZone`, `SetDefaultConfOpt` | Preferred | SimApprox | Good surface coverage, simulation-grade internals. |

## 4.5 Realtime control

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| realtime mode selection | `SetRtControlMode` | Preferred | Experimental | State machine bug fixed; still a simulated facade. |
| loop / state stream diagnostics | `RuntimeDiagnostics`, runtime status topic | Preferred | Experimental | Exposes active profile, loop rate, stream rate, latency. |
| runtime profile / option capability query | `GetProfileCapabilities` | Preferred | SimApprox | Runtime-backed profile and option descriptor surface. |
| RT command loop | `robot_rt.cpp`, runtime control bridge | Preferred | Experimental | Simulated facade, not controller-grade UDP timing. RT `MoveC` now performs geometric circle fitting, explicit start validation, quaternion-aware start/target checks, radius-aware arc-step Cartesian dispatch, and uses the same semantic RT command bridge as the source-tree shim on the xMate6 lane. |
| RT profile selection | `ros2_control_rt.yaml` | Preferred | Experimental | Explicit split from NRT profile is enforced. |
| RT TCP / load context | `SetEndEffectorFrame`, `SetLoad`, `model().setTcpCoor`, `model().setLoad` | Preferred | SimApprox | Session-scoped TCP/load context is now shared between model and runtime bridge. |
| RT force-control frame | `SetFcCoor` | Preferred | SimApprox | Consumed by Cartesian impedance wrench transformation; still simulation-grade. |
| RT network tolerance / RCI compatibility | `SetRtNetworkTolerance`, `UseRciClient` | Preferred | SimApprox | Tolerance now affects direct-RT timeout window; RCI compatibility blocks direct-RT dispatch when enabled. |

## 4.6 IO and communication

The install-facing public xMate6 contract marks IO/registers as out of scope. Internal ROS/backend plumbing remains for runtime validation and package-internal workflows, but the public SDK lane no longer advertises these calls as supported behavior.


| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| DI / DO / AI / AO | `GetDI`, `GetDO`, `SetDI`, `SetDO`, `GetAI`, `SetAO` | Preferred | Unsupported | Install-facing public xMate6 lane returns deterministic `not_implemented`; backend/internal validation path remains outside the public contract. |
| indexed register IO | `ReadRegisterEx`, `WriteRegisterEx` | Preferred | Unsupported | Install-facing public xMate6 lane returns deterministic `not_implemented`; backend/internal validation path remains outside the public contract. |
| key/value register IO | `ReadRegister`, `WriteRegister` | Legacy facade | Unsupported | Symbol names remain for source compatibility, but the public lane no longer promises register semantics. |
| xPanel output | `SetXPanelVout` | Preferred | Unsupported | Install-facing public xMate6 lane returns deterministic `not_implemented`. |

## 4.7 RL project interfaces

The install-facing public xMate6 contract marks RL/catalog workflows as out of scope. Internal ROS/backend plumbing may still keep simulation-grade implementations for source-tree validation, but the public SDK lane no longer advertises these calls as supported behavior.

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| project catalog | `GetRlProjectInfo` | Preferred | Unsupported | Install-facing public xMate6 lane returns deterministic `not_implemented`; internal/backend simulation remains out of public scope. |
| load / start / stop | `LoadRLProject`, `StartRLProject`, `StopRLProject` | Preferred | Unsupported | Install-facing public xMate6 lane returns deterministic `not_implemented`; internal/backend simulation remains out of public scope. |
| pause / running options | `PauseRLProject`, `SetProjectRunningOpt` | Preferred | Unsupported | Install-facing public xMate6 lane returns deterministic `not_implemented`; internal/backend simulation remains out of public scope. |
| tool / wobj catalogs | `GetToolCatalog`, `GetWobjCatalog` | Preferred | Unsupported | Install-facing public xMate6 lane does not advertise catalog enumeration; backend/internal validation may still keep simulation-grade catalog services. |

## 4.8 Cobot-specific interfaces

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| drag / replay / record | drag + replay services/actions | Preferred | SimApprox | Stable facade with simulated mechanics; drag now requires manual+power-off preconditions and replay requires matching toolset context plus rate within `(0, 3)`. |
| structured wrench / force | `GetEndWrench` | Preferred | SimApprox | Preferred semantic surface for frame-aware wrench data. |
| simple end torque array | `GetEndEffectorTorque` | Legacy facade | FacadeOnly | Compatibility-only array surface. |
| singularity avoidance | `SetAvoidSingularity`, `GetAvoidSingularity` | Preferred | Unsupported | The xMate6 compatibility lane explicitly rejects this surface; the official manual scopes it to xMateCR/xMateSR only, not generic xMate6 operation. |

## 8.3.7 Planner

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| queued planning / retiming | planner runtime core | Preferred | SimApprox | Stable architecture, simulation-grade limits and timing. |
| preflight validation | `ValidateMotion` | Preferred | SimApprox | Will remain the main pre-execution validation surface. |
| blended cartesian continuity | planner core | Preferred | Experimental | Still the highest-risk area for future refinement. |

## 8.3.8 Model library

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| FK / IK | `model()` facade + `CalcFk`, `CalcIk` | Preferred | SimApprox | Policy-driven multi-backend architecture. |
| Jacobian | `model()` facade | Preferred | SimApprox | KDL native or improved-DH geometric path. |
| wrench / torque mapping | `MapCartesianToJointTorque`, `GetEndWrench` | Preferred | SimApprox | Simulation-grade approximation. |
| backend policy | `KinematicsPolicy` | Preferred | SimApprox | KDL primary, improved-DH auxiliary, single-primary-backend rule. |

## Unsupported or intentionally absent

- controller-grade UDP transport parity
- proprietary controller firmware internals
- guaranteed hard realtime behaviour outside Gazebo simulation
