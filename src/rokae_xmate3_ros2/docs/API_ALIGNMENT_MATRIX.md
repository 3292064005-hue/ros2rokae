# API Alignment Matrix

This project tracks alignment at the **individual-interface level**. Do not interpret a chapter label as blanket parity.

## Conventions

- **Preferred**: the main contract users should build against.
- **Legacy facade**: compatibility surface kept for older examples or SDK-shaped code.
- **StrictAligned**: public shape and intended simulation semantics are aligned.
- **SimApprox**: public shape is present, but the implementation is simulation-grade.
- **Experimental**: usable for validation, but not stable enough for parity claims.
- **Unsupported**: intentionally absent.

## 4.3 Robot basic operations and state

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| connect / disconnect | `Connect`, `Disconnect`, wrapper connect APIs | Preferred | StrictAligned | Runtime and wrapper surfaces share one session state. |
| power / operate mode | `GetPowerState`, `SetPowerState`, `GetOperateMode`, `SetOperateMode` | Preferred | StrictAligned | Runtime truth is authoritative. |
| joint / posture / toolset reads | `GetJointPos`, `GetJointVel`, `GetPosture`, `GetCartPosture`, `GetToolset` | Preferred | StrictAligned | Simulation-grade state values, stable interface shape. |
| soft limit | `GetSoftLimit`, `SetSoftLimit` | Preferred | StrictAligned | Runtime-backed defaults and spec limits are wired. |
| clear alarm / logs | `ClearServoAlarm`, `QueryControllerLog` | Preferred | SimApprox | Alarm and controller logs are simulated. |

## 4.4 Non-realtime motion control

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| motion mode / move reset / move start / stop | `SetMotionControlMode`, `MoveReset`, `MoveStart`, `Stop` | Preferred | StrictAligned | Ownership and queue semantics are runtime-backed. |
| MoveAbsJ / MoveJ / MoveL / MoveC / MoveCF / MoveSP | actions + planner runtime | Preferred | StrictAligned | Simulated execution with stable public shape. |
| validation / retiming | `ValidateMotion` | Preferred | SimApprox | Preflight is simulation-grade but already exposes branch and retimer notes. |
| blend / default speed / default zone / conf | `SetDefaultSpeed`, `SetDefaultZone`, `SetDefaultConfOpt` | Preferred | SimApprox | Good surface coverage, simulation-grade internals. |

## 4.5 Realtime control

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| realtime mode selection | `SetRtControlMode` | Preferred | Experimental | State machine bug fixed; still a simulated facade. |
| loop / state stream diagnostics | `RuntimeDiagnostics`, runtime status topic | Preferred | Experimental | Exposes active profile, loop rate, stream rate, latency. |
| RT command loop | `robot_rt.cpp`, runtime control bridge | Preferred | Experimental | Simulated facade, not controller-grade UDP timing. |
| RT profile selection | `ros2_control_rt.yaml` | Preferred | Experimental | Explicit split from NRT profile is enforced. |

## 4.6 IO and communication

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| DI / DO / AI / AO | `GetDI`, `GetDO`, `SetDI`, `SetDO`, `GetAI`, `SetAO` | Preferred | StrictAligned | Stable simulation-grade contract. |
| indexed register IO | `ReadRegisterEx`, `WriteRegisterEx` | Preferred | SimApprox | Preferred semantic model with `name + index`. |
| key/value register IO | `ReadRegister`, `WriteRegister` | Legacy facade | FacadeOnly | Compatibility facade over the preferred register model. |
| xPanel output | `SetXPanelVout` | Preferred | SimApprox | Explicit runtime-backed simulation knob. |

## 4.7 RL project interfaces

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| project catalog | `GetRlProjectInfo` | Preferred | SimApprox | Runtime-backed simulated catalog. |
| load / start / stop | `LoadRLProject`, `StartRLProject`, `StopRLProject` | Preferred | SimApprox | Stable facade, simulated execution semantics. |
| pause / running options | `PauseRLProject`, `SetProjectRunningOpt` | Preferred | SimApprox | Runtime-backed loop/rate options. |
| tool / wobj catalogs | `GetToolCatalog`, `GetWobjCatalog` | Preferred | SimApprox | Explicit runtime catalog surfaces. |

## 4.8 Cobot-specific interfaces

| Manual surface | Package surface | Preferred / Legacy | Status | Notes |
|---|---|---|---|---|
| drag / replay / record | drag + replay services/actions | Preferred | SimApprox | Stable facade with simulated mechanics. |
| structured wrench / force | `GetEndWrench` | Preferred | SimApprox | Preferred semantic surface for frame-aware wrench data. |
| simple end torque array | `GetEndEffectorTorque` | Legacy facade | FacadeOnly | Compatibility-only array surface. |
| singularity avoidance | `SetAvoidSingularity`, `GetAvoidSingularity` | Preferred | SimApprox | Policy-backed runtime flag, simulation-grade effect. |

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
