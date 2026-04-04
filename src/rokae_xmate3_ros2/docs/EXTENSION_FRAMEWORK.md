# Extension Framework

This document freezes the long-term expansion plan so the package can continue to grow without re-breaking its architecture.

## Platform identity

Treat this repository as a **simulation-grade xCore SDK platform for xMate3**, not as a loose collection of ROS2 services.

That means every new feature must attach to one of six layers:

1. **Public API** – SDK-compatible headers and the ROS2 wrapper surface
2. **Contract** – message/service/action schemas and alignment docs
3. **Runtime** – the single source of truth for modes, ownership, diagnostics, RL state, tool/wobj state
4. **Kinematics** – FK, IK, Jacobian, singularity, wrench mapping, planner inverse projection
5. **Simulation / Control** – Gazebo + ros2_control profile wiring
6. **Verification** – unit, contract, strict, and harness tests

## Immutable design rules

### 1. Runtime is the only state authority

No new feature may invent a second truth in the wrapper or shim layer.
Any persistent motion mode, RT mode, RL option, active profile, tool catalog, or wobj catalog state must live in runtime.

### 2. Main contracts and legacy contracts must be explicit

Every domain keeps a preferred contract and an optional compatibility facade.
Examples:

- `ReadRegisterEx` / `WriteRegisterEx` are the preferred register contracts.
- `ReadRegister` / `WriteRegister` are compatibility facades.
- `GetEndWrench` is the preferred force/wrench contract.
- `GetEndEffectorTorque` is the compatibility facade.

### 3. Kinematics uses one primary backend per request

The default production path is:

- primary backend: `KDL`
- auxiliary backend: `improved_dh`
- finite-difference: test oracle only

A single FK / IK / Jacobian / inverse-projection request must not switch primary backends mid-solve.

### 4. RT and NRT remain permanently split

- `config/ros2_control_nrt.yaml` is the queued non-realtime profile.
- `config/ros2_control_rt.yaml` is the simulated realtime facade profile.

Do not merge them back into one control profile.

## Expansion domains

### Runtime observability

Next capabilities should land here:

- runtime trace topic
- motion session ID
- active planner name
- active kinematics backend
- fallback-trigger reason
- singularity warnings / degradation cause
- RT watchdog summary
- queue depth metrics

### Kinematics and model

Continue growing:

- manipulability / singularity grading
- improved seed pool management
- tool / payload context-aware scoring
- branch continuity scoring
- gravity / dynamics approximations exposed via model facade

### Planner

The next high-value addition is a **trajectory preflight validator** that reports:

- reachability
- chosen backend
- chosen IK branch
- continuity risk
- singularity risk
- joint-limit risk
- retimer family

### RT facade

Expand into a full subsystem with:

- field registry
- subscription plan
- loop statistics
- watchdog
- pre-arm checks
- mode capability matrix

Current baseline after Pass 5:
- field registry exists
- subscription plan builder exists
- watchdog exists
- pre-arm checks exist
- remaining work is stronger loop statistics, capability matrix, and contract exposure

### Communication / RL / cobot semantics

Stabilize these runtime domains through `runtime_catalog_service` and runtime-owned read models:

- register namespaces and indexed references
- RL project/task catalogs
- tool and wobj descriptor catalogs
- wrench / external joint torque sampling semantics
- xPanel state as an explicit runtime domain instead of an isolated knob

## Release gate for new features

A feature is not “done” until all of the following are true:

1. it is attached to the correct layer above
2. its preferred vs legacy contract status is documented
3. runtime truth is clearly defined
4. diagnostics or trace visibility exists if the feature is stateful
5. at least one contract or unit test proves the claim

## Self-check questions

Before merging any future feature, ask:

- Did I attach this feature to the right layer, or did I smuggle logic into the shim?
- Does runtime remain the only source of truth?
- Did I define a preferred contract and a compatibility contract, or is the surface ambiguous?
- Can diagnostics explain the feature when it fails?
- Is there a test proving the alignment claim?


## Pass 6 additions

- Runtime profile capability matrix
- Runtime option catalog descriptors
- Diagnostics exposure for profile/runtime-option/catalog summaries


## Pass 7 additions

- explicit profile capability query surface
- runtime diagnostics now include active request and execution backend summaries
- SDK wrapper profile capability accessors


## 2026-04 strict runtime authority landing
- SDK wrapper 的 `projectInfo/toolsInfo/wobjsInfo/setProjectRunningOpt/pauseProject` 默认不再把 runtime 失败静默伪装成成功。
- 仅当显式设置 `ROKAE_SDK_LEGACY_CATALOG_FALLBACK=true` 时，允许返回旧缓存作为兼容旁路。
- wrapper 析构不再调用全局 `rclcpp::shutdown()`；ROS 生命周期改由进程级 `RosContextOwner` 统一持有。
- `startReceiveRobotState/updateRobotState` 默认执行 strict RT state 语义：订阅计划降级直接拒绝，1ms in-loop 订阅仅接受 controller-native/本地派生 RT 字段，RT 失败不再静默回退到 NRT joint-state 查询。


### 2026-04 strict RT state addendum

- 1 ms in-loop RT subscriptions are controller-native/local-derived only.
- Cartesian pose / external torque cache fields stay available only on the slower polled state-stream path because they are still built from service-backed sources; xMate3 六轴 shim no longer exposes synthetic `psi_*` / `elbow*` RT placeholders.
