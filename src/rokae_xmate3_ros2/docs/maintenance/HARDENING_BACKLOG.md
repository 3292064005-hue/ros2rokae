# Hardening Backlog

This backlog is intentionally short and ordered. It is the set of remaining items most likely to prevent future rework.

## P0 — Keep the current correct skeleton fixed

Do not revert these:
- runtime as the only state authority
- RT/NRT split profiles
- KDL primary backend + improved DH auxiliary backend
- preferred vs legacy contract split
- per-interface alignment documentation

## P1 — Enforce single-primary-backend requests

Goal:
- one FK / IK / Jacobian / planner inverse-projection request uses exactly one primary backend

Needed work:
- request-level tracing in kinematics [done]
- request-level tests proving no silent backend mixing [done]
- planner tests that bind continuity scoring to the same primary backend used for solving [done]

## P2 — Planner preflight reporting

Goal:
- expose a first-class report before execution

Needed work:
- reachable / singularity / continuity / branch-switch / fallback notes
- a stable report object or service-level payload
- tests for failure reasons and fallback notes

## P3 — RT subsystem hardening

Goal:
- turn the simulated RT facade into a stronger subsystem

Needed work:
- RT field registry
- RT subscription plan
- watchdog / pre-arm checks
- richer loop statistics and diagnostics tests

Progress in Pass 5:
- added `rt_field_registry`
- added `rt_subscription_plan`
- added `rt_watchdog` and `rt_prearm_checks`
- surfaced RT subscription / prearm / watchdog data through runtime diagnostics

## P4 — Runtime catalog normalization

Goal:
- normalize RL / tool / wobj runtime truth

Progress in Pass 4:
- added `runtime_catalog_service`
- moved `GetRlProjectInfo` / `GetToolCatalog` / `GetWobjCatalog` into the query domain
- added runtime-catalog unit coverage and policy documentation

Remaining work:
- extend normalized catalogs into SDK legacy wrappers
- add register namespace descriptors as a runtime-backed catalog

## P5 — Contract and docs lock-in

Goal:
- every stronger alignment claim is test-backed

Needed work:
- more contract tests
- no chapter-level overclaiming in README
- every preferred/legacy pair documented in one place

## P0 — Runtime state machine

- make terminal runtime status writes go through an explicit `RuntimeStateMachine`
- keep `planner_loop` and `runtime_execution_switch` as event producers, not final state authorities
- prove the split with unit tests


## 2026-04 strict runtime authority landing
- SDK wrapper 的 `projectInfo/toolsInfo/wobjsInfo/setProjectRunningOpt/pauseProject` 默认不再把 runtime 失败静默伪装成成功。
- 仅当显式设置 `ROKAE_SDK_LEGACY_CATALOG_FALLBACK=true` 时，允许返回旧缓存作为兼容旁路。
- `startReceiveRobotState/updateRobotState` 默认执行 strict RT state 语义：订阅计划降级直接拒绝，1ms in-loop 订阅仅接受 controller-native/本地派生 RT 字段，RT 失败不再静默回退到 NRT joint-state 查询。
- wrapper 析构不再调用全局 `rclcpp::shutdown()`；ROS 生命周期改由进程级 `RosContextOwner` 统一持有。
