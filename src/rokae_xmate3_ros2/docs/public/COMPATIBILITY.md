# Compatibility

Status: Active
Audience: public SDK integrators and compatibility reviewers
Purpose: current xMateER3 public lane boundary and opt-in rules

## 1. Public Scope

The public contract covers:

- xMateER3 six-axis public lane
- public headers: `rokae/robot.h`, `rokae/model.h`, `rokae/motion_control_rt.h`, `rokae/planner.h`, `rokae/data_types.h`, `rokae/utility.h`
- install-facing targets: `xCoreSDK::xCoreSDK_core`, `xCoreSDK::xCoreSDK_shared`, `xCoreSDK::xCoreSDK_static`, `xCoreSDK::xCoreSDK_ros_bridge`
- canonical ROS surface under `/xmate_er3/*`

The default public lane excludes calibration, RL, generic IO/register/xPanel parity, internal/full service exposure, controller-grade hardware RT parity, and path record/replay as default public behavior.

Everything else in the default public simulation lane is expected to be smoke-verifiable: connection lifecycle, power/mode control, state queries, toolset, soft limits, FK/IK, NRT motion queue/start/stop/reset, wrench diagnostics, runtime snapshot, and profile capability reporting.

## 2. Consumer Contract

Primary install-tree consumer:

```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

Runtime component consumers must request components explicitly:

```cmake
find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

Rules:

- `xCoreSDK::xCoreSDK_core` is the install-facing primary consumer.
- `xCoreSDK::xCoreSDK_shared`, `xCoreSDK::xCoreSDK_static`, and `xCoreSDK::xCoreSDK_ros_bridge` are opt-in component paths.
- `rokae_xmate3_ros2/*`, `rokae/sdk_shim*.hpp`, and `rokae/detail/*` are not install-facing public contracts.
- Source-tree ROS/Gazebo dependencies are allowed for simulation builds; they are not exported as public CMake dependencies of the core target.

## 3. Behavioral Contract

- `MoveAppend`: queue accepted means success; execution begins only through `moveStart()`.
- `stop()`: pause-only.
- `moveReset()`: clears queued NRT work and runtime execution cache.
- `calibrateFrame()`: compatibility signature only; returns `function_not_supported`.
- `GetEndWrench`: preferred public wrench query.
- `GetEndEffectorTorque`: legacy compatibility facade.
- `MoveSP`: experimental extension; default public profile rejects `sp_cmds` before queueing.
- RT compatibility APIs remain explicit opt-in; Gazebo 语义仍是 simulation-grade.
- 路径录制/回放: 默认 public profile 不注册; enable only through `public_xmate_er3_experimental` or `internal_full`.
- `replayPath()`: experimental immediate-submit side-lane and does not require `moveStart()`.
- Public smoke must also prove that IO/RL/register services are absent and that headless SDK smoke follows the same public behavior as Gazebo/JTC.

## 4. Alignment Summary

| Area | Status | Notes |
|---|---|---|
| basic robot operations and state queries | aligned | xMateER3 public lane |
| NRT motion | aligned | queue/start/pause/reset semantics are fixed |
| RT compatibility APIs | experimental opt-in | installed signatures remain; default public ROS services are not registered |
| IO / communication | outside public scope | internal/legacy only |
| RL project | outside public scope | internal/backend only |
| planner | aligned default lane | `MoveSP` and path replay are experimental extensions |
| model | simulation-grade | see [KINEMATICS_AND_MODEL.md](KINEMATICS_AND_MODEL.md) |

## 5. Source Layout

- public ROSIDL root: `srv/`
- internal/backend ROSIDL root: `internal_interfaces/srv/`
- public examples: `examples/cpp/`
- internal/backend examples: `examples/internal/cpp/`

## 6. Compatibility Alias Policy

`compatibility_alias_policy` accepts `canonical_only`, `canonical_plus_compat`, or `legacy_only`. The default is `canonical_only`.

Use `/xmate_er3/*` for current public code. Use `/xmate3/*` only for explicit compatibility-alias testing.

## 7. Consumer Matrix

- `test/compat/install_tree_core_only/`: core-only public consumer.
- `test/compat/install_tree_runtime_components/`: explicit runtime component path for `shared/static/ros_bridge`.
- `test/compat/install_tree/`: aggregate compatibility regression harness.

The model facade public header depends on the backend-neutral provider interface only. Gazebo-backed provider ownership is an implementation detail guarded by runtime source-integrity checks.

Related docs: [../reference/SDK_ALIGNMENT.md](../reference/SDK_ALIGNMENT.md), [RUNTIME_PROFILES.md](RUNTIME_PROFILES.md), [../release/BUILD_RELEASE.md](../release/BUILD_RELEASE.md), [EXAMPLES.md](EXAMPLES.md).
