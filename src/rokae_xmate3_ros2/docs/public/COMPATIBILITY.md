# Compatibility

> 状态：Active  
> 受众：Public SDK 集成者 / 兼容性审计人员  
> 作用：xMateER3 public compatibility lane 的唯一主说明  
> 上游事实来源：`include/rokae/*`、`cmake/targets_sdk_compat.cmake`、`cmake/xCoreSDKConfig.cmake.in`、`docs/reference/xmate_er3_alignment_manifest.json`、compat tests  
> 最后校验：2026-04-18

## 1. Scope

当前 public contract 只覆盖：
- xMateER3 六轴 public lane
- `rokae/robot.h`
- `rokae/model.h`
- `rokae/motion_control_rt.h`
- `rokae/planner.h`
- `rokae/data_types.h`
- `rokae/utility.h`
- install-facing targets: `xCoreSDK::xCoreSDK_core`, `xCoreSDK::xCoreSDK_shared`, `xCoreSDK::xCoreSDK_ros_bridge`（其中主消费者为 `xCoreSDK::xCoreSDK_core`；运行时桥接 target 需显式组件请求；兼容导出仍保留 `xCoreSDK::xCoreSDK_static`）

明确排除：
- 标定
- RL
- IO / 寄存器 / xPanel 的 public 承诺
- internal/full service exposure
- experimental RT loop examples

## 2. Consumer contract

### CMake
```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
# install-facing 主消费者：
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

若只做纯模型/规划消费，再改用 `find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)` + `xCoreSDK::xCoreSDK_core`。

### Public include surface
```cpp
#include <rokae/robot.h>
#include <rokae/model.h>
#include <rokae/motion_control_rt.h>
#include <rokae/planner.h>
#include <rokae/data_types.h>
#include <rokae/utility.h>
```

说明：
- `rokae_xmate3_ros2/*`、`rokae/sdk_shim*.hpp`、`rokae/detail/*` 不属于 install-facing public contract。
- `xCoreSDK::xCoreSDK_core` 是 install-facing 主消费者；`xCoreSDK::xCoreSDK_shared` / `xCoreSDK::xCoreSDK_static` / `xCoreSDK::xCoreSDK_ros_bridge` 仅在显式请求 shared/static/ros_bridge 组件时导出，用于 Robot 会话、RT 控制和 ROS2 runtime / action / service 桥接能力。
- source-tree package 仍然可以声明 ROS2/Gazebo 依赖用于仿真构建；但 install-facing `xCoreSDK` public target 不再把 Gazebo 作为 public CMake 直绑依赖导出。
- 默认 launch/runtime host 与 install-facing `xCoreSDK_BACKEND_MODE` 统一由 `config/default_runtime_host_policy.env` 单点导出到 `xCoreSDKConfig.cmake`。

## 3. Behavioral contract

- `MoveAppend`：**queue accepted** 即成功；执行由 `moveStart()` 提交。
- `stop()`：pause-only。
- `moveReset()`：清队列与执行缓存。
- `calibrateFrame()`：仅保留签名，返回 `function_not_supported`。
- `GetEndWrench`：public lane 的首选扩展查询面。
- `MoveSP`：已收敛为 public xMateER3 lane 的 NRT 扩展能力，遵循 queue/start/pause 主链。
- 路径录制：属于 public xMateER3 lane 的 NRT 扩展能力。
- `replayPath()`：立即提交型 side-lane；不进入 `MoveAppend -> moveStart()` staged queue，但仍受 NRT 连接/上电/runtime gate 约束。
- profile capability 查询返回 machine-readable `authority_scope / fidelity_class / model_revision`。

## 4. Alignment summary

| 区域 | 当前状态 | 说明 |
|---|---|---|
| 机器人基本操作及信息查询 | 对齐 | xMateER3 public lane 主路径 |
| 非实时运动控制 | 对齐 | NRT queue/start/pause 语义已收口 |
| 实时控制 | 部分对齐 | install-facing 保留接口；Gazebo 语义仍是 simulation-grade |
| IO / communication | 不纳入 public | 仅保留 legacy/internal 语义 |
| RL project | 不纳入 public | 仅保留 internal/backend 语义 |
| cobot specific | 部分对齐 | 拖动、路径录制/回放保留在 xMateER3 六轴 public lane；奇异规避不纳入 public |
| planner | 对齐 | `MoveSP` 已并入 public xMateER3 lane 的 NRT 扩展面 |
| model | 仿真近似 | 见 `KINEMATICS_AND_MODEL.md` |

## 5. Source-tree boundary

- public contract ROSIDL root: `srv/`
- internal/backend-only ROSIDL root: `internal_interfaces/srv/`
- public examples root: `examples/cpp/`
- internal/backend examples root: `examples/internal/cpp/`

## 6. Related docs

- [`../reference/SDK_ALIGNMENT.md`](../reference/SDK_ALIGNMENT.md)
- [`../reference/RUNTIME_STATE_MACHINE.md`](../reference/RUNTIME_STATE_MACHINE.md)
- [`../reference/RECORDED_PATH_SCHEMA.md`](../reference/RECORDED_PATH_SCHEMA.md)
- [`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- [`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md)
- [`EXAMPLES.md`](EXAMPLES.md)


- [`../release/ACCEPTANCE_LAYERS.md`](../release/ACCEPTANCE_LAYERS.md)


- `compatibility_alias_policy`：`canonical_plus_compat` / `canonical_only` / `legacy_only`，默认导出为 `canonical_plus_compat`。

## 7. Consumer matrix

Install-tree consumer coverage is split by contract surface:

- `test/compat/install_tree_core_only/` validates the pure C++ core SDK consumer without resolving runtime/ROS bridge targets.
- `test/compat/install_tree_runtime_components/` validates the explicit runtime component path for `shared/static/ros_bridge`.
- `test/compat/install_tree/` remains the aggregate compatibility regression harness.

This split is intentional: the core target is the install-facing primary consumer, while runtime and ROS bridge targets are opt-in components.

### Public model provider boundary

The model facade public header depends on the backend-neutral provider interface only. Gazebo-backed provider ownership is an implementation detail and is guarded by the runtime source-integrity checks.
