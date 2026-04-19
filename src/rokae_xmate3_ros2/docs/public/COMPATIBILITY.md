# Compatibility

> 状态：Active  
> 受众：Public SDK 集成者 / 兼容性审计人员  
> 作用：xMate6 public compatibility lane 的唯一主说明  
> 上游事实来源：`include/rokae/*`、`cmake/targets_sdk_compat.cmake`、`cmake/xCoreSDKConfig.cmake.in`、`docs/reference/xmate6_official_alignment_manifest.json`、compat tests  
> 最后校验：2026-04-18

## 1. Scope

当前 public contract 只覆盖：
- xMate 六轴 public lane
- `rokae/robot.h`
- `rokae/model.h`
- `rokae/motion_control_rt.h`
- `rokae/planner.h`
- `rokae/data_types.h`
- `rokae/utility.h`
- install-facing targets: `xCoreSDK::xCoreSDK_static`, `xCoreSDK::xCoreSDK_shared`

明确排除：
- 标定
- RL
- IO / 寄存器 / xPanel 的 public 承诺
- internal/full service exposure
- experimental RT loop examples

## 2. Consumer contract

### CMake
```cmake
find_package(xCoreSDK CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_static)
```

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
- source-tree package 仍然可以声明 ROS2/Gazebo 依赖用于仿真构建；但 install-facing `xCoreSDK` public target 不再把 Gazebo 作为 public CMake 直绑依赖导出。

## 3. Behavioral contract

- `MoveAppend`：**queue accepted** 即成功；执行由 `moveStart()` 提交。
- `stop()`：pause-only。
- `moveReset()`：清队列与执行缓存。
- `calibrateFrame()`：仅保留签名，返回 `function_not_supported`。
- `GetEndWrench`：public lane 的首选扩展查询面。
- `MoveSP`、路径录制/回放：已收敛为 public xMate6 lane 的 NRT 扩展能力，语义以 queue/start/pause 主链为准。
- profile capability 查询返回 machine-readable `authority_scope / fidelity_class / model_revision`。

## 4. Alignment summary

| 区域 | 当前状态 | 说明 |
|---|---|---|
| 机器人基本操作及信息查询 | 对齐 | xMate6 public lane 主路径 |
| 非实时运动控制 | 对齐 | NRT queue/start/pause 语义已收口 |
| 实时控制 | 部分对齐 | install-facing 保留接口；Gazebo 语义仍是 simulation-grade |
| IO / communication | 不纳入 public | 仅保留 legacy/internal 语义 |
| RL project | 不纳入 public | 仅保留 internal/backend 语义 |
| cobot specific | 部分对齐 | 拖动、路径录制/回放保留在 xMate 六轴 public lane；奇异规避不纳入 public |
| planner | 对齐 | `MoveSP` 已并入 public xMate6 lane 的 NRT 扩展面 |
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
