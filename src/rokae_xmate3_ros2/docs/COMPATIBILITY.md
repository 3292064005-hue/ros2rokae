# Compatibility

> 状态：Active  
> 受众：Public SDK 集成者 / 兼容性审计人员  
> 作用：xMate6 public compatibility lane 的唯一主说明  
> 上游事实来源：`include/rokae/*`、`cmake/targets_sdk_compat.cmake`、`cmake/xCoreSDKConfig.cmake.in`、`docs/xmate6_official_alignment_manifest.json`、compat tests  
> 最后校验：2026-04-17

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
- experimental RT examples

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

## 4. Alignment matrix (condensed)

| 区域 | 说明 | 当前状态 |
|---|---|---|
| 4.3 机器人基本操作及信息查询 | xMate6 public lane 主路径 | 对齐 |
| 4.4 非实时运动控制 | NRT queue/start/pause 语义已收口 | 对齐 |
| 4.5 实时控制 | 安装态保留接口；Gazebo 语义仍是 simulation-grade | 部分对齐 |
| 4.6 IO / communication | `ReadRegisterEx / WriteRegisterEx`、`SetXPanelVout` 仅保留 legacy facade，不属于 public contract | 不纳入 public |
| 4.7 RL project | `GetRlProjectInfo` 等仅保留 internal/backend 语义 | 不纳入 public |
| 4.8 cobot specific | 只保留 xMate 六轴主线上真实消费的协作接口 | 部分对齐 |
| 8.3.7 planner | `MoveSP` 仍为 non-public / experimental 入口 | 非 public |
| 8.3.8 model | 近似实现，见 `KINEMATICS_AND_MODEL.md` | 仿真近似 |

## 5. Verification assets

- official-alignment manifest: `docs/xmate6_official_alignment_manifest.json`
- compatibility checks: `test/harness/check_compat_public_abi.py`
- install-tree consumer checks: `test/harness/run_install_tree_consumer.py`
- exported-symbol checks: `test/harness/check_exported_symbols.py`
- machine-readable matrix source: `docs/xmate6_official_alignment_manifest.json`

## 6. Related docs

- [`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- [`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- [`EXAMPLES.md`](EXAMPLES.md)
