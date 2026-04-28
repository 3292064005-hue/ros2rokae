# 快速入门指南

> 状态：Active  
> 受众：第一次使用本仓的人  
> 作用：最短路径跑起 xMateER3 六轴 public lane  
> 最后校验：2026-04-18

## 1. 先确认范围

当前快速入门只覆盖：
- xMateER3 六轴 public compatibility lane
- canonical launch
- 非实时主链和 public examples
- ROS2/Gazebo-backed install-facing compatibility lane

不覆盖：
- 标定
- RL
- IO
- internal/backend 专家路径

## 2. 环境准备

### baseline
- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11

### 依赖安装
```bash
sudo apt install   ros-humble-desktop-full   ros-humble-gazebo-ros   ros-humble-gazebo-ros-pkgs   ros-humble-ros2-control   ros-humble-ros2-controllers   ros-humble-joint-state-publisher-gui   ros-humble-xacro   python3-numpy   python3-lxml   libeigen3-dev
```

先执行安装态 preflight：
```bash
ROKAE_PKG_PREFIX="$(ros2 pkg prefix rokae_xmate3_ros2)"
ROKAE_TOOLS="${ROKAE_PKG_PREFIX}/share/rokae_xmate3_ros2/tools"
"${ROKAE_TOOLS}/check_target_environment.sh"
```

## 3. 编译工作空间

### install-facing mirrored entrypoint
公共快速入门只给 install-facing 命令：
```bash
cd ~/ros2_ws0
source /opt/ros/humble/setup.bash
ROKAE_PKG_PREFIX="$(ros2 pkg prefix rokae_xmate3_ros2)"
ROKAE_TOOLS="${ROKAE_PKG_PREFIX}/share/rokae_xmate3_ros2/tools"
"${ROKAE_TOOLS}/clean_build_env.sh" colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

维护命令只在 [`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md) 中说明，不在 install-facing 公共文档里展开。

## 4. 启动 public lane

### canonical 入口
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

或：
```bash
ros2 launch rokae_xmate3_ros2 xmate_er3_public.launch.py
```

说明：
- `simulation.launch.py` 是规范入口
- `xmate3_simulation.launch.py` / `xmate3_gazebo.launch.py` 是兼容别名
- `launch_profile` 默认值现在由 `config/default_runtime_host_policy.env` 提供，默认 profile 为 `public_xmate_er3_sdk`
- `launch_profile` 现在 fail-fast，未知值会直接报错

## 5. 运行 public 示例

```bash
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_15_move_queue_and_events
ros2 run rokae_xmate3_ros2 example_99_complete_demo
```

更多示例见 [`EXAMPLES.md`](EXAMPLES.md)。

## 6. install-facing C++ 工程消费

```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
# install-facing 主消费者：
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

只使用公共头：
```cpp
#include <rokae/robot.h>
#include <rokae/model.h>
#include <rokae/motion_control_rt.h>
#include <rokae/planner.h>
#include <rokae/data_types.h>
#include <rokae/utility.h>
```

不要把 `rokae/sdk_shim*.hpp` 当作安装态 public contract；这些头属于兼容实现细节。

需要 Robot/RT/runtime bridge 时，显式请求：
```cmake
find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

只做纯 SDK / 模型消费时，可改用：
```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
```
这条路径只解析 `xCoreSDK::xCoreSDK_core`，并只暴露模型/规划 core 能力；`Robot` 会话、RT 控制需要显式请求 `shared` 组件并链接 `xCoreSDK::xCoreSDK_shared`，ROS2 action/service 桥接则走 `xCoreSDK::xCoreSDK_ros_bridge`。兼容导出仍保留 `xCoreSDK::xCoreSDK_static`，但不再作为主消费者验证链。

## 7. 先记住这 4 条语义

1. `MoveAppend` 只负责排队，**queue accepted** 就返回成功。
2. `moveStart()` 才真正提交执行。
3. `stop()` 是 pause，不清空队列。
4. `moveReset()` 才会丢弃已排队 NRT 请求。
5. `replayPath()` 是立即提交型 side-lane，不经过 `moveStart()`。

## 8. 出错先看哪里

- 环境/依赖：[`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md)
- profile / query authority：[`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- public contract：[`COMPATIBILITY.md`](COMPATIBILITY.md)
- runtime 状态机：[`../reference/RUNTIME_STATE_MACHINE.md`](../reference/RUNTIME_STATE_MACHINE.md)
- 路径录制 schema：[`../reference/RECORDED_PATH_SCHEMA.md`](../reference/RECORDED_PATH_SCHEMA.md)
- 示例分层：[`EXAMPLES.md`](EXAMPLES.md)
- 诊断门限派生工具：`share/rokae_xmate3_ros2/tools/derive_runtime_diag_gate.py`
- 分层验收矩阵：[`../release/ACCEPTANCE_LAYERS.md`](../release/ACCEPTANCE_LAYERS.md)

## 9. install-tree consumer matrix

安装态消费样例拆成三层，避免把主消费者和运行时组件混在一起：

- `test/compat/install_tree_core_only/`：只请求 `find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)`，只链接 `xCoreSDK::xCoreSDK_core`。
- `test/compat/install_tree_runtime_components/`：显式请求 `shared static ros_bridge`，用于 Robot/RT/runtime bridge 消费面。
- `test/compat/install_tree/`：聚合型回归样例，用于一次性覆盖 core 与 runtime 组合契约。

因此，`xCoreSDK::xCoreSDK_core` 是 install-facing 主消费者；`xCoreSDK::xCoreSDK_shared` 和 `xCoreSDK::xCoreSDK_ros_bridge` 只属于显式 runtime/bridge 组件路径。

### Model facade include rule

Applications that include `rokae_xmate3_ros2/model_facade.hpp` receive only the backend-neutral provider contract. Concrete Gazebo provider headers are not part of the public facade include chain.
