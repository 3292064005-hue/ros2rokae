# 快速入门指南

> 状态：Active  
> 受众：第一次使用本仓的人  
> 作用：最短路径跑起 xMate 六轴 public lane  
> 最后校验：2026-04-17

## 1. 先确认范围

当前快速入门只覆盖：
- xMate 六轴 public compatibility lane
- canonical launch
- 非实时主链和 public examples

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

可先执行：
```bash
tools/check_target_environment.sh
```

## 3. 编译工作空间

```bash
cd ~/ros2_ws0
source /opt/ros/humble/setup.bash
src/rokae_xmate3_ros2/tools/clean_build_env.sh   colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

## 4. 启动 public lane

### canonical 入口
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

或：
```bash
ros2 launch rokae_xmate3_ros2 xmate6_public.launch.py
```

说明：
- `simulation.launch.py` 是规范入口
- `xmate3_simulation.launch.py` / `xmate3_gazebo.launch.py` 是兼容别名
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
find_package(xCoreSDK CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_static)
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

## 7. 先记住这 4 条语义

1. `MoveAppend` 只负责排队，**queue accepted** 就返回成功。
2. `moveStart()` 才真正提交执行。
3. `stop()` 是 pause，不清空队列。
4. `moveReset()` 才会丢弃已排队 NRT 请求。

## 8. 出错先看哪里

- 环境/依赖：[`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- profile / query authority：[`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- public contract：[`COMPATIBILITY.md`](COMPATIBILITY.md)
- 示例分层：[`EXAMPLES.md`](EXAMPLES.md)
