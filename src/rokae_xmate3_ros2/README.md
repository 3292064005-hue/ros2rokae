# ROKAE xMate3 ROS2 仿真 SDK

> xCore SDK C++ API 的 ROS2 Gazebo 仿真环境实现

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)

---

## 目录

- [功能特性](#功能特性)
- [文件结构](#文件结构)
- [快速开始](#快速开始)
- [示例程序](#示例程序)
- [API 参考](#api-参考)
- [架构说明](#架构说明)
- [常见问题](#常见问题)

---

## 功能特性

### 完整的 xCore SDK C++ API 支持

| 章节 | 功能 | 状态 |
|------|------|------|
| **4.3** | 机器人基本操作及信息查询 | ✅ 完整支持 |
| **4.4** | 非实时运动控制接口 | ✅ 完整支持 |
| **4.6** | IO与通信接口 | ✅ 完整支持 |
| **4.8** | 协作机器人专属接口 | ✅ 完整支持 |

### 运动指令

- `MoveAbsJ` - 轴空间绝对运动
- `MoveJ` - 轴空间相对运动（笛卡尔目标）
- `MoveL` - 直线运动
- `MoveC` - 圆弧运动
- `MoveCF` - 圆弧运动（指定角度）
- `MoveSP` - 螺旋运动

### 运动学计算

- 正运动学 (FK)
- 逆运动学 (IK)
- 雅可比矩阵计算

### 轨迹规划

- 关节空间插补
- 笛卡尔空间直线插补
- 圆弧插补
- S型速度规划

---

## 文件结构

```
rokae_xmate3_ros2/
├── CMakeLists.txt              # CMake构建配置
├── package.xml                 # ROS2包清单
├── README.md                   # 本文件
│
├── include/                    # 头文件
│   └── rokae_xmate3_ros2/
│       ├── robot.hpp           # SDK主头文件
│       ├── types.hpp           # 类型定义
│       └── gazebo/             # Gazebo插件头文件
│           ├── kinematics.hpp
│           ├── motion_types.hpp
│           └── trajectory_planner.hpp
│
├── src/                        # 源代码
│   ├── robot.cpp               # SDK实现
│   └── gazebo/
│       ├── xcore_controller_gazebo_plugin.cpp
│       ├── kinematics.cpp
│       └── trajectory_planner.cpp
│
├── examples/                   # 示例程序 (按SDK手册章节组织)
│   ├── README.md               # 示例说明
│   └── cpp/
│       ├── 01_basic_connect.cpp           # 第4.3节 - 基础连接
│       ├── 02_joint_cartesian_read.cpp    # 第4.3节 - 关节/位姿读取
│       ├── 03_kinematics.cpp              # 第4.3节 - 运动学计算
│       ├── 04_motion_basic.cpp            # 第4.4节 - 基础运动
│       ├── 05_motion_cartesian.cpp        # 第4.4节 - 笛卡尔运动
│       ├── 06_io_control.cpp              # 第4.6节 - IO控制
│       ├── 07_safety_collision.cpp        # 第4.3/4.8节 - 安全功能
│       ├── 08_path_record_replay.cpp      # 第4.8节 - 路径录制回放
│       └── 99_complete_demo.cpp           # 综合演示
│
├── launch/                     # Launch文件
│   ├── simulation.launch.py    # 完整仿真启动
│   ├── rviz_only.launch.py     # 仅RViz可视化
│   └── xmate3_simulation.launch.py (旧版兼容)
│
├── config/                     # 配置文件
│   ├── ros2_control.yaml       # ROS2 Control配置
│   └── xMate3.rviz            # RViz配置
│
├── urdf/                       # 机器人模型
│   ├── xMate3.xacro
│   ├── xmate3_gazebo.xacro
│   └── materials.xacro
│
├── meshes/                     # 网格模型
│   ├── visual/
│   └── collision/
│
├── worlds/                     # Gazebo世界文件
│   └── empty.world
│
├── docs/                       # 文档
│   └── QUICKSTART.md           # 快速入门指南
│
├── example/                    # 旧版示例 (保留兼容)
├── msg/                        # ROS2消息定义
├── srv/                        # ROS2服务定义
└── action/                     # ROS2动作定义
```

---

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
sudo apt install \
  ros-humble-desktop-full \
  ros-humble-gazebo-ros \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  libeigen3-dev
```

### 2. 编译

```bash
cd ~/ros2_ws0
colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

### 3. 启动仿真

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

> ⚠️ **等待插件就绪**
>
> 仿真启动后，xCore 控制插件需要少许时间来完成初始化并提供 `/xmate3/cobot/move_append` 动作服务器。
> 在运行示例程序之前请观察控制台输出（会显示“仿真环境启动完成”提示），或者手动使用 `ros2 action list` 确认动作服务器已出现。
> SDK 在内部会等待该服务器最多 10 秒，示例也会打印错误信息以便排查。  

### 4. 运行示例

```bash
# 综合演示（推荐）
ros2 run rokae_xmate3_ros2 example_99_complete_demo

# 或按章节学习
ros2 run rokae_xmate3_ros2 example_01_basic_connect
ros2 run rokae_xmate3_ros2 example_04_motion_basic
```

详细步骤请参考 [快速入门指南](docs/QUICKSTART.md)。

---

## 示例程序

所有示例程序都严格按照 **xCore SDK C++ 使用手册** 的章节组织：

| 示例 | 对应手册章节 | 功能说明 |
|------|-------------|---------|
| 01 | 4.3 | 机器人基本连接与信息查询 |
| 02 | 4.3 | 关节位置与笛卡尔位姿读取 |
| 03 | 4.3 | 运动学计算 (FK/IK) |
| 04 | 4.4 | 基础运动控制 (MoveAbsJ) |
| 05 | 4.4 | 笛卡尔空间运动 (MoveJ/MoveL/MoveC) |
| 06 | 4.6 | IO控制 (DI/DO/AI/AO) |
| 07 | 4.3/4.8 | 安全与碰撞检测 |
| 08 | 4.8 | 路径录制与回放 |
| 99 | - | 综合演示 |

更多示例说明请参考 [examples/README.md](examples/README.md)。

---

## API 参考

### 机器人基本操作 (4.3)

```cpp
#include "rokae_xmate3_ros2/robot.hpp"

// 创建机器人对象
rokae::ros2::xMateRobot robot;

// 连接机器人
std::error_code ec;
robot.connectToRobot(ec);

// 上电
robot.setPowerState(true, ec);

// 设置操作模式
robot.setOperateMode(rokae::OperateMode::automatic, ec);

// 获取信息
auto info = robot.robotInfo(ec);
auto joints = robot.jointPos(ec);
auto pose = robot.cartPosture(rokae::CoordinateType::flangeInBase, ec);
```

### 运动控制 (4.4)

```cpp
// 设置运动控制模式
robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);

// 设置默认参数
robot.setDefaultSpeed(50, ec);   // 50% 速度
robot.setDefaultZone(5, ec);     // 5mm 转弯区
robot.moveReset(ec);            // 清空缓存

// MoveAbsJ - 轴空间绝对运动 (命令会先缓存到本地，
// 只有在调用 moveStart() 时才通过 /xmate3/cobot/move_append
// 动作服务器一次性发送到 Gazebo 插件)
rokae::MoveAbsJCommand absj_cmd;
absj_cmd.target.joints = {0.5, 0.3, -0.2, 0.0, 0.5, 0.0};
absj_cmd.speed = 30;
absj_cmd.zone = 0;
robot.moveAbsJ(absj_cmd, ec);

// MoveL - 直线运动，同样会先缓存
rokae::MoveLCommand l_cmd;
l_cmd.target.x = 0.4;
l_cmd.target.y = 0.0;
l_cmd.target.z = 0.5;
l_cmd.target.rx = 3.14159;
l_cmd.speed = 20;
robot.moveL(l_cmd, ec);

// 启动运动（触发 MoveAppend action 并清空缓存）
robot.moveStart(ec);
```

### 运动学计算 (4.3)

```cpp
// 正运动学
rokae::JointPosition jp;
jp.joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
auto fk = robot.calcFk(jp, ec);

// 逆运动学
rokae::CartesianPosition target;
target.x = 0.4;
target.y = 0.0;
target.z = 0.5;
target.rx = 3.14159;
auto ik = robot.calcIk(target, ec);
```

### IO控制 (4.6)

```cpp
// 数字IO
bool di = robot.getDI(0, 0, ec);
robot.setDO(0, 0, true, ec);

// 模拟IO
double ai = robot.getAI(0, 0, ec);
robot.setAO(0, 0, 2.5, ec);
```

完整API文档请参考 xCore SDK C++ 使用手册。

---

## 架构说明

```
┌─────────────────────────────────────────────────────────────┐
│                    用户应用程序                               │
│              (使用 xCore SDK C++ API)                        │
│  example_01_basic_connect  example_04_motion_basic  ...    │
└───────────────────────┬─────────────────────────────────────┘
                        │ ROS2 服务/Action
┌───────────────────────▼─────────────────────────────────────┐
│              rokae_xmate3_ros2 SDK 库                        │
│              (robot.hpp / robot.cpp)                          │
│         xCore SDK C++ API 的 ROS2 客户端封装                  │
└───────────────────────┬─────────────────────────────────────┘
                        │ ROS2 服务/话题
┌───────────────────────▼─────────────────────────────────────┐
│          xcore_controller_gazebo_plugin                      │
│    (Gazebo 插件 - 运动控制、轨迹规划、运动学)                  │
└───────────────────────┬─────────────────────────────────────┘
                        │ Gazebo API
┌───────────────────────▼─────────────────────────────────────┐
│                     Gazebo 11 仿真器                         │
│              (物理引擎 + xMate3 URDF模型)                     │
└─────────────────────────────────────────────────────────────┘
```

---

## ROS2 接口

### 服务 (Services)

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/xmate3/cobot/connect` | `Connect` | 连接机器人 |
| `/xmate3/cobot/disconnect` | `Disconnect` | 断开连接 |
| `/xmate3/cobot/set_power_state` | `SetPowerState` | 电源控制 |
| `/xmate3/cobot/get_joint_pos` | `GetJointPos` | 获取关节位置 |
| `/xmate3/cobot/get_cart_posture` | `GetCartPosture` | 获取笛卡尔位姿 |
| `/xmate3/cobot/calc_fk` | `CalcFk` | 正运动学计算 |
| `/xmate3/cobot/calc_ik` | `CalcIk` | 逆运动学计算 |
| `/xmate3/cobot/move_reset` | `MoveReset` | 重置运动队列 |
| `/xmate3/cobot/move_start` | `MoveStart` | 启动运动 |
| `/xmate3/cobot/stop` | `Stop` | 停止运动 |

### 话题 (Topics)

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/xmate3/cobot/operation_state` | `OperationState` | 操作状态 |

### 动作 (Actions)

| 名称 | 类型 | 说明 |
|------|------|------|
| `/xmate3/cobot/move_append` | `MoveAppend` | 缓存并批量发送非实时运动指令 |

### 控制器

| 控制器名 | 类型 | 说明 |
|---------|------|------|
| `joint_state_broadcaster` | JointStateBroadcaster | 关节状态广播 |
| `joint_trajectory_controller` | JointTrajectoryController | 关节轨迹控制 |
| `xmate_effort_controller` | JointGroupEffortController | 力控制 |
| `forward_position_controller` | ForwardCommandController | 前向位置控制 |

---

## 常见问题

### Q: 如何启动仿真？
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

### Q: 示例程序无法连接？
确保仿真已启动并且机器人已spawn：
```bash
ros2 service list | grep xmate3
```
### Q: 运行示例后机械臂不动？
仿真启动初期可能尚未完成插件初始化，MoveAppend 动作服务器还未准备好。
在运行示例前等待控制台提示“仿真环境启动完成”，或查询动作列表：
```bash
ros2 action list | grep move_append
```
SDK 从 2.1.0 版本起会在 `moveStart()` 中自动等待服务器最多 10 秒，并在失败时输出错误信息。
### Q: 控制器无法加载？
检查控制器管理器：
```bash
ros2 control list_controllers
```

更多常见问题请参考 [快速入门指南](docs/QUICKSTART.md#常见问题)。

---

## 相关资源

- [快速入门指南](docs/QUICKSTART.md)
- [示例程序说明](examples/README.md)
- xCore SDK C++ 使用手册
- [ROKAE 官网](https://www.rokae.com)

---

## 许可证

Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

**版本**: 2.1.0 | **最后更新**: 2024
