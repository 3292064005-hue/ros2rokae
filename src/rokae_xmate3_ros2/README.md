# ROKAE xMate3 ROS2 仿真 SDK

> 基于 xCore SDK C++ API 形状的 ROS2 Gazebo 仿真功能包

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)

## 目录
- [功能特性](#功能特性)
- [文件结构](#文件结构)
- [快速开始](#快速开始)
- [示例程序](#示例程序)
- [API 参考](#api-参考)
- [架构说明](#架构说明)
- [ROS2 接口](#ros2-接口)
- [常见问题](#常见问题)

## 功能特性

### xCore SDK C++ API 兼容支持

状态标记说明：
- `已对齐`：接口形状与 Gazebo 仿真语义都已经收口到默认实现
- `近似实现`：接口可用，但内部是仿真近似，不等价于真机控制器
- `实验性`：接口或语义仍在收口，适合验证链路，不适合当精度保证
- `不支持`：当前版本明确不提供

| 章节 | 功能 | 状态 |
|------|------|------|
| **4.3** | 机器人基本操作及信息查询 | 已对齐 |
| **4.4** | 非实时运动控制接口 | 已对齐 |
| **4.5** | 实时控制接口 | 实验性 |
| **4.6** | IO 与通信接口 | 已对齐 |
| **4.7** | RL 工程接口 | 近似实现 |
| **4.8** | 协作机器人专属接口 | 近似实现 |
| **8.3.7** | 路径规划类 | 近似实现 |
| **8.3.8** | 模型库 | 近似实现 |
| **8.3.9** | 工具函数 | 已对齐 |

完整示例覆盖范围见 [examples/README.md](examples/README.md)。

### 运动能力
- `MoveAbsJ`
- `MoveJ`
- `MoveL`
- `MoveC`
- `MoveCF`
- `MoveSP`
- Jog / 路径录制 / 路径回放
- RT 位置 / 阻抗 / 跟随 / S 线 / 力矩示例（实验性，Gazebo simulated RT facade）

### 运动学与模型
- 正运动学 (FK)
- 逆运动学 (IK)
- 雅可比矩阵（近似实现，默认优先 KDL/URDF 后端）
- 关节/笛卡尔速度与加速度（近似实现）
- 力矩估计与无摩擦力矩（近似实现）

### 兼容扩展
- `rokae/robot.h`、`rokae/data_types.h`、`rokae/model.h`、`rokae/planner.h`、`rokae/motion_control_rt.h`
- `rokae/utility.h` 与官方示例同名工具头
- `/xmate3/io/*` 与 `/xmate3/cobot/*` 双命名空间兼容
- 寄存器、RL、奇异位规避、末端力矩等仿真接口

## 文件结构

```text
rokae_xmate3_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   ├── rokae/
│   └── rokae_xmate3_ros2/
├── src/
│   ├── robot.cpp
│   └── gazebo/
├── examples/
│   ├── README.md
│   └── cpp/
├── launch/
│   ├── simulation.launch.py
│   ├── rviz_only.launch.py
│   ├── xmate3_simulation.launch.py
│   └── xmate3_gazebo.launch.py
├── config/
│   ├── ros2_control.yaml
│   └── xMate3.rviz
├── urdf/
│   ├── xMate3.xacro
│   ├── xMate3.ros2_control.xacro
│   ├── xMate3.urdf
│   └── materials.xacro
├── models/
│   └── rokae_xmate3_ros2/
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
├── worlds/
│   └── empty.world
├── docs/
│   └── QUICKSTART.md
├── msg/
├── srv/
└── action/
```

> `xMate3.xacro` 是主要维护入口，mesh 路径默认走 `models/rokae_xmate3_ros2/meshes/...`。

## 快速开始

### 1. 环境准备

```bash
sudo apt install \
  ros-humble-desktop-full \
  ros-humble-gazebo-ros \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  libeigen3-dev \
  python3-numpy \
  python3-lxml
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

# 可选
ros2 launch rokae_xmate3_ros2 simulation.launch.py gui:=false rviz:=false
ros2 launch rokae_xmate3_ros2 simulation.launch.py backend_mode:=effort
ros2 launch rokae_xmate3_ros2 simulation.launch.py backend_mode:=jtc enable_xcore_plugin:=false
ros2 launch rokae_xmate3_ros2 xmate3_simulation.launch.py
ros2 launch rokae_xmate3_ros2 xmate3_gazebo.launch.py
ros2 launch rokae_xmate3_ros2 rviz_only.launch.py
```

### 4. 运行示例

```bash
ros2 run rokae_xmate3_ros2 example_99_complete_demo
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_25_rt_s_line
```

详细步骤请参考 [docs/QUICKSTART.md](docs/QUICKSTART.md)。

## 示例程序

| 示例 | 对应手册章节 | 功能说明 |
|------|-------------|---------|
| 01 | 4.3 | 机器人基本连接与信息查询 |
| 02 | 4.3 | 关节位置与笛卡尔位姿读取 |
| 03 | 4.3 | 运动学计算 (FK/IK) |
| 04 | 4.4 | 基础运动控制 |
| 05 | 4.4 | 笛卡尔空间运动 |
| 06 | 4.6 | IO 控制 |
| 07 | 4.3/4.8 | 安全与碰撞检测 |
| 08 | 4.8 | 路径录制与回放 |
| 09 | 4.5/4.7/4.8 | RT / RL / 寄存器 / 奇异规避 |
| 10 | 4.3/4.4/4.8 | 官方 SDK 工作流映射 |
| 11 | 4.4 | `confData`、偏移、在线调速、`MoveSP` |
| 12 | 4.3 | 多线程状态流读取 |
| 13 | 4.7 | RL 工程查询、加载、运行控制 |
| 14 | 8.3.8 | 扩展模型计算 |
| 15 | 4.3/4.4 | 运动队列、事件回调、控制器日志 |
| 16 | 4.6 | typed 寄存器、xPanel、运行时选项 |
| 17 | 4.3/4.5 | 状态缓存与异步轮询 |
| 18 | 4.8 | 工具、工件与坐标系标定 |
| 19 | 4.8 | 末端力矩、奇异规避与诊断 |
| 20-26 | 4.5 | RT 控制系列示例（Gazebo simulated RT facade） |
| 99 | - | 综合演示 |

更多说明请参考 [examples/README.md](examples/README.md)。

## API 参考

### 机器人基本操作

```cpp
#include "rokae/robot.h"

rokae::xMateRobot robot;
std::error_code ec;
robot.connectToRobot(ec);
robot.setPowerState(true, ec);
robot.setOperateMode(rokae::OperateMode::automatic, ec);
auto joints = robot.getJointPos(ec);
```

### 运动控制

`setDefaultSpeed()` 使用 `mm/s`，`setDefaultZone()` 使用 `mm`，`adjustSpeedOnline(scale)` 会对当前与后续 NRT 轨迹做仿真重定时。

接口能力分级：
- `GenerateSTrajectory(joint)`：近似实现，shared quintic retimer
- `GenerateSTrajectory(cartesian)`：不支持
- 碰撞检测：近似实现，residual-based 仿真监督，不等价于真机安全功能
- 实时控制：实验性，simulated RT facade，不承诺真机 1kHz/硬实时语义
- 模型库 / 力矩 / 无摩擦力矩：近似实现，基于统一仿真代理项，不是完整刚体动力学库

```cpp
robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
robot.setDefaultSpeed(500, ec);  // mm/s
robot.setDefaultZone(5, ec);      // mm
robot.moveReset(ec);
robot.moveStart(ec);
```

### 运动学计算

```cpp
auto model = robot.model();
auto pose = model.calcFk(rokae::JointPosition(6, 0.0));
auto ik = model.calcIk(pose);
```

`xMateModel` 当前使用 KDL/URDF 运动学后端提供 FK / Jacobian / 速度映射，动力学相关接口仍是统一近似模型而非完整刚体动力学库。

### 启动装配模式

- `backend_mode:=hybrid`：默认模式，同时启用 xCore plugin 与 JTC，由 owner 状态机仲裁执行权
- `backend_mode:=effort`：只启用 xCore plugin / effort runtime
- `backend_mode:=jtc`：只启用 ros2_control / JTC；需同时设 `enable_xcore_plugin:=false`
- `enable_xcore_plugin:=false`：显式关闭 xCore Gazebo plugin 装配

## 架构说明

```text
用户应用/示例
        |
        | ROS2 服务 / Action / 话题
        v
rokae_xmate3_ros2 SDK 封装层
        |
        | ROS2 与 Gazebo 桥接
        v
xcore_controller_gazebo_plugin
        |
        | Gazebo API
        v
Gazebo 11 + xMate3 模型
```

## ROS2 接口

### 服务
- `/xmate3/cobot/connect`
- `/xmate3/cobot/disconnect`
- `/xmate3/cobot/get_joint_pos`
- `/xmate3/cobot/get_cart_posture`
- `/xmate3/cobot/calc_fk`
- `/xmate3/cobot/calc_ik`
- `/xmate3/cobot/move_reset`
- `/xmate3/cobot/move_start`
- `/xmate3/cobot/stop`

### 话题
- `/xmate3/joint_states`
- `/xmate3/cobot/operation_state`

### 动作
- `/xmate3/cobot/move_append`
- `/xmate3/cobot/jog`
- `/xmate3/cobot/replay_path`

## 常见问题

### Q: 如何启动仿真？
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

### Q: 示例程序无法连接？
确认仿真已启动并且机器人已生成：
```bash
ros2 service list | grep xmate3
```

### Q: 运行示例后机械臂不动？
等待控制插件初始化完成后再运行示例：
```bash
ros2 action list | grep move_append
```

### Q: `spawn_entity.py` 报 `ModuleNotFoundError`？
```bash
sudo apt install python3-numpy python3-lxml
```

## 相关资源
- [快速入门指南](docs/QUICKSTART.md)
- [示例程序说明](examples/README.md)
- xCore SDK C++ 使用手册
- [ROKAE 官网](https://www.rokae.com)

## 许可证

Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD.

Licensed under the Apache License, Version 2.0.
