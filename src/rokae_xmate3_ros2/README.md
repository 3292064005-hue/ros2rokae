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
- 兼容别名 `/xmate3/cobot/get_joint_torque`、`/xmate3/cobot/get_end_torque` 保留不变，底层类型统一到 `GetJointTorques` / `GetEndEffectorTorque`
- 内建 `/xmate3/internal/validate_motion`、`/xmate3/internal/get_runtime_diagnostics` 和 `/xmate3/internal/runtime_status` 用于预验证与运行时诊断

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
│   ├── sdk/
│   │   ├── robot_internal.hpp
│   │   ├── robot_clients.cpp
│   │   ├── robot_state.cpp
│   │   ├── robot_motion.cpp
│   │   └── robot_model.cpp
│   ├── gazebo/
│   └── runtime/
├── examples/
│   ├── README.md
│   └── cpp/
├── test/
│   ├── unit/
│   ├── harness/
│   └── strict/
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
├── generated/
│   └── xMate3.urdf
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

> `xMate3.xacro` 是主要维护入口，构建期会额外生成一份 `generated/xMate3.urdf` 供 KDL/测试主路径直接加载；mesh 路径默认走 `models/rokae_xmate3_ros2/meshes/...`。
> `src/sdk/` 负责 SDK façade 分层；`src/gazebo/` 聚焦模型与 Gazebo 插件；`src/runtime/` 聚焦规划、状态与执行链；`test/` 按 `unit / harness / strict` 三层组织。

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
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

release build、`ctest` 与源码归档打包建议统一通过 `src/rokae_xmate3_ros2/tools/clean_build_env.sh` 运行。
它会固定 `/usr/bin/python3`，并清理 Conda 相关环境变量，避免影子库路径污染构建或发布结果。
release-grade build/test/package 必须通过这个 wrapper 运行；它只负责清理环境污染，不再向运行时主动注入当前目录或 `build/*`、`install/*` 动态库路径。

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

常用诊断命令：
```bash
ros2 service call /xmate3/internal/get_runtime_diagnostics rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}"
ros2 topic echo /xmate3/internal/runtime_status --once
```

默认发布验收命令：
```bash
cd ~/ros2_ws0
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
cd build/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest --output-on-failure
```

全量 headless examples harness（非默认门禁）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_FULL_EXAMPLES_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -R gazebo_examples_full --output-on-failure
```

backend mode smoke（非默认，覆盖 `effort / jtc / hybrid`）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_BACKEND_MODE_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -L gazebo_integration_modes --output-on-failure
```

teardown 质量回归（非默认，验证 `prepare_shutdown` 契约、phase 推进与优雅停机路径）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -L gazebo_teardown_quality --output-on-failure
```

teardown 重复压力回归（非默认，连续跑 10 次 shutdown 路径收竞态）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -R gazebo_teardown_quality_repeat --output-on-failure
```

源码归档一致性检查（发布 zip 前，确保候选包按 `package root + workspace sidecars` 布局，且包含 `owner_arbiter.hpp`、工作区根 `.gitignore` / `colcon_defaults.yaml` 与 `flange/tool0/tcp/payload` 终局化 xacro）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_SOURCE_ARCHIVE=/path/to/src.zip /home/chen/ros2_ws0/src/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build . --target verify_source_archive
```

生成并校验唯一候选源码归档（推荐的发布出口）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_SOURCE_ARCHIVE_OUTPUT=$PWD/rokae_source_candidate.zip /home/chen/ros2_ws0/src/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build . --target release_candidate
```

禁止直接上传手工打包的 `src.zip`；只允许分发 `package_verified_source_archive` 生成并通过 archive gate 的候选包。
候选包主体固定为 `src/rokae_xmate3_ros2` 的 package root，工作区根只白名单附带 `.gitignore` 和 `colcon_defaults.yaml`。
`Testing/`、工作区根 PDF、`tmp_*` 日志以及缓存/构建产物都会被 archive gate 拒绝。
候选包旁边会同时生成 `.manifest.txt` 与 `.sha256` sidecar，用来核对“外发包就是刚刚验过的那个包”。
manifest 会额外记录 `package.xml` 版本号、git commit hash 和 UTC 生成时间，方便后续追溯候选包来源。

### Release package rule

Only the archive produced by `release_candidate` / `package_verified_source_archive` may be distributed.

Do not distribute:
- workspace snapshots
- manually zipped source trees
- archives containing `Testing/`, `build/`, `log/`, `__pycache__`, `.pyc`, `.pdf` or `tmp_*`

Recommended release flow:

```bash
cd ~/ros2_ws0
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake -S src/rokae_xmate3_ros2 -B build/rokae_release
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build build/rokae_release -j
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest --test-dir build/rokae_release --output-on-failure
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build build/rokae_release --target release_candidate
```

默认 `ctest --output-on-failure` 固定为 16 项门禁：
- 13 个纯单测
- `gazebo_sdk_regression`
- `gazebo_examples_smoke`
- `gazebo_alias_smoke`

teardown 质量与 full examples 业务回归已经拆成两个质量信号：
- `gazebo_examples_full` 只关注 `27/27` examples 业务通过
- `gazebo_teardown_quality` 单独关注 `prepare_shutdown` 返回的统一 contract：`owner / runtime_phase / shutdown_phase / safe_to_delete / safe_to_stop_world`
- `gazebo_teardown_quality_repeat` 单独关注 teardown 路径的重复稳定性

当前 JTC 主链定义为 `position trajectory backend`，不是完整闭环伺服替身。

### Runtime contract headers

公共 runtime contract 头固定从 `include/rokae_xmate3_ros2/runtime/*` 引用：

- `include/rokae_xmate3_ros2/runtime/runtime_contract.hpp`
- `include/rokae_xmate3_ros2/runtime/owner_arbiter.hpp`
- `include/rokae_xmate3_ros2/runtime/shutdown_coordinator.hpp`

`src/runtime/*.hpp` 下保留的同名头只用于兼容旧 include，新代码不要再依赖这些路径。

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
auto joints = robot.jointPos(ec);
```

### 运动控制

`setDefaultSpeed()` 使用 `mm/s`，`setDefaultZone()` 使用 `mm`，`adjustSpeedOnline(scale)` 会对当前与后续 NRT 轨迹做仿真重定时。

接口能力分级：
- `GenerateSTrajectory(joint)`：近似实现，shared quintic retimer
- `GenerateSTrajectory(cartesian)`：近似实现，基于路径弧长与 seeded IK 的笛卡尔近似轨迹
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

统一规划失败原因：
- `unreachable_pose`
- `conf_mismatch`
- `soft_limit_violation`
- `near_singularity_rejected`
- `runtime_busy`
- `shutdown_in_progress`

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
- `backend_mode:=jtc`：只启用 ros2_control / JTC；JTC 当前是 position trajectory backend，`velocity/acceleration` 作为轨迹边界与重定时辅助信息
- `enable_xcore_plugin:=false`：显式关闭 xCore Gazebo plugin 装配

## 架构说明

```text
用户应用/示例
        |
        | SDK facade / ROS2 服务 / Action / 话题
        v
Request adapter / Service facade
        |
        | 几何路径 / lookahead / zone / unified retimer
        v
Motion runtime / Owner arbiter / Supervisor
        |
        | backend dispatch
        v
JTC backend        Effort backend
        |                 |
        +-------- Gazebo plugin --------+
                          |
                          v
               Gazebo 11 + xMate3 URDF/KDL/model facade
```

## ROS2 接口

### 服务
- `/xmate3/cobot/connect`
- `/xmate3/cobot/disconnect`
- `/xmate3/cobot/get_joint_pos`
- `/xmate3/cobot/get_joint_torque`
- `/xmate3/cobot/get_end_torque`
- `/xmate3/cobot/get_cart_posture`
- `/xmate3/cobot/calc_fk`
- `/xmate3/cobot/calc_ik`
- `/xmate3/cobot/move_reset`
- `/xmate3/cobot/move_start`
- `/xmate3/cobot/stop`
- `/xmate3/internal/validate_motion`
- `/xmate3/internal/get_runtime_diagnostics`
- `/xmate3/internal/prepare_shutdown`

### 话题
- `/xmate3/joint_states`
- `/xmate3/cobot/operation_state`
- `/xmate3/internal/runtime_status`

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
