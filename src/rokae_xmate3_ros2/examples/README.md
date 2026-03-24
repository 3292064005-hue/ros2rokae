# 示例程序说明

> 本目录包含按 xCore SDK C++ 使用手册与官方 xMate3 风格整理的示例程序

## 目录结构

```text
examples/
├── cpp/
│   ├── example_common.hpp
│   ├── 01_basic_connect.cpp
│   ├── 02_joint_cartesian_read.cpp
│   ├── 03_kinematics.cpp
│   ├── 04_motion_basic.cpp
│   ├── 05_motion_cartesian.cpp
│   ├── 06_io_control.cpp
│   ├── 07_safety_collision.cpp
│   ├── 08_path_record_replay.cpp
│   ├── 09_advanced_sdk_compat.cpp
│   ├── 10_sdk_workflow_xmate3.cpp
│   ├── 11_move_advanced_xmate3.cpp
│   ├── 12_state_stream_threaded.cpp
│   ├── 13_rl_project_workflow.cpp
│   ├── 14_model_extended.cpp
│   ├── 15_move_queue_and_events.cpp
│   ├── 16_registers_and_runtime_options.cpp
│   ├── 17_state_stream_cache.cpp
│   ├── 18_toolset_and_calibration.cpp
│   ├── 19_diagnostics_and_wrench.cpp
│   ├── 20_rt_joint_position.cpp
│   ├── 21_rt_move_commands.cpp
│   ├── 22_rt_joint_impedance.cpp
│   ├── 23_rt_cartesian_impedance.cpp
│   ├── 24_rt_follow_position.cpp
│   ├── 25_rt_s_line.cpp
│   ├── 26_rt_torque_control.cpp
│   └── 99_complete_demo.cpp
└── README.md
```

## 示例索引

能力分级说明：
- `已对齐`：示例调用路径和默认仿真语义已经收口
- `近似实现`：示例可运行，但依赖仿真近似模型或近似时间律
- `实验性`：主要用于验证链路与接口形状，不承诺真机级语义
- `不支持`：当前示例或接口明确未提供

全量 examples harness 分组说明：
- `严格通过组`：`01-08`、`10-19`、`99`。这些示例在 headless Gazebo 下必须完成主流程并退出 `0`。
- `实验性 RT 组`：`20-26`，以及 `09` 里的 RT 子流程。若当前仿真 backend 不支持某个 RT 子能力，会打印 `simulation-only` / `skipped` 并退出 `0`。
- `模型近似组`：`03`、`09`、`14`、`19`、`24`、`26`。这些示例默认接受 KDL/URDF-backed 运动学 + approximate dynamics。

### 第 4.3 节
- `01_basic_connect.cpp`: 连接机器人、获取信息、电源/模式切换
- `02_joint_cartesian_read.cpp`: 读取关节位置、速度、力矩、笛卡尔位姿
- `03_kinematics.cpp`: 正逆运动学、雅可比、模型计算
  说明：`xMateModel` 当前是 KDL/URDF-backed 运动学 + approximate dynamics（近似实现）
- `15_move_queue_and_events.cpp`: 运动队列、事件回调、控制器日志
- `17_state_stream_cache.cpp`: 状态缓存、字段轮询、异步采样

### 第 4.4 节
- `04_motion_basic.cpp`: `MoveAbsJ`
- `05_motion_cartesian.cpp`: `MoveJ` / `MoveL` / `MoveC`
- `11_move_advanced_xmate3.cpp`: `confData`、默认轴配置、偏移、在线调速、`MoveSP`

### 第 4.5 节
- `20_rt_joint_position.cpp`: RT 关节位置
- `21_rt_move_commands.cpp`: RT `MoveJ` / `MoveL` / `MoveC`
- `22_rt_joint_impedance.cpp`: RT 关节阻抗
- `23_rt_cartesian_impedance.cpp`: RT 笛卡尔阻抗
- `24_rt_follow_position.cpp`: RT 跟随位置
- `25_rt_s_line.cpp`: RT S 线轨迹
- `26_rt_torque_control.cpp`: RT 力矩控制 smoke 示例
  说明：4.5 节所有示例均属于实验性仿真 RT 语义，不代表真机控制柜 1ms 契约

### 第 4.6 节
- `06_io_control.cpp`: DI/DO/AI/AO 读写、仿真模式
- `16_registers_and_runtime_options.cpp`: typed 寄存器、xPanel、电压与运行时选项

### 第 4.7 节
- `13_rl_project_workflow.cpp`: RL 工程查询、加载、运行控制

### 第 4.8 节
- `07_safety_collision.cpp`: 碰撞检测、软限位、拖动示教
- `08_path_record_replay.cpp`: 路径录制、保存、回放
  说明：回放速率通过时间轴重参数化近似原始速率，不是控制柜级保证
- `09_advanced_sdk_compat.cpp`: 寄存器、RT、RL、动力学、奇异规避等高级能力
- `18_toolset_and_calibration.cpp`: 工具、工件与坐标系标定
- `19_diagnostics_and_wrench.cpp`: 末端力矩、奇异规避、诊断接口

### 官方风格补充
- `10_sdk_workflow_xmate3.cpp`
- `12_state_stream_threaded.cpp`
- `14_model_extended.cpp`
  说明：展示 KDL-backed Jacobian / cartesian velocity / approximate acceleration 映射（近似实现）
- `99_complete_demo.cpp`

## 使用方法

### 编译
```bash
cd ~/ros2_ws0
colcon build --packages-select rokae_xmate3_ros2
source install/setup.bash
```

### 运行
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
ros2 run rokae_xmate3_ros2 example_01_basic_connect
```

兼容启动名 `xmate3_simulation.launch.py` 和 `xmate3_gazebo.launch.py` 仍可用。

### 全量 examples harness
```bash
cd ~/ros2_ws0
colcon build --packages-select rokae_xmate3_ros2
cd build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_FULL_EXAMPLES_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
ctest -R gazebo_examples_full --output-on-failure
```

默认 `ctest` 不会包含这条全量 harness。

backend mode smoke（非默认，覆盖 `effort / jtc / hybrid`）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_BACKEND_MODE_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
ctest -L gazebo_integration_modes --output-on-failure
```

## 学习路径建议

### 入门
1. `99_complete_demo.cpp`
2. `01_basic_connect.cpp`
3. `04_motion_basic.cpp`
4. `05_motion_cartesian.cpp`

### 进阶
1. `03_kinematics.cpp`
2. `11_move_advanced_xmate3.cpp`
3. `15_move_queue_and_events.cpp`
4. `16_registers_and_runtime_options.cpp`
5. `17_state_stream_cache.cpp`
6. `18_toolset_and_calibration.cpp`
7. `19_diagnostics_and_wrench.cpp`
8. `13_rl_project_workflow.cpp`

### RT 能力
1. `20_rt_joint_position.cpp`
2. `21_rt_move_commands.cpp`
3. `22_rt_joint_impedance.cpp`
4. `23_rt_cartesian_impedance.cpp`
5. `24_rt_follow_position.cpp`
6. `25_rt_s_line.cpp`
7. `26_rt_torque_control.cpp`

> 说明：20-26 在 Gazebo 中保留 RT API 的调用形状，但语义是 simulation-only / approximate，属于实验性能力。
> 说明：`25_rt_s_line.cpp` 使用 shared quintic retimer；笛卡尔 `GenerateSTrajectory` 当前是基于路径弧长与 seeded IK 的近似实现。
> 说明：`08_path_record_replay.cpp` 的 `replayPath(name, rate)` 采用时间轴重参数化近似原始速率。

## 注意事项
1. 所有示例都需要先启动仿真。
2. 示例包含完整的 `std::error_code` 处理。
3. 本包聚焦 xMate3 六轴 Gazebo 仿真，不等价覆盖真机与其他机型能力。
4. `26_rt_torque_control.cpp` 在 Gazebo 中主要用于验证接口与控制链路。
5. `xMateModel` / `03_kinematics.cpp` / `14_model_extended.cpp` 的动力学结果属于可解释的仿真近似，不等价于完整刚体动力学求解。
6. `simulation.launch.py` 默认使用 `backend_mode:=hybrid`；若只验证 JTC，可显式传 `backend_mode:=jtc enable_xcore_plugin:=false`。

## 回归分层
1. 默认 `ctest`：9 个 unit tests + `gazebo_sdk_regression` + `gazebo_examples_smoke` + `gazebo_alias_smoke`
2. `gazebo_examples_full`：非默认，全量 `27/27` examples 可运行性回归
3. `gazebo_integration_modes`：非默认，覆盖 `effort-only / jtc-only / hybrid`
4. `gazebo_teardown_quality`：非默认，覆盖 `prepare_shutdown` 契约、phase 推进、`safe_to_delete` 与优雅停机路径
5. `gazebo_teardown_quality_repeat`：非默认，连续跑 10 次 teardown 路径，专门收 shutdown 竞态

源码归档发布建议只走经过 `package_verified_source_archive` 校验的候选 zip，不要直接上传未验证的源码包。
候选包默认只包含 package root 内容，并从工作区根附带 `.gitignore` 与 `colcon_defaults.yaml` 两个 sidecar。
5. strict Gazebo 集：非默认，覆盖长路径、长时间、blend/lookahead、replay fidelity 与 ownership stability

发布源码归档前，建议执行：

```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_SOURCE_ARCHIVE=/path/to/src.zip /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . --target verify_source_archive
```

## 更多资源
- [快速入门指南](../docs/QUICKSTART.md)
- [主 README](../README.md)
- [示例源码目录](./cpp)
