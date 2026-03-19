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

### 第 4.3 节
- `01_basic_connect.cpp`: 连接机器人、获取信息、电源/模式切换
- `02_joint_cartesian_read.cpp`: 读取关节位置、速度、力矩、笛卡尔位姿
- `03_kinematics.cpp`: 正逆运动学、雅可比、模型计算

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

### 第 4.6 节
- `06_io_control.cpp`: DI/DO/AI/AO 读写、仿真模式

### 第 4.7 节
- `13_rl_project_workflow.cpp`: RL 工程查询、加载、运行控制

### 第 4.8 节
- `07_safety_collision.cpp`: 碰撞检测、软限位、拖动示教
- `08_path_record_replay.cpp`: 路径录制、保存、回放
- `09_advanced_sdk_compat.cpp`: 寄存器、RT、RL、动力学、奇异规避等高级能力

### 官方风格补充
- `10_sdk_workflow_xmate3.cpp`
- `12_state_stream_threaded.cpp`
- `14_model_extended.cpp`
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

## 学习路径建议

### 入门
1. `99_complete_demo.cpp`
2. `01_basic_connect.cpp`
3. `04_motion_basic.cpp`
4. `05_motion_cartesian.cpp`

### 进阶
1. `03_kinematics.cpp`
2. `06_io_control.cpp`
3. `11_move_advanced_xmate3.cpp`
4. `13_rl_project_workflow.cpp`

### RT 能力
1. `20_rt_joint_position.cpp`
2. `21_rt_move_commands.cpp`
3. `22_rt_joint_impedance.cpp`
4. `23_rt_cartesian_impedance.cpp`
5. `24_rt_follow_position.cpp`
6. `25_rt_s_line.cpp`
7. `26_rt_torque_control.cpp`

## 注意事项
1. 所有示例都需要先启动仿真。
2. 示例包含完整的 `std::error_code` 处理。
3. 本包聚焦 xMate3 六轴 Gazebo 仿真，不等价覆盖真机与其他机型能力。
4. `26_rt_torque_control.cpp` 在 Gazebo 中主要用于验证接口与控制链路。

## 更多资源
- [快速入门指南](../docs/QUICKSTART.md)
- [主 README](../README.md)
- [示例源码目录](./cpp)
