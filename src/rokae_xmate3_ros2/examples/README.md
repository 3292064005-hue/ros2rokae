# 示例程序说明

> 本目录包含按 xCore SDK C++ 使用手册 与官方 xMate3 示例风格整理的示例程序

---

## 目录结构

```text
examples/
├── cpp/
│   ├── example_common.hpp             # 示例公共辅助
│   ├── 01_basic_connect.cpp           # 基础连接
│   ├── 02_joint_cartesian_read.cpp    # 状态读取
│   ├── 03_kinematics.cpp              # FK/IK/雅可比
│   ├── 04_motion_basic.cpp            # 基础运动
│   ├── 05_motion_cartesian.cpp        # MoveJ/MoveL/MoveC
│   ├── 06_io_control.cpp              # IO控制
│   ├── 07_safety_collision.cpp        # 安全功能
│   ├── 08_path_record_replay.cpp      # 路径录制回放
│   ├── 09_advanced_sdk_compat.cpp     # 高级兼容能力
│   ├── 10_sdk_workflow_xmate3.cpp     # SDK工作流
│   ├── 11_move_advanced_xmate3.cpp    # 高级运动/confData
│   ├── 12_state_stream_threaded.cpp   # 状态流线程
│   ├── 13_rl_project_workflow.cpp     # RL工程
│   ├── 14_model_extended.cpp          # 扩展模型
│   ├── 20_rt_joint_position.cpp       # RT关节位置
│   ├── 21_rt_move_commands.cpp        # RT MoveJ/L/C
│   ├── 22_rt_joint_impedance.cpp      # RT关节阻抗
│   ├── 23_rt_cartesian_impedance.cpp  # RT笛卡尔阻抗
│   ├── 24_rt_follow_position.cpp      # RT跟随位置
│   ├── 25_rt_s_line.cpp               # RT S线规划
│   ├── 26_rt_torque_control.cpp       # RT力矩控制
│   └── 99_complete_demo.cpp           # 综合演示
└── README.md                          # 本文件
```

---

## 示例索引

### 第4.3节 - 机器人基本操作及信息查询

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 1 | `01_basic_connect.cpp` | 连接机器人、获取信息、电源/模式切换 |
| 2 | `02_joint_cartesian_read.cpp` | 读取关节位置、速度、力矩、笛卡尔位姿 |
| 3 | `03_kinematics.cpp` | 正运动学、逆运动学、雅可比与模型计算 |

### 第4.4节 - 非实时运动控制接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 4 | `04_motion_basic.cpp` | `MoveAbsJ` 轴空间绝对运动 |
| 5 | `05_motion_cartesian.cpp` | `MoveJ` / `MoveL` / `MoveC` 笛卡尔运动 |
| 11 | `11_move_advanced_xmate3.cpp` | `confData`、默认轴配置、偏移、在线调速、`MoveSP` |

### 第4.5节 - 实时控制接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 20 | `20_rt_joint_position.cpp` | RT 关节位置控制 |
| 21 | `21_rt_move_commands.cpp` | RT `MoveJ` / `MoveL` / `MoveC` |
| 22 | `22_rt_joint_impedance.cpp` | RT 关节阻抗控制 |
| 23 | `23_rt_cartesian_impedance.cpp` | RT 笛卡尔阻抗控制 |
| 24 | `24_rt_follow_position.cpp` | RT 跟随位置 |
| 25 | `25_rt_s_line.cpp` | RT S 线轨迹示例 |
| 26 | `26_rt_torque_control.cpp` | RT 力矩控制示例 |

### 第4.6节 - IO与通信接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 6 | `06_io_control.cpp` | DI/DO/AI/AO 读写、仿真模式 |

### 第4.7节 - RL 工程接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 13 | `13_rl_project_workflow.cpp` | RL 工程查询、加载、运行控制 |

### 第4.8节 - 协作机器人专属接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 7 | `07_safety_collision.cpp` | 碰撞检测、软限位、拖动示教 |
| 8 | `08_path_record_replay.cpp` | 路径录制、保存、回放 |
| 9 | `09_advanced_sdk_compat.cpp` | 寄存器、RT、RL、动力学、奇异规避等高级能力 |

### 官方风格补充示例

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 10 | `10_sdk_workflow_xmate3.cpp` | 官方 SDK 工作流映射 |
| 12 | `12_state_stream_threaded.cpp` | 多线程状态流读取 |
| 14 | `14_model_extended.cpp` | 扩展模型能力 |
| 99 | `99_complete_demo.cpp` | 完整功能演示流程 |

---

## 使用方法

### 1. 编译
所有示例在 `colcon build` 时会自动编译：

```bash
cd ~/ros2_ws0
colcon build --packages-select rokae_xmate3_ros2
source install/setup.bash
```

### 2. 运行

```bash
# 先启动仿真
ros2 launch rokae_xmate3_ros2 simulation.launch.py

# 在新终端运行示例
ros2 run rokae_xmate3_ros2 example_01_basic_connect
```

兼容启动名 `xmate3_simulation.launch.py` 和 `xmate3_gazebo.launch.py` 也仍然可用。

### 3. 查看源代码

```bash
# 查看示例 26 的源代码
sed -n '1,220p' ~/ros2_ws0/src/rokae_xmate3_ros2/examples/cpp/26_rt_torque_control.cpp
```

---

## 快速参考

### 编译后的可执行文件

| 可执行文件名 | 说明 |
|------------|------|
| `example_01_basic_connect` | 基础连接 |
| `example_02_joint_cartesian_read` | 关节/位姿读取 |
| `example_03_kinematics` | 运动学计算 |
| `example_04_motion_basic` | 基础运动 |
| `example_05_motion_cartesian` | 笛卡尔运动 |
| `example_06_io_control` | IO 控制 |
| `example_07_safety_collision` | 安全功能 |
| `example_08_path_record_replay` | 路径录制回放 |
| `example_09_advanced_sdk_compat` | 高级兼容能力 |
| `example_10_sdk_workflow_xmate3` | SDK 工作流 |
| `example_11_move_advanced_xmate3` | 高级运动 |
| `example_12_state_stream_threaded` | 状态流线程 |
| `example_13_rl_project_workflow` | RL 工程 |
| `example_14_model_extended` | 扩展模型 |
| `example_20_rt_joint_position` | RT 关节位置 |
| `example_21_rt_move_commands` | RT Move 指令 |
| `example_22_rt_joint_impedance` | RT 关节阻抗 |
| `example_23_rt_cartesian_impedance` | RT 笛卡尔阻抗 |
| `example_24_rt_follow_position` | RT 跟随位置 |
| `example_25_rt_s_line` | RT S 线规划 |
| `example_26_rt_torque_control` | RT 力矩控制 |
| `example_99_complete_demo` | 综合演示 |

---

## 学习路径建议

### 入门
1. `99_complete_demo.cpp` - 先看整体流程
2. `01_basic_connect.cpp` - 学习初始化与连接
3. `04_motion_basic.cpp` - 学习基础运动
4. `05_motion_cartesian.cpp` - 学习笛卡尔运动

### 进阶
1. `03_kinematics.cpp` - 运动学与模型
2. `06_io_control.cpp` - IO 与通信
3. `11_move_advanced_xmate3.cpp` - `confData`、偏移与高级运动
4. `13_rl_project_workflow.cpp` - RL 工程

### RT 能力
1. `20_rt_joint_position.cpp` - RT 基础入口
2. `21_rt_move_commands.cpp` - RT 运动命令
3. `22_rt_joint_impedance.cpp` / `23_rt_cartesian_impedance.cpp` - 阻抗控制
4. `24_rt_follow_position.cpp` / `25_rt_s_line.cpp` - 跟随与轨迹规划
5. `26_rt_torque_control.cpp` - 力矩控制接口形状

---

## 注意事项

1. 所有示例都需要先启动仿真。
2. 示例中包含完整的 `std::error_code` 处理。
3. API 命名与 xCore SDK C++ 使用手册 对齐，并补齐了 `rokae/utility.h` 与官方示例常用工具函数。
4. 本包以 xMate3 六轴 Gazebo 仿真为目标，不等价覆盖官方示例中的 7 轴/工业机器人真机能力。
5. `26_rt_torque_control.cpp` 在 Gazebo 中主要用于验证接口和控制链路，不能等价替代真机力矩控制效果。

---

## 更多资源

- [快速入门指南](../docs/QUICKSTART.md)
- [MoveIt2 使用指南](../docs/MOVEIT2_GUIDE.md)
- [主 README](../README.md)
- [示例源码目录](./cpp)
