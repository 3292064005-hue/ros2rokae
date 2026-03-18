# 快速入门指南

> 本文档帮助您快速上手 xMate3 ROS2 仿真环境

---

## 目录
1. [环境准备](#1-环境准备)
2. [编译工作空间](#2-编译工作空间)
3. [启动仿真](#3-启动仿真)
4. [运行示例](#4-运行示例)
5. [下一步](#5-下一步)

---

## 1. 环境准备

### 1.1 系统要求
- **操作系统**: Ubuntu 22.04 LTS
- **ROS2版本**: Humble Hawksbill
- **Gazebo版本**: 11

### 1.2 依赖安装
```bash
# ROS2 Humble (如果未安装)
sudo apt install ros-humble-desktop-full

# Gazebo 和 ROS2 Control
sudo apt install \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  python3-numpy \
  python3-lxml

# 其他依赖
sudo apt install libeigen3-dev
```

> `gazebo_ros/spawn_entity.py` 依赖 `python3-numpy` 和 `python3-lxml`。如果启动仿真时报 `ModuleNotFoundError`，先安装上面的包再重试。当前 `simulation.launch.py` 会固定使用系统 Python 调用 `spawn_entity.py`，避免 conda/miniconda 破坏 ROS Humble 的 Python ABI。

---

## 2. 编译工作空间

### 2.1 进入工作空间
```bash
cd ~/ros2_ws0
```

### 2.2 编译
```bash
# 仅编译此功能包
colcon build --packages-select rokae_xmate3_ros2 --symlink-install

# 或编译整个工作空间
colcon build --symlink-install
```

### 2.3 设置环境
```bash
source install/setup.bash
```

> 建议将以上命令添加到 `~/.bashrc` 中：
> ```bash
> echo "source ~/ros2_ws0/install/setup.bash" >> ~/.bashrc
> ```

---

## 3. 启动仿真

### 3.1 启动完整仿真环境
```bash
ros2 launch rokae_xmate3_ros2 xmate3_simulation.launch.py
```

这将启动：
- Gazebo 仿真器
- xMate3 机器人模型
- 控制器管理器
- 关节状态广播器
- 关节轨迹控制器
- RViz2 可视化

### 3.2 仅启动 RViz (查看模型)
```bash
ros2 launch rokae_xmate3_ros2 rviz_only.launch.py
```

### 3.3 Launch 参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gui` | `true` | 是否启动Gazebo GUI |
| `rviz` | `true` | 是否启动RViz |
| `headless` | `false` | 无头模式 |
| `world` | `empty.world` | Gazebo世界文件 |

---

## 4. 运行示例

### 4.1 综合演示 (推荐新手)
```bash
ros2 run rokae_xmate3_ros2 example_99_complete_demo
```

### 4.2 按章节学习示例

所有示例程序都严格按照 **xCore SDK C++ 使用手册** 的章节组织：

| 示例 | 对应手册章节 | 功能说明 |
|------|-------------|---------|
| `example_01_basic_connect` | 4.3 | 机器人基本连接与信息查询 |
| `example_02_joint_cartesian_read` | 4.3 | 关节位置与笛卡尔位姿读取 |
| `example_03_kinematics` | 4.3 | 运动学计算 (FK/IK) |
| `example_04_motion_basic` | 4.4 | 基础运动控制 (MoveAbsJ) |
| `example_05_motion_cartesian` | 4.4 | 笛卡尔空间运动 (MoveJ/MoveL/MoveC) |
| `example_06_io_control` | 4.6 | IO 控制 (DI/DO/AI/AO) |
| `example_07_safety_collision` | 4.3/4.8 | 安全与碰撞检测 |
| `example_08_path_record_replay` | 4.8 | 路径录制与回放 |
| `example_09_advanced_sdk_compat` | 4.5/4.7/4.8 | RT 控制 / RL / 寄存器 / 动力学 / 奇异规避 |
| `example_10_sdk_workflow_xmate3` | 4.3/4.4/4.8 | 官方 SDK 工作流映射 |
| `example_11_move_advanced_xmate3` | 4.4 | `confData`、偏移、默认轴配置、在线调速、`MoveSP` |
| `example_12_state_stream_threaded` | 4.3 | 多线程状态流读取 |
| `example_13_rl_project_workflow` | 4.7 | RL 工程查询、加载、运行控制 |
| `example_14_model_extended` | 8.3.8 | 扩展模型计算 |
| `example_20_rt_joint_position` | 4.5 | RT 关节位置控制 |
| `example_21_rt_move_commands` | 4.5 | RT `MoveJ` / `MoveL` / `MoveC` |
| `example_22_rt_joint_impedance` | 4.5 | RT 关节阻抗控制 |
| `example_23_rt_cartesian_impedance` | 4.5 | RT 笛卡尔阻抗控制 |
| `example_24_rt_follow_position` | 4.5 | RT 跟随位置 |
| `example_25_rt_s_line` | 4.5 / 8.3.7 | RT S 线轨迹示例 |
| `example_26_rt_torque_control` | 4.5 | RT 力矩控制 smoke 示例 |
| `example_99_complete_demo` | - | 综合演示 |

### 4.3 运行单个示例
```bash
# 确保仿真已启动
ros2 launch rokae_xmate3_ros2 xmate3_simulation.launch.py

# 在新终端中运行示例
source ~/ros2_ws0/install/setup.bash
ros2 run rokae_xmate3_ros2 example_01_basic_connect
```

---

## 5. 下一步

- 阅读 [README.md](../README.md) 了解完整功能
- 查看 [examples/cpp/](../examples/cpp/) 目录下的示例源代码
- 参考 xCore SDK C++ 使用手册了解API详情

---

## 常见问题

### Q: 编译失败
A: 确保所有依赖已安装，并且ROS2环境已正确设置：
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws0
rm -rf build/ install/ log/
colcon build
```

### Q: Gazebo 无法启动
A: 检查环境变量：
```bash
echo $GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$HOME/ros2_ws0/install/rokae_xmate3_ros2/lib:$GAZEBO_PLUGIN_PATH
```

### Q: `spawn_entity.py` 报 `ModuleNotFoundError`
A: 安装 Python 运行依赖后重试：
```bash
sudo apt install python3-numpy python3-lxml
```

### Q: 控制器加载失败
A: 确保 `/controller_manager` 节点正在运行：
```bash
ros2 node list
ros2 control list_controllers
```

### Q: 示例程序报错 "连接失败"
A: 确保仿真已启动并且机器人spawn成功：
```bash
ros2 service list | grep xmate3
```

---

## 获取帮助

- 查看 [README.md](../README.md)
- 查阅 xCore SDK C++ 使用手册
- 访问 https://www.rokae.com

---

**文档版本**: 2.1.0 | **最后更新**: 2024
