# 快速入门指南

> 面向 xCore SDK + Gazebo 仿真的最短上手路径

## 1. 环境准备

### 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo 11

### 依赖安装
```bash
sudo apt install \
  ros-humble-desktop-full \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  python3-numpy \
  python3-lxml \
  libeigen3-dev
```

## 2. 编译工作空间

```bash
cd ~/ros2_ws0
source /opt/ros/humble/setup.bash
colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

## 3. 启动仿真

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

常用变体：
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py gui:=false rviz:=false
ros2 launch rokae_xmate3_ros2 xmate3_simulation.launch.py
ros2 launch rokae_xmate3_ros2 xmate3_gazebo.launch.py
ros2 launch rokae_xmate3_ros2 rviz_only.launch.py
```

这会启动：
- Gazebo 仿真器
- xMate3 机器人模型
- `xcore_controller_gazebo_plugin`
- 可选 RViz2 可视化

## 4. 运行示例

### 综合演示
```bash
ros2 run rokae_xmate3_ros2 example_99_complete_demo
```

### 关键示例
```bash
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_05_motion_cartesian
ros2 run rokae_xmate3_ros2 example_10_sdk_workflow_xmate3
ros2 run rokae_xmate3_ros2 example_25_rt_s_line
```

### 按章节查看
- `example_01_basic_connect`
- `example_02_joint_cartesian_read`
- `example_03_kinematics`
- `example_04_motion_basic`
- `example_05_motion_cartesian`
- `example_06_io_control`
- `example_07_safety_collision`
- `example_08_path_record_replay`
- `example_09_advanced_sdk_compat`
- `example_10_sdk_workflow_xmate3`
- `example_11_move_advanced_xmate3`
- `example_12_state_stream_threaded`
- `example_13_rl_project_workflow`
- `example_14_model_extended`
- `example_20_rt_joint_position`
- `example_21_rt_move_commands`
- `example_22_rt_joint_impedance`
- `example_23_rt_cartesian_impedance`
- `example_24_rt_follow_position`
- `example_25_rt_s_line`
- `example_26_rt_torque_control`
- `example_99_complete_demo`

## 5. 常见问题

### 编译失败
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws0
rm -rf build/ install/ log/
colcon build
```

### Gazebo 无法启动
```bash
echo $GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$HOME/ros2_ws0/install/rokae_xmate3_ros2/lib:$GAZEBO_PLUGIN_PATH
```

### `spawn_entity.py` 缺 Python 模块
```bash
sudo apt install python3-numpy python3-lxml
```

### 示例程序连接失败
```bash
ros2 service list | grep xmate3
ros2 action list | grep move_append
```

## 6. 下一步
- 阅读 [README.md](../README.md)
- 查看 [examples/README.md](../examples/README.md)
- 对照 xCore SDK C++ 使用手册逐项练习
