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
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

## 2.1 安装态 xCoreSDK compatibility package

如果目标是让外部 C++ 工程像官方 SDK 一样按安装包消费，而不是直接依赖源码树，请先执行安装：

```bash
cd ~/ros2_ws0
source /opt/ros/humble/setup.bash
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

随后下游 CMake 工程可直接使用。`xCoreSDK::xCoreSDK_static` 与 `xCoreSDK::xCoreSDK_shared` 现在都是真实 install-facing 目标；官方风格消费建议优先走 static lane：

```cmake
find_package(xCoreSDK CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_static)
```

安装态公共头限定为：

- `rokae/robot.h`
- `rokae/model.h`
- `rokae/motion_control_rt.h`
- `rokae/planner.h`
- `rokae/data_types.h`
- `rokae/utility.h`

`rokae/sdk_shim*.hpp`、`rokae/detail/*` 与 `rokae_xmate3_ros2/*` 不属于官方兼容安装面。

注意：当前 `xCoreSDK` 兼容安装面仍然是 **ROS2/Gazebo-backed** 的。`xCoreSDK::xCoreSDK_static` 与 `xCoreSDK::xCoreSDK_shared` 都会在配置期解析 ROS2/Gazebo 依赖，并在运行期继续依赖相同的 ROS2/Gazebo 动态库与插件环境。解决的是 ABI/安装边界问题，而不是把实现层从 ROS2/Gazebo 完全剥离。

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

说明：`simulation.launch.py` 是规范入口，`xmate3_simulation.launch.py` 与 `xmate3_gazebo.launch.py` 现在仅保留为参数全透传别名入口。非 canonical 模型输入默认被拒绝，只有显式设置 `allow_noncanonical_model:=true` 时才允许开发者旁路。

这会启动：
- Gazebo 仿真器
- xMate3 机器人模型
- `xcore_controller_gazebo_plugin`
- 可选 RViz2 可视化

启动日志会额外打印 capability / diagnostics banner，默认应看到：
- `backend_mode=hybrid`
- `RT 能力级别: experimental`
- `/xmate3/internal/validate_motion`
- `/xmate3/internal/get_runtime_diagnostics`
- `/xmate3/internal/runtime_status`

## 4. 运行示例

### 综合演示
```bash
ros2 run rokae_xmate3_ros2 example_99_complete_demo
```

### 关键示例
```bash
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_10_sdk_workflow_xmate3
ros2 run rokae_xmate3_ros2 example_15_move_queue_and_events
ros2 run rokae_xmate3_ros2 example_25_rt_s_line
```

`example_05_motion_cartesian` 与 `example_08_path_record_replay` 已降级为源码树 internal/backend example；安装态 public compat 示例默认不再依赖 `/xmate3/internal/validate_motion`。

### 运行时诊断
```bash
ros2 service call /xmate3/internal/get_runtime_diagnostics rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}"
ros2 topic echo /xmate3/internal/runtime_status --once
```

兼容别名仍可使用：
```bash
ros2 interface show rokae_xmate3_ros2/srv/GetJointTorques
ros2 interface show rokae_xmate3_ros2/srv/GetEndEffectorTorque
```

### 按章节查看
- `example_01_basic_connect`
- `example_02_joint_cartesian_read`
- `example_03_kinematics`
- `example_04_motion_basic`
- `example_07_safety_collision`
- `example_09_advanced_sdk_compat`
- `example_10_sdk_workflow_xmate3`
- `example_11_move_advanced_xmate3`
- `example_12_state_stream_threaded`
- `example_14_model_extended`
- `example_15_move_queue_and_events`
- `example_17_state_stream_cache`
- `example_18_toolset_only`（toolset-only，不包含标定）
- `example_19_diagnostics_and_wrench`
- `example_20_rt_joint_position`
- `example_21_rt_move_commands`
- `example_22_rt_joint_impedance`
- `example_23_rt_cartesian_impedance`
- `example_24_rt_follow_position`
- `example_25_rt_s_line`
- `example_26_rt_torque_control`

> 说明：`example_20` 到 `example_26` 在 Gazebo 中是 simulated RT facade，用于仿真工作流验证，不承诺真机级 1kHz。
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

### 规划失败时如何排查
优先查看统一失败原因：
- `unreachable_pose`
- `conf_mismatch`
- `soft_limit_violation`
- `near_singularity_rejected`
- `runtime_busy`
- `shutdown_in_progress`

然后执行：
```bash
ros2 service call /xmate3/internal/get_runtime_diagnostics rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}"
```

## 6. 下一步
- 阅读 [README.md](../README.md)
- 查看 [examples/README.md](../examples/README.md)
- 对照 xCore SDK C++ 使用手册逐项练习

默认发布验收：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest --output-on-failure
```

请在锁定的 Ubuntu 22.04 + ROS2 Humble + Gazebo11 目标环境中执行该门禁；当前文档不对非目标环境下的门禁数量或通过数作事实承诺。


- `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES=ON` 可显式安装 internal/backend example 二进制；默认 **OFF**，避免把源码树内部验证入口混入 public 安装面。

> IO / registers / RL / calibration 已移出安装态 public xMate6 contract。对应符号仍保留用于源码兼容，但 install-facing public SDK lane 将返回确定性的 `not_implemented` 错误。
