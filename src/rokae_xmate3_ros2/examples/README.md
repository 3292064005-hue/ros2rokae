# 示例程序说明

> 本目录包含按 xCore SDK C++ 使用手册与当前 xMate6 compatibility lane 整理的示例程序。

## 示例分层

### Public compat examples
这些示例属于安装态兼容入口，目标是让外部工程参考 `rokae/*.h + xCoreSDK::xCoreSDK_static` 的官方风格调用路径；`xCoreSDK::xCoreSDK_shared` 继续保留为动态兼容入口。

注意：当前安装态 compatibility lane 仍然依赖与本包一致的 ROS2/Gazebo 目标环境；示例分层强调的是 **public C++ ABI 头文件面** 与 **internal backend 实现面** 的隔离。

- `01_basic_connect.cpp`
- `02_joint_cartesian_read.cpp`
- `03_kinematics.cpp`
- `04_motion_basic.cpp`
- `07_safety_collision.cpp`
- `09_advanced_sdk_compat.cpp`
- `10_sdk_workflow_xmate3.cpp`
- `11_move_advanced_xmate3.cpp`
- `12_state_stream_threaded.cpp`
- `14_model_extended.cpp`
- `15_move_queue_and_events.cpp`
- `17_state_stream_cache.cpp（状态发送周期已收紧到手册允许的 8 ms）`
- `18_toolset_only.cpp`（toolset-only 示例）
- `19_diagnostics_and_wrench.cpp`
- `20_rt_joint_position.cpp`
- `21_rt_move_commands.cpp`
- `22_rt_joint_impedance.cpp`
- `23_rt_cartesian_impedance.cpp`
- `24_rt_follow_position.cpp`
- `25_rt_s_line.cpp`
- `26_rt_torque_control.cpp`
- `27_rt_1khz_stress.cpp`
- `99_complete_demo.cpp`

### Internal/backend examples
这些示例保留给源码树内的 ROS2/backend 验证，不属于安装态 public compat contract：

- `05_motion_cartesian.cpp`：依赖 `/xmate3/internal/validate_motion` 与 `rclcpp`。
- `06_io_control.cpp`：IO/control facade 验证，不属于 public xMate6 contract。
- `08_path_record_replay.cpp`：路径录制/回放链路（internal/backend only）。
- `13_rl_project_workflow.cpp`：RL 工程链路，不在本轮范围。
- `16_registers_and_runtime_options.cpp`：typed 寄存器、xPanel 与运行时选项验证。

## 能力边界

- 本轮只收口 **xMate 六轴 compatibility lane**。
- 拖动示例现在应先切到 **manual** 并确保机器人处于**下电**状态，再调用 `enableDrag()`；这与官方示例前置保持一致。
- 路径回放示例不再隐式覆盖当前 toolset，上下文不匹配会直接失败。
- `setAvoidSingularity()/getAvoidSingularity()` 在当前 xMate6 compatibility lane 上显式返回 unsupported；官方手册将该能力限定在 xMateCR/xMateSR 机型。
- IO/寄存器示例仅用于 backend/internal 验证；安装态 public xMate6 lane 对这些入口统一返回 deterministic `not_implemented`。
- **不做 RL 语义硬化**，因此 RL 示例被降级为 internal/backend example。
- **不做标定链**，因此 `18_toolset_only.cpp` 只保留 toolset 管理，不再演示 `calibrateFrame()`。
- RT 相关示例保留 API 形状，但在 Gazebo 中仍属于 simulation-only / approximate 语义。
- 实时状态接收示例只使用 SDK 手册允许的周期：`1/2/4/8ms/1s`。

## 使用方法

### Public compat examples（推荐）
```bash
cd ~/ros2_ws0
src/rokae_xmate3_ros2/tools/clean_build_env.sh   colcon build --packages-select rokae_xmate3_ros2
source install/setup.bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
ros2 run rokae_xmate3_ros2 example_01_basic_connect
```

### Internal/backend examples
这些示例需要源码树内 backend/internal contracts 存在，适合开发者验证，不建议作为安装态用户入口：

```bash
ros2 run rokae_xmate3_ros2 example_05_motion_cartesian
ros2 run rokae_xmate3_ros2 example_13_rl_project_workflow
```

## 学习路径建议

### 入门
1. `99_complete_demo.cpp`
2. `01_basic_connect.cpp`
3. `04_motion_basic.cpp`
4. `15_move_queue_and_events.cpp`

### 进阶
1. `03_kinematics.cpp`
2. `11_move_advanced_xmate3.cpp`
3. `15_move_queue_and_events.cpp`
4. install-facing public xMate6 lane excludes IO / register / RL / calibration workflows; those examples remain backend/internal only.
5. `17_state_stream_cache.cpp（状态发送周期已收紧到手册允许的 8 ms）`
6. `18_toolset_only.cpp`
7. `19_diagnostics_and_wrench.cpp`

### RT
1. `20_rt_joint_position.cpp`
2. `21_rt_move_commands.cpp`
3. `22_rt_joint_impedance.cpp`
4. `23_rt_cartesian_impedance.cpp`
5. `24_rt_follow_position.cpp`
6. `25_rt_s_line.cpp`
7. `26_rt_torque_control.cpp`
8. `27_rt_1khz_stress.cpp`

压测脚本（默认 `daemon` + `hard_1khz`，600s）：
```bash
bash src/rokae_xmate3_ros2/tools/run_rt_1khz_stress.sh ~/ros2_ws0 600 5 daemon
```

如果当前主机没有实时调度权限（`CAP_SYS_NICE/CAP_IPC_LOCK`），可先用仿真通道验证链路：
```bash
bash src/rokae_xmate3_ros2/tools/run_rt_1khz_stress.sh ~/ros2_ws0 600 5 simulation
```


- `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES=ON` 可显式安装 internal/backend example 二进制；默认 **OFF**，避免把源码树内部验证入口混入 public 安装面。

## Out-of-scope modules on the public xMate6 lane

The install-facing xMate6 compatibility contract intentionally excludes IO, typed registers, RL workflows, and calibration. Their symbol names remain available for source compatibility, but the public SDK lane returns deterministic `not_implemented` errors instead of advertising them as public guarantees.
