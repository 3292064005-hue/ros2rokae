# Examples

> 状态：Active  
> 受众：使用者 / 示例维护者  
> 作用：示例分层与运行方式的唯一主说明  
> 最后校验：2026-04-17

## 1. Rules

- public examples 只服务 xMate 六轴 public compatibility lane
- internal/backend examples 用于 runtime / backend / RT 验证
- public lane 不包含 IO / RL / calibration
- public lane 不公开 experimental RT 示例

## 2. Public examples

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
- `example_18_toolset_only`
- `example_19_diagnostics_and_wrench`
- `example_99_complete_demo`

## 3. Internal/backend examples

- `example_05_motion_cartesian`
- `example_06_io_control`
- `example_08_path_record_replay`
- `example_13_rl_project_workflow`
- `example_16_registers_and_runtime_options`
- `example_20_rt_joint_position`
- `example_21_rt_move_commands`
- `example_22_rt_joint_impedance`
- `example_23_rt_cartesian_impedance`
- `example_24_rt_follow_position`
- `example_25_rt_s_line`
- `example_26_rt_torque_control`
- `example_27_rt_1khz_stress`

## 4. Run

### public lane
```bash
src/rokae_xmate3_ros2/tools/clean_build_env.sh   colcon build --packages-select rokae_xmate3_ros2
source install/setup.bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
ros2 run rokae_xmate3_ros2 example_04_motion_basic
```

### internal/backend lane
```bash
src/rokae_xmate3_ros2/tools/clean_build_env.sh   colcon build --packages-select rokae_xmate3_ros2
source install/setup.bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py service_exposure_profile:=internal_full
ros2 run rokae_xmate3_ros2 example_25_rt_s_line
```

## 5. Behavioral reminders

- `MoveAppend` success means **queue accepted**
- `moveStart()` is the only execution authority
- `stop()` is pause-only
- `moveReset()` drops queued NRT work
