# Examples

Status: Active
Audience: users and example maintainers
Purpose: public/internal example split and the shortest run order

## 1. Rules

- Public examples serve the xMateER3 public compatibility lane.
- Internal/backend examples serve runtime, backend, path, and experimental RT validation.
- The public lane does not include IO, RL, calibration, or experimental RT control-loop examples.

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
- `example_08_path_record_replay`
- `example_06_io_control`
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

Build and source:

```bash
cd /media/chen/New/plform/project/ros2_ws0
source /opt/ros/humble/setup.bash
bash src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

Launch:

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py launch_profile:=public_xmate_er3_jtc
```

Recommended public smoke order:

```bash
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_15_move_queue_and_events
ros2 run rokae_xmate3_ros2 example_19_diagnostics_and_wrench
ros2 run rokae_xmate3_ros2 example_99_complete_demo
```

Internal/backend examples require maintainer workspace configuration and explicit internal or experimental service exposure. They are not part of the default public xMateER3 lane.

## 5. Behavioral Reminders

- `MoveAppend` success means queue accepted.
- `moveStart()` is the execution authority.
- `stop()` is pause-only.
- `moveReset()` drops queued NRT work.
- Path record/replay examples are experimental/internal.
- Acceptance layers and install-facing entrypoints follow [../release/ACCEPTANCE_LAYERS.md](../release/ACCEPTANCE_LAYERS.md).

Source layout: public examples live in `examples/cpp/`; internal/backend examples live in `examples/internal/cpp/` and are only built when `ROKAE_BUILD_INTERNAL_BACKEND_EXAMPLES=ON`.
