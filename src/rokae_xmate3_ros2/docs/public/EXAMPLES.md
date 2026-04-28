# Examples

> 状态：Active  
> 受众：使用者 / 示例维护者  
> 作用：示例分层与运行方式的唯一主说明  
> 最后校验：2026-04-18

## 1. Rules

- public examples 只服务 xMateER3 六轴 public compatibility lane
- internal/backend examples 用于 runtime / backend / RT 验证
- public lane 不包含 IO / RL / calibration
- public lane 不公开 experimental RT 控制回环示例

## 2. Public examples

- `example_01_basic_connect`
- `example_02_joint_cartesian_read`
- `example_03_kinematics`
- `example_04_motion_basic`
- `example_07_safety_collision`
- `example_08_path_record_replay`
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

### install-facing mirrored tools
安装态公共文档默认使用安装镜像工具入口：
```bash
ROKAE_PKG_PREFIX="$(ros2 pkg prefix rokae_xmate3_ros2)"
ROKAE_TOOLS="${ROKAE_PKG_PREFIX}/share/rokae_xmate3_ros2/tools"
```

### public lane
```bash
"${ROKAE_TOOLS}/clean_build_env.sh" colcon build --packages-select rokae_xmate3_ros2
source install/setup.bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
ros2 run rokae_xmate3_ros2 example_04_motion_basic
```

### internal/backend lane
internal/backend lane 仍要求 maintainer workspace，并通过内部服务暴露配置启动。install-facing 公共文档只保留入口说明，不直接展开 source-tree 命令；具体维护步骤见 [`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md)。

## 5. Behavioral reminders

- `MoveAppend` success means **queue accepted**
- `moveStart()` is the only execution authority
- `stop()` is pause-only
- `moveReset()` drops queued NRT work
- path record/replay asset schema follows [`../reference/RECORDED_PATH_SCHEMA.md`](../reference/RECORDED_PATH_SCHEMA.md)
- acceptance layers and install-facing entrypoints follow [`../release/ACCEPTANCE_LAYERS.md`](../release/ACCEPTANCE_LAYERS.md)

> Source layout: public examples are stored in `examples/cpp/`; internal/backend-only examples are stored in `examples/internal/cpp/` and are only built when `ROKAE_BUILD_INTERNAL_BACKEND_EXAMPLES=ON`.
