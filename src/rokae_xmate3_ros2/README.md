# ROKAE xMateER3 Public SDK Compatibility Lane

`rokae_xmate3_ros2` provides the xMateER3 six-axis ROS 2 / Gazebo simulation lane and the install-facing `xCoreSDK` compatibility surface.

This public lane only promises public SDK compatibility, simulation-grade runtime behavior, basic state/query services, toolset, model/kinematics, install-tree consumption, and acceptance gates. It does not promise calibration, RL projects, generic IO/register/xPanel parity, or controller-grade hardware RT parity.

本仓 public lane 只承诺公开 SDK 兼容面、仿真级运行时、基础状态查询、toolset、运动学模型、安装树消费和验收契约；不承诺坐标系标定、RL 工程、通用 IO / 寄存器 / xPanel parity。

除上述排除项外，public 仿真验收覆盖 connect/disconnect、power、operate mode、state query、toolset、soft limit、FK/IK、NRT motion queue、wrench diagnostics、runtime snapshot，以及 JTC/headless 两条 public 启动链路。

## Start Here

- Quickstart: [docs/public/QUICKSTART.md](docs/public/QUICKSTART.md)
- Examples: [docs/public/EXAMPLES.md](docs/public/EXAMPLES.md)
- Compatibility boundary: [docs/public/COMPATIBILITY.md](docs/public/COMPATIBILITY.md)
- Runtime profiles: [docs/public/RUNTIME_PROFILES.md](docs/public/RUNTIME_PROFILES.md)
- Kinematics/model: [docs/public/KINEMATICS_AND_MODEL.md](docs/public/KINEMATICS_AND_MODEL.md)
- Architecture: [docs/architecture/ARCHITECTURE.md](docs/architecture/ARCHITECTURE.md)
- Provider boundary: [docs/architecture/PROVIDER_BOUNDARY.md](docs/architecture/PROVIDER_BOUNDARY.md)
- Build/release: [docs/release/BUILD_RELEASE.md](docs/release/BUILD_RELEASE.md)
- Environment lock: [docs/release/ENVIRONMENT_LOCK.md](docs/release/ENVIRONMENT_LOCK.md)
- Acceptance layers: [docs/release/ACCEPTANCE_LAYERS.md](docs/release/ACCEPTANCE_LAYERS.md)
- SDK alignment: [docs/reference/SDK_ALIGNMENT.md](docs/reference/SDK_ALIGNMENT.md)
- Capability matrix: [docs/reference/CAPABILITY_MATRIX.md](docs/reference/CAPABILITY_MATRIX.md)
- Runtime state machine: [docs/reference/RUNTIME_STATE_MACHINE.md](docs/reference/RUNTIME_STATE_MACHINE.md)
- Recorded path schema: [docs/reference/RECORDED_PATH_SCHEMA.md](docs/reference/RECORDED_PATH_SCHEMA.md)

## Build

```bash
cd /media/chen/New/plform/project/ros2_ws0
source /opt/ros/humble/setup.bash
bash src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

The full source/Gazebo build expects Ubuntu 22.04, ROS 2 Humble, Gazebo 11, `colcon`, `ros2`, an initialized `rosdep` database, and `gazebo_ros`.

## Launch

Default public JTC profile:

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py launch_profile:=public_xmate_er3_jtc
```

Headless SDK smoke profile:

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py \
  launch_profile:=public_xmate_er3_headless_sdk_smoke gui:=false rviz:=false
```

`public_xmate_er3_jtc` defaults to `require_runtime_readiness:=true`; launch waits for controller manager, active `joint_trajectory_controller`, and `/joint_trajectory_controller/follow_joint_trajectory`. Disable readiness only for launch/debug inspection, not for Gazebo/JTC proof.

## Verify

```bash
bash src/rokae_xmate3_ros2/tools/run_static_sanity.sh
bash src/rokae_xmate3_ros2/tools/run_launch_smoke.sh /media/chen/New/plform/project/ros2_ws0
bash src/rokae_xmate3_ros2/tools/run_main_chain_smoke.sh /media/chen/New/plform/project/ros2_ws0
bash src/rokae_xmate3_ros2/tools/run_headless_sdk_smoke.sh /media/chen/New/plform/project/ros2_ws0
bash src/rokae_xmate3_ros2/tools/run_experimental_opt_in_smoke.sh /media/chen/New/plform/project/ros2_ws0
```

Full source-tree quick/semantic gate:

```bash
bash src/rokae_xmate3_ros2/tools/run_full_source_tree_build_gate.sh \
  /media/chen/New/plform/project/ros2_ws0 --ctest-labels 'quick_gate;semantic_gate'
```

If this fails before build with `environment-lock: rosdep database is not initialized`, fix local rosdep first. For code-path verification on a known local machine, maintainers may use `ROKAE_IGNORE_ENV_LOCK=1`; release evidence must use the locked environment report accepted by `tools/verify_target_env_acceptance_report.py`.

## Install-Tree SDK Consumption

canonical install-facing identity: `xCoreSDK`

`xCoreSDK::xCoreSDK_core` 是默认 install-facing C++ core SDK 入口:

```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

Runtime/Robot session users must explicitly request runtime components:

```cmake
find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

`xCoreSDK::xCoreSDK_ros_bridge` is the ROS bridge target. Private headers such as `rokae/sdk_shim*.hpp` are implementation details, not the public install contract.

## Behavior Boundary

- `MoveAppend` success means queue accepted; `moveStart()` starts staged NRT execution.
- `stop()` is pause-only; `moveReset()` drops queued NRT work.
- `replayPath()` is an experimental immediate-submit side-lane.
- `GetEndWrench` is the preferred public wrench query; `GetEndEffectorTorque` is legacy.
- `MoveSP`, path record/replay, drag, and RT ROS services require `public_xmate_er3_experimental` or `internal_full`.
- The default public profile rejects experimental `MoveSP` action payloads before queueing.
- Default public smoke also asserts that experimental and non-target IO/RL/register services are absent.
- `public_xmate_er3_headless_sdk_smoke` runs the same public service/action semantics without Gazebo GUI/JTC.
- 当前 runtime 是 simulation-grade，不承诺 controller-grade 实机闭环 parity。
- 坐标系标定类接口只保留兼容签名，返回 `function_not_supported`。

## Contract Anchors

- canonical description artifact: `<build>/generated/urdf/xMateER3.urdf`
- canonical metadata: `<build>/generated/urdf/xMateER3.description.json`
- compatibility alias artifact: `<build>/generated/urdf/xMate3.urdf`
- compatibility alias metadata: `<build>/generated/urdf/xMate3.description.json`
- runtime snapshot service: `/xmate_er3/cobot/get_runtime_state_snapshot`
- compatibility alias policy: default `canonical_only`; use `canonical_plus_compat` only when legacy `/xmate3` names are required
- service contract manifest: `src/runtime/service_contract_manifest.hpp`
- service contract registry: `src/runtime/service_contract_manifest.cpp`

`README.md` is intentionally short. Active documentation lives in `docs/public`, `docs/release`, `docs/reference`, and `docs/architecture`.
