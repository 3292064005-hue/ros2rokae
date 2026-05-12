# Quickstart

Status: Active
Audience: first-time users and acceptance runners
Purpose: shortest path to build, launch, smoke, and diagnose the xMateER3 public lane

## Scope

This guide covers the xMateER3 six-axis public compatibility lane, canonical launch, public examples, and the ROS2/Gazebo-backed install-facing compatibility lane.

It does not cover calibration, RL, generic IO/register parity, or internal/backend expert paths.

## Environment

Target baseline:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- initialized `rosdep` database

Install common dependencies:

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

Preflight after install:

```bash
ROKAE_PKG_PREFIX="$(ros2 pkg prefix rokae_xmate3_ros2)"
ROKAE_TOOLS="${ROKAE_PKG_PREFIX}/share/rokae_xmate3_ros2/tools"
"${ROKAE_TOOLS}/check_target_environment.sh"
```

## Build

Source workspace path used by the maintained gates:

```bash
cd /media/chen/New/plform/project/ros2_ws0
source /opt/ros/humble/setup.bash
bash src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

Release/source-tree validation is documented in [../release/BUILD_RELEASE.md](../release/BUILD_RELEASE.md). The full gate runs with `ROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF` and is not a real hardware/实机 validation.

## Launch

Canonical default:

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

Equivalent public wrapper:

```bash
ros2 launch rokae_xmate3_ros2 xmate_er3_public.launch.py
```

Useful smoke variants:

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py \
  launch_profile:=public_xmate_er3_jtc gui:=false rviz:=false

ros2 launch rokae_xmate3_ros2 simulation.launch.py \
  launch_profile:=public_xmate_er3_headless_sdk_smoke gui:=false rviz:=false
```

Notes:

- `simulation.launch.py` is the canonical entry.
- `xmate3_simulation.launch.py` and `xmate3_gazebo.launch.py` are compatibility aliases.
- Unknown `launch_profile` values fail fast.
- `public_xmate_er3_jtc` defaults to `require_runtime_readiness:=true`, waiting for controller manager, active JTC, and the FollowJointTrajectory action server.

## Verify

Run static contracts:

```bash
bash src/rokae_xmate3_ros2/tools/run_static_sanity.sh
```

Run source-tree quick/semantic gate:

```bash
ROKAE_IGNORE_ENV_LOCK=1 bash src/rokae_xmate3_ros2/tools/run_full_source_tree_build_gate.sh \
  /media/chen/New/plform/project/ros2_ws0 --ctest-labels 'quick_gate;semantic_gate'
```

Use the environment-lock version without `ROKAE_IGNORE_ENV_LOCK=1` for release evidence.

Run launch, JTC main-chain, headless SDK, and explicit experimental opt-in smoke:

```bash
bash src/rokae_xmate3_ros2/tools/run_launch_smoke.sh /media/chen/New/plform/project/ros2_ws0
bash src/rokae_xmate3_ros2/tools/run_main_chain_smoke.sh /media/chen/New/plform/project/ros2_ws0
bash src/rokae_xmate3_ros2/tools/run_headless_sdk_smoke.sh /media/chen/New/plform/project/ros2_ws0
bash src/rokae_xmate3_ros2/tools/run_experimental_opt_in_smoke.sh /media/chen/New/plform/project/ros2_ws0
```

`run_main_chain_smoke.sh` proves the default public chain: connect/disconnect, power, operate mode, toolset, soft limit, FK/IK, NRT motion, wrench diagnostics, runtime snapshot, and absence of non-target IO/RL/register services.

Runtime diagnostics thresholds can be inspected and recalibrated with `share/rokae_xmate3_ros2/tools/derive_runtime_diag_gate.py`.

## Public Examples

```bash
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_15_move_queue_and_events
ros2 run rokae_xmate3_ros2 example_99_complete_demo
```

More examples are listed in [EXAMPLES.md](EXAMPLES.md).

## Install-Tree C++ Consumer

Core-only install-facing 主消费者:

```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

Public headers:

```cpp
#include <rokae/robot.h>
#include <rokae/model.h>
#include <rokae/motion_control_rt.h>
#include <rokae/planner.h>
#include <rokae/data_types.h>
#include <rokae/utility.h>
```

Do not treat `rokae/sdk_shim*.hpp` as an install-facing public contract.

Robot session, RT control, or ROS bridge consumers must request runtime components explicitly:

```cmake
find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

`xCoreSDK::xCoreSDK_ros_bridge` is the ROS bridge target. `xCoreSDK::xCoreSDK_static` is retained for compatibility, but the primary consumer remains `xCoreSDK::xCoreSDK_core`.

Install-tree consumer tests are split:

- `test/compat/install_tree_core_only/`: `find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)` and `xCoreSDK::xCoreSDK_core`.
- `test/compat/install_tree_runtime_components/`: explicit shared/static/ros_bridge runtime component path.
- `test/compat/install_tree/`: aggregate regression harness.

## Behavioral Reminders

1. `MoveAppend` returns success when the queue accepts the command.
2. `moveStart()` starts staged NRT execution.
3. `stop()` pauses; `moveReset()` clears queued work.
4. `replayPath()` is an experimental immediate-submit side-lane.
5. Default public profile does not register path record/replay, drag, or RT ROS services.
6. Headless SDK smoke uses the same public semantics without Gazebo GUI/JTC.

## Troubleshooting Map

- Environment fails before build: [../release/ENVIRONMENT_LOCK.md](../release/ENVIRONMENT_LOCK.md)
- Build/release gates: [../release/BUILD_RELEASE.md](../release/BUILD_RELEASE.md)
- Runtime profile or query authority: [RUNTIME_PROFILES.md](RUNTIME_PROFILES.md)
- Public contract: [COMPATIBILITY.md](COMPATIBILITY.md)
- Runtime state machine: [../reference/RUNTIME_STATE_MACHINE.md](../reference/RUNTIME_STATE_MACHINE.md)
- Acceptance layers: [../release/ACCEPTANCE_LAYERS.md](../release/ACCEPTANCE_LAYERS.md)

Applications that include `rokae_xmate3_ros2/model_facade.hpp` receive only the backend-neutral provider contract. Concrete Gazebo provider headers are not part of the public facade include chain.
