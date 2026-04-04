# Environment Lock

This package is expected to run against the following baseline unless explicitly overridden:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11 / gazebo_ros bridge available from the sourced ROS environment
- Python resolved from `ROKAE_PYTHON_EXECUTABLE` or `python3` on `PATH`
- Launch/runtime discovery resolved from sourced package metadata instead of hard-coded `/usr/share/...` paths

The CI gate scripts fail fast when the ROS environment is missing so that environment drift is visible rather than silently tolerated.

## Acceptance Bundle

Use `tools/run_target_env_acceptance.sh` to build the locked container image and execute the package
validation bundle in a target-like Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 environment. If you are
already on the locked target environment, you can run the same bundle directly with
`tools/run_target_env_acceptance.sh --local-target-env`.

When `--launch-smoke` is enabled, the acceptance bundle runs `tools/run_launch_smoke.sh` inside the locked image. That smoke step validates installed-package discovery, xacro expansion, and `ros2 launch --show-args` resolution against the built workspace rather than only syntax-compiling the launch files. Each acceptance run, including failed environment checks or failed gates, emits a machine-readable report through `tools/write_target_env_report.py`; by default `tools/run_target_env_acceptance.sh` stores it under `artifacts/target_env_acceptance/`, and the GitHub workflow uploads the same directory as an artifact.


## Preflight Check

Run `tools/check_target_environment.sh` before `colcon build` if you want an explicit
fail-fast message when the workspace is not sourced against the locked Ubuntu 22.04 / ROS 2
Humble / Gazebo 11 baseline. Set `ROKAE_IGNORE_ENV_LOCK=1` only when you are intentionally
validating a different environment.


The lock check now also validates `rosdep`, `ros2`, and `gazebo_ros` because all three are required by the documented target-environment acceptance bundle (`tools/run_target_env_acceptance.sh`).
