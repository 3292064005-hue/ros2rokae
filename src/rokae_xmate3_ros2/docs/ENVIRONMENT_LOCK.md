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
validation bundle in a target-like Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 environment.
