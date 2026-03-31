# ChatGPT Rewrite Summary

## Core implementation changes

- Centralized xMate3 spec accessors in `include/rokae_xmate3_ros2/spec/xmate3_spec.hpp`
- Expanded planner preflight from input validation to policy/risk report generation:
  - stable reject reasons
  - estimated duration
  - branch/singularity/continuity/soft-limit risk
  - recommended stop point
- Extended planner trace propagation into runtime notes
- Added request-level kinematics trace exposure in `xMate3Kinematics`
- Propagated IK trace into `planner_core` and `ValidateMotion`
- Hardened RT prearm checks:
  - capability flag gate
  - mode-specific required RT fields
  - explicit failure statuses
- Hardened RT watchdog:
  - avg/max gap
  - consecutive late cycles
  - stale state count
  - starvation windows
  - last trigger reason
- Extended runtime diagnostics snapshot/message/service publishing with the new RT watchdog fields

## Updated tests

- `test_planner_preflight.cpp`
- `test_rt_hardening.cpp`
- `test_runtime_diagnostics.cpp`
- `test_kinematics_backend.cpp`

## Files modified

- include/rokae_xmate3_ros2/gazebo/kinematics.hpp
- include/rokae_xmate3_ros2/spec/xmate3_spec.hpp
- msg/RuntimeDiagnostics.msg
- src/gazebo/kinematics.cpp
- src/runtime/diagnostics_state.cpp
- src/runtime/diagnostics_state.hpp
- src/runtime/planner_core.cpp
- src/runtime/planner_preflight.cpp
- src/runtime/planner_preflight.hpp
- src/runtime/planner_trace.cpp
- src/runtime/query_diagnostics_service.cpp
- src/runtime/query_kinematics_service.cpp
- src/runtime/rt_prearm_checks.cpp
- src/runtime/rt_prearm_checks.hpp
- src/runtime/rt_watchdog.cpp
- src/runtime/rt_watchdog.hpp
- src/runtime/runtime_control_bridge.cpp
- src/runtime/runtime_publish_bridge.cpp
- src/runtime/runtime_snapshots.hpp
- test/unit/test_kinematics_backend.cpp
- test/unit/test_planner_preflight.cpp
- test/unit/test_rt_hardening.cpp
- test/unit/test_runtime_diagnostics.cpp
