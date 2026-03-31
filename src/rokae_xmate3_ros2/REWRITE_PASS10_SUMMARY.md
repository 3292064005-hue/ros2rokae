# REWRITE PASS 10 SUMMARY

This pass continues from round3 and focuses on turning the runtime from a large-featured simulation facade into a more explainable, extensible platform core.

## What changed

### 1. Plan explanation and degradation tracking
- `MotionPlan` now carries:
  - `selection_policy`
  - `explanation_summary`
  - `degradation_chain`
- `planner_pipeline` candidates now expose score breakdown terms:
  - continuity component
  - singularity component
  - soft-limit component
  - duration component
  - degradation penalty
- `planner_core` now builds a structured explanation summary and degradation chain instead of relying only on free-form notes.
- `trajectory_goal_builder` includes explanation / selection policy / degradation entries in runtime-facing summaries.

### 2. Planning capability catalog
- Added `runtime/planning_capability_service.hpp/.cpp`
- New catalogs describe:
  - kinematics backends
  - retimer policies
  - planner selection policies
- `GetProfileCapabilities.srv` extended to expose these capability matrices to clients.
- `runtime_control_bridge` now publishes a planning capability summary into diagnostics.

### 3. Diagnostics upgraded from raw fields to operational summaries
- `RuntimeDiagnostics.msg` and `RuntimeDiagnosticsSnapshot` now track:
  - `last_plan_summary`
  - `last_selected_candidate`
  - `planning_capability_summary`
  - `event_bus_summary`
  - `runtime_event_count`
  - `planning_rejection_count`
  - `watchdog_trigger_count`
- `RuntimeDiagnosticsState` now:
  - tracks event counters
  - summarizes recent runtime events
  - records the most recent plan summary and selected candidate
- Query/publish bridges now propagate these new fields.

### 4. Begin sdk shim de-monolithization
- Added narrow migration wrapper headers:
  - `rokae/sdk_shim_state.hpp`
  - `rokae/sdk_shim_motion_nrt.hpp`
  - `rokae/sdk_shim_motion_rt.hpp`
  - `rokae/sdk_shim_io.hpp`
  - `rokae/sdk_shim_project_tooling.hpp`
  - `rokae/sdk_shim_diagnostics.hpp`
- The monolithic `sdk_shim.hpp` now documents these narrower entrypoints as the preferred migration direction for new code.

## Tests updated
- `test_motion_planner_core.cpp`
- `test_runtime_diagnostics.cpp`
- `test_runtime_profile_service.cpp`
- `test_runtime_profile_capabilities_contract.cpp`

## Local verification performed in container
Performed syntax-only checks on pure C++ units that do not require the unavailable ROS2/ament/rosidl environment:
- `src/runtime/planner_pipeline.cpp`
- `src/runtime/planning_capability_service.cpp`
- `src/runtime/diagnostics_state.cpp`
- `src/runtime/runtime_state_machine.cpp`

## Remaining limitation
Full-package validation still requires the user's local ROS 2 environment because this container does not provide the full ROS 2/ament/rosidl toolchain needed for:
- `colcon build`
- message/service regeneration
- full unit/smoke/strict regression
