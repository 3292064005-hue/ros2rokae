# Pass 9 rewrite summary

## Focus

Pass 9 continues from round2 and moves the codebase closer to the “planning kernel + timing kernel + runtime state eventing” architecture:

1. planner candidate selection is introduced instead of relying on a single implicit execution policy
2. unified retimer gains explicit policy and input validation semantics
3. runtime state now carries explicit last-event information and diagnostics keep a recent runtime event trail
4. runtime-facing summaries expose candidate selection in addition to backend / branch / stop-point metadata

## Core code changes

### Planning pipeline

- Added `src/runtime/planner_pipeline.hpp/.cpp`
- Introduced `PlannerExecutionCandidate` and lightweight candidate scoring/selection helpers
- `planner_core.cpp` now:
  - builds candidate execution policies from preflight risk
  - selects an execution candidate
  - overrides the effective stop-point policy with the selected candidate
  - stores `selected_candidate` and `candidate_summaries` on `MotionPlan`

This means high-risk requests no longer only *report* risk; they can now steer the selected execution policy.

### Runtime-facing planning summaries

- `MotionPlan` now carries:
  - `selected_candidate`
  - `candidate_summaries`
- `trajectory_goal_builder.cpp` includes selected planner candidate and summarized candidate evaluations in the runtime plan summary string

### Unified retimer hardening

- Added `RetimerPolicy`
  - `nominal`
  - `conservative`
  - `online_safe`
- Added `RetimerValidationReport`
- Added `validateUnifiedRetimerInput(...)`
- `RetimerMetadata` now records `policy`
- `describeRetimerMetadata(...)` now serializes the selected policy
- unified retimer entry points now accept an optional `RetimerPolicy`
- `makeUnifiedRetimerConfig(...)` and `makeUnifiedRetimerLimits(...)` now shape limits/step sizing according to policy

This pushes the retimer closer to a real timing kernel rather than a generic helper.

### Runtime event semantics

- `RuntimeEventType` now includes:
  - `planning_rejected`
  - `trajectory_retimed`
  - `watchdog_triggered`
- `RuntimeStatus` now tracks `last_event`
- `runtime_state_machine.cpp` now writes explicit event names into state
- planner rejection now uses `planning_rejected`
- JTC online retiming path emits `trajectory_retimed`

### Diagnostics event trail

- `RuntimeDiagnosticsSnapshot` now tracks:
  - `last_runtime_event`
  - `recent_runtime_events`
- `RuntimeDiagnostics.msg` is extended with the same fields
- diagnostics update logic now keeps a bounded recent runtime event trail
- publish/query bridges expose the new fields

This gives the runtime a small but useful causal trace instead of only a final state string.

## Test additions

### `test_motion_planner_core.cpp`
- Added `HighRiskMoveJSelectsConservativePlannerCandidate`
- Verifies candidate selection and candidate summary exposure

### `test_unified_retimer.cpp`
- Added validation failure coverage for non-finite waypoints
- Added metadata coverage for conservative retimer policy

### `test_runtime_state_machine.cpp`
- Added `PlanningRejectedAndRetimedEventsExposeLastEvent`
- Verifies explicit event naming and planning rejection terminal behavior

### `test_runtime_diagnostics.cpp`
- Added `RecentRuntimeEventsTrackTransitions`
- Verifies diagnostics recent-event tracking

## Practical effect

Pass 9 is not just a field-extension pass.

It moves the project in four concrete ways:

1. planning chooses between candidate execution policies instead of silently inheriting a single one
2. retiming now has policy semantics and validation semantics
3. runtime transitions are more observable and easier to explain
4. diagnostics now preserve a short runtime event history that can help explain how a request reached its current state

## Remaining gap

The largest remaining structural item is still the same:

- `sdk_shim.hpp` is still too large and should eventually be split into domain-specific adapter headers

Also, full ROS2/ament compile + rosidl regeneration still requires a local ROS2 environment with `ament_cmake` and generated interface headers available.
