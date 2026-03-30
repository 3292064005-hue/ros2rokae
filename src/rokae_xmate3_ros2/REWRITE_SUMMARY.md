# Rewrite Summary

This rewrite pass converts the package from a few oversized translation units into a staged architecture that keeps the public ROS/SDK surface stable while shrinking internal coupling.

## What changed in this pass

### 1. Runtime facade layer was decomposed
The old `src/runtime/service_facade.cpp` was split into:
- `src/runtime/service_facade_utils.{hpp,cpp}`
- `src/runtime/control_facade.cpp`
- `src/runtime/query_facade.cpp`
- `src/runtime/io_program_facade.cpp`
- `src/runtime/path_facade.cpp`

This isolates control/query/IO/path responsibilities and removes the single 1300+ line implementation bottleneck.

### 2. ROS binding registration was decomposed
The old `src/runtime/ros_bindings.cpp` was split into:
- `src/runtime/ros_service_factory.hpp`
- `src/runtime/ros_service_registry.cpp`
- `src/runtime/compatibility_alias_registry.cpp`
- `src/runtime/move_append_action_registry.cpp`
- `src/runtime/ros_bindings.cpp` (constructor/orchestration only)

This separates service wiring, compatibility aliases, and MoveAppend action execution.

### 3. Runtime state persistence layer was decomposed
The old `src/runtime/runtime_state.cpp` was split into:
- `src/runtime/runtime_state_utils.{hpp,cpp}`
- `src/runtime/session_state.cpp`
- `src/runtime/motion_options_state.cpp`
- `src/runtime/tooling_state.cpp`
- `src/runtime/data_store_state.cpp`
- `src/runtime/program_state.cpp`
- `src/runtime/diagnostics_state.cpp`

`runtime_state.hpp` remains the umbrella contract, so external includes do not break.

### 4. SDK wrapper motion layer was decomposed
The old monolithic `src/sdk/robot_motion.cpp` was split into:
- `src/sdk/robot_motion.cpp`
- `src/sdk/robot_io.cpp`
- `src/sdk/robot_path.cpp`
- `src/sdk/robot_motion_dispatch.cpp`

This cleanly separates non-real-time motion control, IO/simulation APIs, drag/path APIs, and MoveAppend dispatch/cache plumbing.

### 5. Build graph was updated without changing exported target names
`CMakeLists.txt` now builds the same exported package artifacts through the new internal translation units, preserving:
- `${PROJECT_NAME}_runtime_core`
- `${PROJECT_NAME}_sdk`
- `xcore_controller_gazebo_plugin`
- all example targets

This keeps launch files, tests, install rules, and downstream linkage stable.

## Intent of the rewrite

The codebase is now materially closer to the target architecture:
- plugin = lifecycle shell
- runtime context = state aggregate root
- runtime facade layer = domain-facing service logic
- ROS bindings = registration/orchestration shell
- SDK wrapper = transport + API layer, not a single motion megafile

## Remaining large hotspots

The following files are still structurally important and are the next recommended split points if another pass is desired:
- `src/gazebo/xcore_controller_gazebo_plugin.cpp`
- `src/runtime/motion_runtime.cpp`
- `src/runtime/planner_core.cpp`
- `src/sdk/robot_clients.cpp`

Those were intentionally left for a later pass because they couple lifecycle, backend ownership, or client transport behavior and therefore require stricter regression verification.
