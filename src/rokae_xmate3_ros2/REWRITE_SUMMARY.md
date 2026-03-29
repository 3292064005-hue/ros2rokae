# Rewrite Summary

This package was rewritten around four concrete corrections:

1. **Single-source spec layer**
   - Added `include/rokae_xmate3_ros2/spec/xmate3_spec.hpp`
   - Centralized xMate3 limits, timing, and DH traceability metadata

2. **Runtime correctness fixes**
   - Fixed RT mode bookkeeping bug in `src/runtime/service_facade.cpp`
   - Removed scattered `-3.14..3.14` fallback soft-limits from runtime/config structs
   - Bound servo tick to the shared spec constant

3. **Manual-aligned model envelope**
   - Updated URDF joint velocity and effort envelopes
   - Updated kinematics/plugin/backend defaults to consume the spec constants
   - Split ros2_control profiles into NRT and SimApprox RT variants

4. **Public API completion**
   - Added wrapper-level `projectInfo / ppToMain / runProject / pauseProject / setProjectRunningOpt`
   - Added `toolsInfo / wobjsInfo / setxPanelVout`
   - Added indexed register helpers and the structured `getEndTorque(...)` overload
   - Reduced SDK shim drift by delegating more behavior back through the wrapper
