#include "runtime/runtime_control_bridge.hpp"

namespace rokae_xmate3_ros2::runtime {

RuntimeControlBridge::RuntimeControlBridge(RuntimeContext &runtime_context)
    : runtime_context_(runtime_context) {}

ControlTickResult RuntimeControlBridge::tick(BackendInterface &backend,
                                             const RobotSnapshot &snapshot,
                                             double dt) const {
  ControlTickResult result;
  auto &session_state = runtime_context_.sessionState();
  auto &motion_runtime = runtime_context_.motionRuntime();

  if (!session_state.powerOn()) {
    motion_runtime.stop("power off");
    if (!backend.brakesLocked()) {
      backend.setBrakeLock(snapshot, true);
      result.brake_locked = backend.brakesLocked();
    }
    result.status = motion_runtime.status();
    return result;
  }

  if (backend.brakesLocked()) {
    backend.setBrakeLock(snapshot, false);
    result.brake_released = !backend.brakesLocked();
  }

  if (session_state.dragMode()) {
    backend.clearControl();
    result.control_cleared = true;
    result.status = motion_runtime.status();
    return result;
  }

  result.status = motion_runtime.tick(backend, dt);
  result.runtime_ticked = true;
  return result;
}

}  // namespace rokae_xmate3_ros2::runtime
