#include "runtime/runtime_control_bridge.hpp"

#include <algorithm>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kServoTickSec = 0.001;
constexpr int kMaxServoSubstepsPerUpdate = 8;

}  // namespace

RuntimeControlBridge::RuntimeControlBridge(RuntimeContext &runtime_context)
    : runtime_context_(runtime_context) {}

ControlTickResult RuntimeControlBridge::tick(BackendInterface &backend,
                                             const RobotSnapshot &snapshot,
                                             double dt) {
  ControlTickResult result;
  auto &session_state = runtime_context_.sessionState();
  auto &motion_runtime = runtime_context_.motionRuntime();

  if (!session_state.powerOn()) {
    servo_accumulator_sec_ = 0.0;
    motion_runtime.stop("power off");
    backend.clearControl();
    result.control_cleared = true;
    if (!backend.brakesLocked()) {
      backend.setBrakeLock(snapshot, true);
    }
    result.brake_locked = backend.brakesLocked();
    result.status = motion_runtime.status();
    return result;
  }

  if (backend.brakesLocked()) {
    backend.setBrakeLock(snapshot, false);
    result.brake_released = !backend.brakesLocked();
  }

  if (session_state.dragMode()) {
    servo_accumulator_sec_ = 0.0;
    backend.clearControl();
    result.control_cleared = true;
    result.status = motion_runtime.status();
    return result;
  }

  const double clamped_dt = std::clamp(dt, 0.0, kServoTickSec * static_cast<double>(kMaxServoSubstepsPerUpdate));
  servo_accumulator_sec_ = std::min(
      servo_accumulator_sec_ + clamped_dt,
      kServoTickSec * static_cast<double>(kMaxServoSubstepsPerUpdate));

  int substeps = 0;
  while (servo_accumulator_sec_ + 1e-12 >= kServoTickSec && substeps < kMaxServoSubstepsPerUpdate) {
    result.status = motion_runtime.tick(backend, kServoTickSec);
    result.runtime_ticked = true;
    servo_accumulator_sec_ -= kServoTickSec;
    ++substeps;
  }

  if (!result.runtime_ticked) {
    result.status = motion_runtime.status();
  }
  return result;
}

}  // namespace rokae_xmate3_ros2::runtime
