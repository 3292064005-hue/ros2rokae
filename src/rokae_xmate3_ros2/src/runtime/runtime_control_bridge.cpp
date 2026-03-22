#include "runtime/runtime_control_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kServoTickSec = 0.001;
constexpr int kMaxServoSubstepsPerUpdate = 8;
constexpr std::array<double, 6> kCollisionGravityProxy = {0.0, 10.0, 7.0, 1.5, 0.8, 0.2};
constexpr std::array<double, 6> kCollisionRetreatEffort = {18.0, 18.0, 16.0, 8.0, 5.0, 3.0};
constexpr double kDefaultCollisionRetreatDistance = 0.04;

bool exceeds_soft_limit(const RobotSnapshot &snapshot, const SoftLimitSnapshot &soft_limit) {
  if (!soft_limit.enabled) {
    return false;
  }
  for (std::size_t axis = 0; axis < snapshot.joint_position.size(); ++axis) {
    if (snapshot.joint_position[axis] < soft_limit.limits[axis][0] ||
        snapshot.joint_position[axis] > soft_limit.limits[axis][1]) {
      return true;
    }
  }
  return false;
}

double tool_com_norm(const ToolsetSnapshot &toolset) {
  return std::sqrt(toolset.tool_com[0] * toolset.tool_com[0] +
                   toolset.tool_com[1] * toolset.tool_com[1] +
                   toolset.tool_com[2] * toolset.tool_com[2]);
}

ControlCommand make_retreat_command(const RobotSnapshot &snapshot,
                                    double fallback_gain,
                                    const RuntimeControlBridgeConfig &config) {
  ControlCommand command;
  const double clamped_gain = std::clamp(fallback_gain, 0.0, 1.0);
  if (clamped_gain <= 0.0) {
    return command;
  }
  const double retreat_scale =
      std::max(config.collision_retreat_distance, 0.0) / kDefaultCollisionRetreatDistance;
  command.has_effort = true;
  for (std::size_t axis = 0; axis < snapshot.joint_velocity.size(); ++axis) {
    const double velocity = snapshot.joint_velocity[axis];
    if (std::fabs(velocity) < 1e-3) {
      command.effort[axis] = 0.0;
      continue;
    }
    command.effort[axis] =
        -std::copysign(kCollisionRetreatEffort[axis] * clamped_gain * retreat_scale, velocity);
  }
  return command;
}

bool detect_collision_event(const RobotSnapshot &snapshot,
                            const CollisionDetectionSnapshot &collision_detection,
                            const ToolsetSnapshot &toolset,
                            const RuntimeControlBridgeConfig &config,
                            std::size_t &axis_index,
                            std::string &message) {
  if (!collision_detection.enabled) {
    return false;
  }

  const double com_norm = tool_com_norm(toolset);
  for (std::size_t axis = 0; axis < snapshot.joint_torque.size(); ++axis) {
    const double expected_torque =
        std::sin(snapshot.joint_position[axis]) *
            (kCollisionGravityProxy[axis] + toolset.tool_mass * (0.15 + 0.05 * static_cast<double>(axis)) +
             com_norm * 0.5) +
        0.04 * snapshot.joint_velocity[axis];
    const double residual = std::fabs(snapshot.joint_torque[axis] - expected_torque);
    const double threshold =
        config.collision_nominal_thresholds[axis] /
        std::clamp(collision_detection.sensitivity[axis], 0.1, 10.0);
    if (residual > threshold) {
      axis_index = axis;
      std::ostringstream stream;
      stream << "collision residual exceeded threshold on joint " << (axis + 1);
      message = stream.str();
      return true;
    }
  }
  return false;
}

}  // namespace

RuntimeControlBridge::RuntimeControlBridge(RuntimeContext &runtime_context,
                                           RuntimeControlBridgeConfig config)
    : runtime_context_(runtime_context), config_(std::move(config)) {}

ControlTickResult RuntimeControlBridge::tick(BackendInterface &backend,
                                             const RobotSnapshot &snapshot,
                                             double dt) {
  ControlTickResult result;
  auto &session_state = runtime_context_.sessionState();
  auto &motion_runtime = runtime_context_.motionRuntime();
  auto &motion_options = runtime_context_.motionOptionsState();
  auto &tooling_state = runtime_context_.toolingState();

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
    backend.setControlOwner(ControlOwner::none);
    backend.clearControl();
    result.control_cleared = true;
    result.status = motion_runtime.status();
    return result;
  }

  const auto soft_limit = motion_options.softLimit();
  if (exceeds_soft_limit(snapshot, soft_limit)) {
    servo_accumulator_sec_ = 0.0;
    motion_runtime.stop("soft limit exceeded during execution");
    backend.setControlOwner(ControlOwner::none);
    backend.clearControl();
    result.control_cleared = true;
    result.status = motion_runtime.status();
    return result;
  }

  std::size_t collision_axis = 0;
  std::string collision_message;
  const auto collision_detection = session_state.collisionDetection();
  if (detect_collision_event(
          snapshot, collision_detection, tooling_state.toolset(), config_, collision_axis, collision_message)) {
    servo_accumulator_sec_ = 0.0;
    if (collision_detection.behaviour == 2) {
      motion_runtime.setActiveSpeedScale(config_.collision_slow_scale);
      collision_message += "; applying stop2 slowdown before stop";
    } else if (collision_detection.behaviour == 3) {
      collision_message += "; suppleStop";
    }

    motion_runtime.stop(collision_message);
    const auto retreat_command = make_retreat_command(snapshot, collision_detection.fallback, config_);
    if (retreat_command.has_effort) {
      backend.setControlOwner(ControlOwner::effort);
      backend.applyControl(retreat_command);
      backend.setControlOwner(ControlOwner::none);
    } else {
      backend.setControlOwner(ControlOwner::none);
      backend.clearControl();
      result.control_cleared = true;
    }
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
