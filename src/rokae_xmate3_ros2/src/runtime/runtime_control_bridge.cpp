#include "runtime/runtime_control_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
#include "rokae_xmate3_ros2/types.hpp"
#include "runtime/rt_prearm_checks.hpp"
#include "runtime/runtime_catalog_service.hpp"
#include "runtime/runtime_profile_service.hpp"
#include "runtime/planning_capability_service.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kServoTickSec = rokae_xmate3_ros2::spec::xmate3::kServoTickSec;
constexpr int kMaxServoSubstepsPerUpdate = 8;
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
                            ::gazebo::xMate3Kinematics &kinematics,
                            const ToolsetSnapshot &toolset,
                            const RuntimeControlBridgeConfig &config,
                            double dt,
                            bool &candidate_active,
                            std::size_t &candidate_axis,
                            double &candidate_time_sec,
                            double &debounce_remaining_sec,
                            std::size_t &axis_index,
                            std::string &message) {
  debounce_remaining_sec = std::max(0.0, debounce_remaining_sec - std::max(dt, 0.0));
  if (!collision_detection.enabled) {
    candidate_active = false;
    candidate_time_sec = 0.0;
    return false;
  }

  const auto expected =
      rokae_xmate3_ros2::gazebo_model::makeModelFacade(
          kinematics,
          {toolset.tool_pose[0], toolset.tool_pose[1], toolset.tool_pose[2],
           toolset.tool_pose[3], toolset.tool_pose[4], toolset.tool_pose[5]},
          {toolset.tool_mass, toolset.tool_com})
          .expectedTorque(snapshot.joint_position, snapshot.joint_velocity);
  std::size_t strongest_axis = 0;
  double strongest_ratio = 0.0;
  double strongest_threshold = 0.0;
  for (std::size_t axis = 0; axis < snapshot.joint_torque.size(); ++axis) {
    const double residual = std::fabs(snapshot.joint_torque[axis] - expected[axis]);
    const double threshold =
        config.collision_nominal_thresholds[axis] /
        std::clamp(collision_detection.sensitivity[axis], 0.1, 10.0);
    const double ratio = threshold > 1e-9 ? residual / threshold : 0.0;
    if (ratio > strongest_ratio) {
      strongest_axis = axis;
      strongest_ratio = ratio;
      strongest_threshold = threshold;
    }
  }

  const double release_ratio = std::clamp(1.0 - config.collision_hysteresis_ratio, 0.05, 1.0);
  if (strongest_ratio < release_ratio) {
    candidate_active = false;
    candidate_time_sec = 0.0;
    return false;
  }

  if (!candidate_active || candidate_axis != strongest_axis) {
    candidate_active = true;
    candidate_axis = strongest_axis;
    candidate_time_sec = std::max(dt, 0.0);
  } else {
    candidate_time_sec += std::max(dt, 0.0);
  }

  if (strongest_ratio <= 1.0 || debounce_remaining_sec > 0.0 ||
      candidate_time_sec + 1e-12 < std::max(config.collision_confirm_window_sec, 0.0)) {
    return false;
  }

  debounce_remaining_sec = std::max(config.collision_debounce_sec, 0.0);
  candidate_active = false;
  candidate_time_sec = 0.0;
  axis_index = strongest_axis;
  std::ostringstream stream;
  stream << "collision residual exceeded threshold on joint " << (strongest_axis + 1)
         << " ratio=" << strongest_ratio << " threshold=" << strongest_threshold;
  message = stream.str();
  return true;
}

}  // namespace

RuntimeControlBridge::RuntimeControlBridge(RuntimeContext &runtime_context,
                                           RuntimeControlBridgeConfig config)
    : runtime_context_(runtime_context),
      config_(std::move(config)),
      default_rt_plan_(buildRtSubscriptionPlan(config_.rt_default_fields,
                                               std::chrono::milliseconds(1),
                                               config_.rt_use_state_data_in_loop)),
      rt_watchdog_(config_.servo_lag_warning_sec) {}

ControlTickResult RuntimeControlBridge::tick(BackendInterface &backend,
                                             const RobotSnapshot &snapshot,
                                             double dt) {
  static rclcpp::Clock steady_clock{RCL_STEADY_TIME};
  ControlTickResult result;
  auto &session_state = runtime_context_.sessionState();
  auto &motion_runtime = runtime_context_.motionRuntime();
  auto &motion_options = runtime_context_.motionOptionsState();
  auto &tooling_state = runtime_context_.toolingState();
  runtime_context_.diagnosticsState().setLastServoDt(dt);
  const double loop_hz = (dt > 1e-9 && std::isfinite(dt)) ? 1.0 / dt : 0.0;
  runtime_context_.diagnosticsState().setLoopMetrics(loop_hz, loop_hz, std::max(dt, 0.0) * 1000.0);
  runtime_context_.diagnosticsState().setSessionModes(session_state.motionMode(), session_state.rtControlMode());
  runtime_context_.diagnosticsState().setRtSubscriptionPlan(default_rt_plan_.summary());
  const auto active_profile = runtime_context_.diagnosticsState().snapshot().active_profile;
  const auto diagnostics_snapshot = runtime_context_.diagnosticsState().snapshot();
  const RtPrearmCheckInput prearm_input{
      session_state.motionMode(),
      session_state.rtControlMode(),
      session_state.powerOn(),
      config_.rt_network_tolerance_configured,
      active_profile,
      diagnostics_snapshot.capability_flags,
      default_rt_plan_};
  const auto prearm_report = evaluateRtPrearm(prearm_input);
  runtime_context_.diagnosticsState().setRtPrearmStatus(prearm_report.summary());
  const bool rt_command_expected = session_state.motionMode() == static_cast<int>(rokae::MotionControlMode::RtCommand);
  rt_watchdog_.observeCycle(dt, true, !rt_command_expected || session_state.powerOn());
  const auto watchdog_snapshot = rt_watchdog_.snapshot();
  runtime_context_.diagnosticsState().setRtWatchdogSummary(
      watchdog_snapshot.summary,
      watchdog_snapshot.late_cycle_count,
      watchdog_snapshot.max_gap_ms,
      watchdog_snapshot.avg_gap_ms,
      watchdog_snapshot.consecutive_late_cycles,
      watchdog_snapshot.stale_state_count,
      watchdog_snapshot.command_starvation_windows,
      watchdog_snapshot.last_trigger_reason);
  const auto profiles = buildRuntimeProfileCatalog(
      diagnostics_snapshot.backend_mode,
      active_profile,
      diagnostics_snapshot.capability_flags);
  runtime_context_.diagnosticsState().setProfileCapabilitySummary(summarizeRuntimeProfileCatalog(profiles));
  const auto kinematics_backends = buildKinematicsBackendCatalog();
  const auto retimer_policies = buildRetimerPolicyCatalog();
  const auto planner_policies = buildPlannerSelectionCatalog();
  runtime_context_.diagnosticsState().setPlanningCapabilitySummary(
      summarizePlanningCapabilityCatalog(kinematics_backends, retimer_policies, planner_policies));
  const auto options = buildRuntimeOptionCatalog(motion_options, session_state);
  runtime_context_.diagnosticsState().setRuntimeOptionSummary(summarizeRuntimeOptionCatalog(options));
  runtime_context_.diagnosticsState().setCatalogSizes(
      static_cast<std::uint32_t>(buildRuntimeToolCatalog(tooling_state).size()),
      static_cast<std::uint32_t>(buildRuntimeWobjCatalog(tooling_state).size()),
      static_cast<std::uint32_t>(buildRuntimeProjectCatalog(runtime_context_.programState()).size()),
      static_cast<std::uint32_t>(buildRuntimeRegisterCatalog(runtime_context_.dataStoreState()).size()));

  if (!session_state.powerOn()) {
    servo_accumulator_sec_ = 0.0;
    collision_candidate_active_ = false;
    collision_candidate_time_sec_ = 0.0;
    collision_debounce_remaining_sec_ = 0.0;
    rt_watchdog_.reset();
    {
      const auto watchdog_snapshot = rt_watchdog_.snapshot();
      runtime_context_.diagnosticsState().setRtWatchdogSummary(
          watchdog_snapshot.summary,
          watchdog_snapshot.late_cycle_count,
          watchdog_snapshot.max_gap_ms,
          watchdog_snapshot.avg_gap_ms,
          watchdog_snapshot.consecutive_late_cycles,
          watchdog_snapshot.stale_state_count,
          watchdog_snapshot.command_starvation_windows,
          watchdog_snapshot.last_trigger_reason);
    }
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
    collision_candidate_active_ = false;
    collision_candidate_time_sec_ = 0.0;
    rt_watchdog_.reset();
    {
      const auto watchdog_snapshot = rt_watchdog_.snapshot();
      runtime_context_.diagnosticsState().setRtWatchdogSummary(
          watchdog_snapshot.summary,
          watchdog_snapshot.late_cycle_count,
          watchdog_snapshot.max_gap_ms,
          watchdog_snapshot.avg_gap_ms,
          watchdog_snapshot.consecutive_late_cycles,
          watchdog_snapshot.stale_state_count,
          watchdog_snapshot.command_starvation_windows,
          watchdog_snapshot.last_trigger_reason);
    }
    backend.setControlOwner(ControlOwner::none);
    backend.clearControl();
    result.control_cleared = true;
    result.status = motion_runtime.status();
    return result;
  }

  const auto soft_limit = motion_options.softLimit();
  if (exceeds_soft_limit(snapshot, soft_limit)) {
    servo_accumulator_sec_ = 0.0;
    collision_candidate_active_ = false;
    collision_candidate_time_sec_ = 0.0;
    rt_watchdog_.reset();
    {
      const auto watchdog_snapshot = rt_watchdog_.snapshot();
      runtime_context_.diagnosticsState().setRtWatchdogSummary(
          watchdog_snapshot.summary,
          watchdog_snapshot.late_cycle_count,
          watchdog_snapshot.max_gap_ms,
          watchdog_snapshot.avg_gap_ms,
          watchdog_snapshot.consecutive_late_cycles,
          watchdog_snapshot.stale_state_count,
          watchdog_snapshot.command_starvation_windows,
          watchdog_snapshot.last_trigger_reason);
    }
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
          snapshot,
          collision_detection,
          kinematics_,
          tooling_state.toolset(),
          config_,
          dt,
          collision_candidate_active_,
          collision_candidate_axis_,
          collision_candidate_time_sec_,
          collision_debounce_remaining_sec_,
          collision_axis,
          collision_message)) {
    servo_accumulator_sec_ = 0.0;
    rt_watchdog_.reset();
    {
      const auto watchdog_snapshot = rt_watchdog_.snapshot();
      runtime_context_.diagnosticsState().setRtWatchdogSummary(
          watchdog_snapshot.summary,
          watchdog_snapshot.late_cycle_count,
          watchdog_snapshot.max_gap_ms,
          watchdog_snapshot.avg_gap_ms,
          watchdog_snapshot.consecutive_late_cycles,
          watchdog_snapshot.stale_state_count,
          watchdog_snapshot.command_starvation_windows,
          watchdog_snapshot.last_trigger_reason);
    }
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
  if (dt > config_.servo_lag_warning_sec) {
    ++servo_lag_warning_count_;
    RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("runtime_control_bridge"),
        steady_clock,
        2000,
        "servo bridge dt=%.6f exceeds warning threshold %.6f (count=%zu)",
        dt,
        config_.servo_lag_warning_sec,
        servo_lag_warning_count_);
  }
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
