#include "runtime/runtime_control_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
#include "rokae_xmate3_ros2/types.hpp"
#include "runtime/rt_field_registry.hpp"
#include "runtime/rt_prearm_checks.hpp"
#include "runtime/runtime_catalog_service.hpp"
#include "runtime/runtime_profile_service.hpp"
#include "runtime/planning_capability_service.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kServoTickSec = rokae_xmate3_ros2::spec::xmate3::kServoTickSec;
constexpr int kMaxServoSubstepsPerUpdate = 8;
constexpr std::array<double, 6> kCollisionRetreatEffort = {18.0, 18.0, 16.0, 8.0, 5.0, 3.0};
constexpr double kDefaultCollisionRetreatDistance = 0.04;
constexpr double kPi = 3.14159265358979323846;

constexpr std::array<double, 16> kIdentityMatrix16 = {1.0, 0.0, 0.0, 0.0,
                                                       0.0, 1.0, 0.0, 0.0,
                                                       0.0, 0.0, 1.0, 0.0,
                                                       0.0, 0.0, 0.0, 1.0};

rokae_xmate3_ros2::gazebo_model::LoadContext toLoadContext(
    const DataStoreState::RtLoadSnapshot &snapshot) {
  return {snapshot.mass, snapshot.cog};
}

template <typename T>
T configured_or(const bool configured, const T &value, const T &fallback) {
  return configured ? value : fallback;
}

double effective_filter_frequency(const DataStoreState::RtControlSnapshot &snapshot,
                                  double configured_frequency) {
  if (!snapshot.filter_limit_configured) {
    return configured_frequency;
  }
  if (!snapshot.filter_limit_enabled) {
    return std::numeric_limits<double>::infinity();
  }
  return std::min(configured_frequency, std::max(snapshot.filter_limit_cutoff_frequency, 1.0));
}

Eigen::Matrix3d read_rotation3x3(const std::array<double, 16> &values) {
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rot(row, col) = values[static_cast<std::size_t>(row * 4 + col)];
    }
  }
  return rot;
}

std::array<double, 6> transform_wrench_to_base(const DataStoreState::RtForceControlFrameSnapshot &config,
                                               const std::array<double, 16> &end_effector_frame,
                                               const std::array<double, 16> &flange_pose,
                                               const std::array<double, 6> &desired_wrench) {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  switch (config.type) {
    case rokae::FrameType::tool:
      rotation = read_rotation3x3(flange_pose) * read_rotation3x3(end_effector_frame) * read_rotation3x3(config.frame);
      break;
    case rokae::FrameType::path:
      rotation = read_rotation3x3(flange_pose) * read_rotation3x3(config.frame);
      break;
    case rokae::FrameType::world:
    case rokae::FrameType::base:
    default:
      rotation = config.configured ? read_rotation3x3(config.frame) : Eigen::Matrix3d::Identity();
      break;
  }
  Eigen::Matrix<double, 6, 1> wrench_in{};
  for (std::size_t i = 0; i < 6; ++i) {
    wrench_in(static_cast<int>(i)) = desired_wrench[i];
  }
  Eigen::Matrix<double, 6, 6> adjoint = Eigen::Matrix<double, 6, 6>::Zero();
  adjoint.block<3, 3>(0, 0) = rotation;
  adjoint.block<3, 3>(3, 3) = rotation;
  const auto wrench_out = adjoint * wrench_in;
  std::array<double, 6> transformed{};
  for (std::size_t i = 0; i < 6; ++i) {
    transformed[i] = wrench_out(static_cast<int>(i));
  }
  return transformed;
}

Eigen::Matrix4d array16_to_matrix(const std::array<double, 16> &values) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      matrix(row, col) = values[static_cast<std::size_t>(row * 4 + col)];
    }
  }
  return matrix;
}

std::vector<double> apply_end_effector_frame_to_tcp_target(const std::vector<double> &tcp_pose,
                                                           const std::array<double, 16> &end_effector_frame) {
  if (tcp_pose.size() != 6) {
    return tcp_pose;
  }
  const auto tcp_isometry = rokae_xmate3_ros2::runtime::pose_utils::poseToIsometry(tcp_pose);
  const Eigen::Isometry3d tool_isometry(array16_to_matrix(end_effector_frame));
  return rokae_xmate3_ros2::runtime::pose_utils::isometryToPose(tcp_isometry * tool_isometry.inverse());
}

bool target_within_cartesian_limit(const DataStoreState::RtCartesianLimitSnapshot &limit,
                                   const std::array<double, 6> &target) {
  if (!limit.enabled) {
    return true;
  }
  const Eigen::Vector4d point(target[0], target[1], target[2], 1.0);
  const Eigen::Vector4d local = array16_to_matrix(limit.frame).inverse() * point;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double half_extent = std::max(0.0, limit.lengths[axis]) * 0.5;
    if (std::fabs(local(static_cast<int>(axis))) > half_extent + 1e-9) {
      return false;
    }
  }
  return true;
}

std::array<double, 6> make_default_joint_damping(const std::array<double, 6> &kp) {
  std::array<double, 6> kd{};
  for (std::size_t i = 0; i < kd.size(); ++i) {
    kd[i] = std::max(2.0, 2.0 * std::sqrt(std::max(1.0, kp[i])));
  }
  return kd;
}

ControlCommand make_pd_effort(const RobotSnapshot &snapshot,
                              const std::array<double, 6> &target,
                              const std::array<double, 6> &kp,
                              const std::array<double, 6> &kd) {
  ControlCommand command;
  command.has_effort = true;
  for (std::size_t i = 0; i < 6; ++i) {
    const double position_error = target[i] - snapshot.joint_position[i];
    const double velocity_error = -snapshot.joint_velocity[i];
    const double unclamped = kp[i] * position_error + kd[i] * velocity_error;
    command.effort[i] = std::clamp(
        unclamped,
        -rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i],
        rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i]);
  }
  return command;
}

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
        std::clamp(collision_detection.sensitivity[axis], 0.01, 2.0);
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
  const auto rt_field_policy_summary = summarizeRtFieldPolicies(default_rt_plan_.fields());
  runtime_context_.diagnosticsState().setRtSubscriptionPlan(default_rt_plan_.summary() + "; " + rt_field_policy_summary);
  const auto active_profile = runtime_context_.diagnosticsState().snapshot().active_profile;
  const auto diagnostics_snapshot = runtime_context_.diagnosticsState().snapshot();
  const auto rt_snapshot = runtime_context_.dataStoreState().rtControlSnapshot();
  const auto semantic_snapshot = runtime_context_.dataStoreState().rtSemanticSnapshot();
  const bool rt_network_tolerance_configured =
      rt_snapshot.rt_network_tolerance_configured || config_.rt_network_tolerance_configured;
  const RtPrearmCheckInput prearm_input{
      session_state.motionMode(),
      session_state.rtControlMode(),
      session_state.powerOn(),
      rt_network_tolerance_configured,
      active_profile,
      diagnostics_snapshot.capability_flags,
      default_rt_plan_};
  const auto prearm_report = evaluateRtPrearm(prearm_input);
  runtime_context_.diagnosticsState().setRtPrearmStatus(prearm_report.summary());
  const bool rt_command_expected = session_state.motionMode() == kSessionMotionModeRt;
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
  runtime_context_.diagnosticsState().setProfileCapabilitySummary(
      summarizeRuntimeProfileCatalog(profiles) + "; " + rt_field_policy_summary);
  const auto kinematics_backends = buildKinematicsBackendCatalog();
  const auto retimer_policies = buildRetimerPolicyCatalog();
  const auto planner_policies = buildPlannerSelectionCatalog();
  runtime_context_.diagnosticsState().setPlanningCapabilitySummary(
      summarizePlanningCapabilityCatalog(kinematics_backends, retimer_policies, planner_policies));
  const auto options = buildRuntimeOptionCatalog(motion_options, session_state, rt_snapshot, semantic_snapshot);
  runtime_context_.diagnosticsState().setRuntimeOptionSummary(summarizeRuntimeOptionCatalog(options));
  const auto semantic_surface = semantic_snapshot.control_surface.empty() ? std::string{"sdk_shim"} : semantic_snapshot.control_surface;
  const auto rt_state_source = default_rt_plan_.use_state_data_in_loop() ? std::string{"rt_stream_in_loop"}
                                                                         : std::string{"rt_stream_polled"};
  runtime_context_.diagnosticsState().setSemanticSurface(
      semantic_surface,
      "runtime",
      semantic_snapshot.dispatch_mode.empty() ? std::string{"idle"} : semantic_snapshot.dispatch_mode);
  runtime_context_.diagnosticsState().setRtStateSource(rt_state_source);
  runtime_context_.diagnosticsState().setModelExactnessSummary(
      "kinematics=simulation_grade;model_primary_backend=kdl;model_fallback_used=false;dynamics=approximate;jacobian=simulation_grade;wrench=approximate;rt_state_source=" +
      rt_state_source);
  runtime_context_.diagnosticsState().setModelBackendInfo("kdl", false);
  runtime_context_.diagnosticsState().setCatalogProvenanceSummary(
      summarizeCatalogProvenance(semantic_snapshot));
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

  if (session_state.motionMode() == kSessionMotionModeRt &&
      session_state.rtControlMode() >= 0) {
    if (rt_snapshot.use_rci_client) {
      backend.setControlOwner(ControlOwner::none);
      backend.clearControl();
      result.control_cleared = true;
      result.status = motion_runtime.status();
      runtime_context_.diagnosticsState().setSemanticSurface(semantic_surface, "runtime_rci_client_enabled", "direct_rt_blocked_by_rci_client");
      return result;
    }

    if (rt_snapshot.stop_requested) {
      backend.setControlOwner(ControlOwner::none);
      backend.clearControl();
      result.control_cleared = true;
      result.status = motion_runtime.status();
      runtime_context_.diagnosticsState().setSemanticSurface(semantic_surface, "runtime_stop", "direct_rt_idle");
      return result;
    }

    DataStoreState::RtDirectCommandSnapshot direct_command;
    const double rt_command_timeout_sec = std::max(rt_snapshot.rt_command_timeout_sec, config_.rt_command_timeout_sec);
    switch (static_cast<rokae::RtControllerMode>(session_state.rtControlMode())) {
      case rokae::RtControllerMode::jointPosition:
      case rokae::RtControllerMode::jointImpedance:
        direct_command = rt_snapshot.joint_position_command;
        break;
      case rokae::RtControllerMode::cartesianPosition:
      case rokae::RtControllerMode::cartesianImpedance:
        direct_command = rt_snapshot.cartesian_position_command;
        break;
      case rokae::RtControllerMode::torque:
        direct_command = rt_snapshot.torque_command;
        break;
      default:
        break;
    }

    const double command_age_sec = direct_command.present
        ? std::chrono::duration<double>(std::chrono::steady_clock::now() - direct_command.updated_at).count()
        : std::numeric_limits<double>::infinity();
    const bool direct_command_fresh =
        direct_command.valid && command_age_sec <= std::max(rt_command_timeout_sec, 1e-6);

    if (direct_command_fresh) {
      if (runtime_context_.currentRuntimeView().busy()) {
        motion_runtime.stop("rt direct control engaged");
      }
      backend.setControlOwner(ControlOwner::effort);
      ControlCommand direct_effort;
      auto joint_kp = configured_or(
          rt_snapshot.joint_impedance_configured,
          rt_snapshot.joint_impedance,
          config_.joint_position_gain);
      auto joint_kd = make_default_joint_damping(joint_kp);
      const auto filter_freq = configured_or(
          rt_snapshot.filter_frequency_configured,
          rt_snapshot.filter_frequency,
          std::array<double, 6>{80.0, 80.0, 80.0, 80.0, 80.0, 80.0});
      const auto cartesian_limit = rt_snapshot.cartesian_limit;
      const auto ee_frame = rt_snapshot.end_effector_frame_configured ? rt_snapshot.end_effector_frame : kIdentityMatrix16;
      const auto fc_frame = rt_snapshot.force_control_frame;
      const auto load_context = toLoadContext(rt_snapshot.load);
      const auto gravity_comp = rokae_xmate3_ros2::gazebo_model::makeModelFacade(kinematics_, {}, load_context).gravity(snapshot.joint_position);
      const auto rt_mode = static_cast<rokae::RtControllerMode>(session_state.rtControlMode());
      if (rt_mode == rokae::RtControllerMode::jointPosition ||
          rt_mode == rokae::RtControllerMode::jointImpedance) {
        auto filtered_target = direct_command.values;
        const double joint_filter_hz = effective_filter_frequency(rt_snapshot, std::max(filter_freq[0], 1.0));
        const double alpha = std::isfinite(joint_filter_hz)
            ? std::clamp(2.0 * kPi * joint_filter_hz * std::max(dt, 1e-6), 0.0, 1.0)
            : 1.0;
        for (std::size_t i = 0; i < 6; ++i) {
          if (has_last_joint_target_) {
            filtered_target[i] = last_joint_target_[i] + alpha * (filtered_target[i] - last_joint_target_[i]);
          }
        }
        last_joint_target_ = filtered_target;
        has_last_joint_target_ = true;
        direct_effort = make_pd_effort(snapshot, filtered_target, joint_kp, joint_kd);
        for (std::size_t i = 0; i < 6; ++i) {
          direct_effort.effort[i] = std::clamp(
              direct_effort.effort[i] + gravity_comp[i],
              -rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i],
              rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i]);
        }
      } else if (rt_mode == rokae::RtControllerMode::cartesianPosition ||
                 rt_mode == rokae::RtControllerMode::cartesianImpedance) {
        if (!target_within_cartesian_limit(cartesian_limit, direct_command.values)) {
          backend.clearControl();
          result.control_cleared = true;
          result.status = motion_runtime.status();
          runtime_context_.diagnosticsState().setSemanticSurface(semantic_surface, "runtime_cartesian_limit_rejected", "direct_rt_rejected");
          return result;
        }
        std::vector<double> target_pose(direct_command.values.begin(), direct_command.values.end());
        target_pose = apply_end_effector_frame_to_tcp_target(target_pose, ee_frame);
        const auto flange_pose = kinematics_.forwardKinematicsRPY(std::vector<double>(snapshot.joint_position.begin(), snapshot.joint_position.end()));
        std::array<double, 16> flange_pose_matrix = kIdentityMatrix16;
        {
          const auto flange_transform = rokae_xmate3_ros2::runtime::pose_utils::poseToIsometry(flange_pose).matrix();
          for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
              flange_pose_matrix[static_cast<std::size_t>(row * 4 + col)] = flange_transform(row, col);
            }
          }
        }
        std::vector<double> current_joints(snapshot.joint_position.begin(), snapshot.joint_position.end());
        const auto target_joints = kinematics_.inverseKinematics(target_pose, current_joints);
        if (target_joints.size() == 6) {
          std::array<double, 6> joint_target{};
          std::copy_n(target_joints.begin(), 6, joint_target.begin());
          const double cartesian_filter_hz = effective_filter_frequency(rt_snapshot, std::max(filter_freq[1], 1.0));
          const double alpha = std::isfinite(cartesian_filter_hz)
              ? std::clamp(2.0 * kPi * cartesian_filter_hz * std::max(dt, 1e-6), 0.0, 1.0)
              : 1.0;
          for (std::size_t i = 0; i < 6; ++i) {
            if (has_last_cartesian_target_) {
              joint_target[i] = last_cartesian_target_[i] + alpha * (joint_target[i] - last_cartesian_target_[i]);
            }
          }
          last_cartesian_target_ = joint_target;
          has_last_cartesian_target_ = true;
          const auto cart_kp = configured_or(
              rt_snapshot.cartesian_impedance_configured,
              rt_snapshot.cartesian_impedance,
              std::array<double, 6>{150.0, 150.0, 150.0, 60.0, 40.0, 25.0});
          for (std::size_t i = 0; i < 6; ++i) {
            joint_kp[i] = std::max(joint_kp[i], cart_kp[i]);
          }
          joint_kd = make_default_joint_damping(joint_kp);
          direct_effort = make_pd_effort(snapshot, joint_target, joint_kp, joint_kd);
          for (std::size_t i = 0; i < 6; ++i) {
            direct_effort.effort[i] = std::clamp(
                direct_effort.effort[i] + gravity_comp[i],
                -rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i],
                rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i]);
          }
          if (rt_mode == rokae::RtControllerMode::cartesianImpedance) {
            const auto desired_wrench_local = configured_or(
                rt_snapshot.cartesian_desired_wrench_configured,
                rt_snapshot.cartesian_desired_wrench,
                std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            const auto desired_wrench = transform_wrench_to_base(fc_frame, ee_frame, flange_pose_matrix, desired_wrench_local);
            const auto jacobian = kinematics_.computeJacobian(current_joints);
            Eigen::Matrix<double, 6, 1> wrench;
            for (std::size_t i = 0; i < 6; ++i) {
              wrench(static_cast<int>(i)) = desired_wrench[i];
            }
            const auto joint_bias = jacobian.transpose() * wrench;
            for (std::size_t i = 0; i < 6; ++i) {
              direct_effort.effort[i] = std::clamp(
                  direct_effort.effort[i] + joint_bias(static_cast<int>(i)),
                  -rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i],
                  rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i]);
            }
          }
        } else {
          backend.clearControl();
          result.control_cleared = true;
          result.status = motion_runtime.status();
          runtime_context_.diagnosticsState().setSemanticSurface(semantic_surface, "runtime_cartesian_ik_failed", "direct_rt_ik_failed");
          return result;
        }
      } else {
        direct_effort.has_effort = true;
        auto filtered = direct_command.values;
        const double cutoff_hz = rt_snapshot.torque_cutoff_frequency_configured
            ? rt_snapshot.torque_cutoff_frequency
            : 80.0;
        const double alpha = std::clamp(2.0 * kPi * cutoff_hz * std::max(dt, 1e-6), 0.0, 1.0);
        for (std::size_t i = 0; i < 6; ++i) {
          if (has_last_torque_command_) {
            filtered[i] = last_torque_command_[i] + alpha * (filtered[i] - last_torque_command_[i]);
          }
          direct_effort.effort[i] = std::clamp(filtered[i] + gravity_comp[i],
              -rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i],
              rokae_xmate3_ros2::spec::xmate3::kDirectTorqueLimit[i]);
        }
        last_torque_command_ = direct_effort.effort;
        has_last_torque_command_ = true;
      }
      backend.applyControl(direct_effort);
      result.status = motion_runtime.status();
      runtime_context_.diagnosticsState().setSemanticSurface(semantic_surface, "runtime_direct_rt", "independent_rt");
      if (direct_command.finished) {
        backend.setControlOwner(ControlOwner::none);
        backend.clearControl();
        result.control_cleared = true;
      }
      last_rt_sequence_ = std::max(last_rt_sequence_, direct_command.sequence);
      return result;
    }

    const bool stale_direct_command = direct_command.present;

    backend.setControlOwner(ControlOwner::none);
    backend.clearControl();
    result.control_cleared = true;
    result.status = motion_runtime.status();
    runtime_context_.diagnosticsState().setSemanticSurface(
        semantic_surface,
        stale_direct_command ? "runtime_direct_command_timeout" : "runtime_no_direct_command",
        stale_direct_command ? "direct_rt_starved" : "direct_rt_waiting");
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
