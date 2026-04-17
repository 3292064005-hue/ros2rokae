#include "runtime/service_facade.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include <Eigen/Dense>

#include "runtime/planner_core.hpp"
#include "runtime/motion_runtime_internal.hpp"
#include "runtime/planner_preflight.hpp"
#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade_utils.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleCalcFk(const rokae_xmate3_ros2::srv::CalcFk::Request &req,
                               rokae_xmate3_ros2::srv::CalcFk::Response &res) const {
  std::vector<double> joints(req.joint_positions.begin(), req.joint_positions.end());
  const auto flange_pose = kinematics_.forwardKinematicsRPY(joints);
  const auto pose = detail::resolve_pose_for_coordinate(flange_pose, tooling_state_.toolset(), 1);
  for (int i = 0; i < 6; ++i) {
    res.posture[i] = pose[i];
  }
  res.success = true;
}

void QueryFacade::handleCalcIk(const rokae_xmate3_ros2::srv::CalcIk::Request &req,
                               rokae_xmate3_ros2::srv::CalcIk::Response &res) const {
  struct RequestGuard {
    explicit RequestGuard(::gazebo::xMate3Kinematics &kinematics) : kinematics_(kinematics) {
      kinematics_.beginRequestContract("query_calc_ik");
    }
    ~RequestGuard() { kinematics_.endRequestContract(); }
    ::gazebo::xMate3Kinematics &kinematics_;
  } request_guard(kinematics_);
  std::vector<double> target(req.target_posture.begin(), req.target_posture.end());
  target = pose_utils::convertEndInRefToFlangeInBase(
      target, tooling_state_.toolset().tool_pose, tooling_state_.toolset().wobj_pose);
  std::array<double, 6> pos{};
  std::array<double, 6> vel{};
  std::array<double, 6> tau{};
  readAuthorityJointState(pos, vel, tau);
  const auto current = detail::snapshot_joints(pos);
  auto candidates = kinematics_.inverseKinematicsMultiSolution(target, current);
  const auto seeded_fast = kinematics_.inverseKinematicsSeededFast(target, current);
  if (!seeded_fast.empty()) {
    candidates.insert(candidates.begin(), seeded_fast);
  }
  const auto soft_limit = motion_options_state_.softLimit();
  ::gazebo::xMate3Kinematics::CartesianIkOptions ik_options;
  ik_options.requested_conf.assign(req.conf_data.begin(), req.conf_data.end());
  ik_options.strict_conf = motion_options_state_.defaultConfOptForced();
  ik_options.avoid_singularity = true;
  ik_options.soft_limit_enabled = soft_limit.enabled;
  ik_options.soft_limits = soft_limit.limits;
  const auto selected = kinematics_.selectBestIkSolution(candidates, target, current, ik_options);
  const auto request_contract = kinematics_.requestContractState();
  if (request_contract.violated) {
    res.success = false;
    res.message = request_contract.violation_reason;
    return;
  }
  if (!selected.success) {
    res.success = false;
    res.message = selected.message;
    return;
  }
  for (size_t i = 0; i < 6 && i < selected.joints.size(); ++i) {
    res.joint_positions[i] = selected.joints[i];
  }
  res.success = true;
  res.message.clear();
}

void QueryFacade::handleCalcJointTorque(
    const rokae_xmate3_ros2::srv::CalcJointTorque::Request &req,
    rokae_xmate3_ros2::srv::CalcJointTorque::Response &res) const {
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(req.joint_pos[i]) || !std::isfinite(req.joint_vel[i]) ||
        !std::isfinite(req.joint_acc[i]) || !std::isfinite(req.external_force[i])) {
      res.success = false;
      res.error_code = 4002;
      res.error_msg = "joint torque inputs must be finite";
      return;
    }
  }
  const auto toolset = tooling_state_.toolset();
  const auto breakdown = rokae_xmate3_ros2::gazebo_model::computeApproximateDynamics(
      kinematics_,
      req.joint_pos,
      req.joint_vel,
      req.joint_acc,
      req.external_force,
      detail::resolve_load_context(toolset));
  res.joint_torque = breakdown.full;
  res.gravity_torque = breakdown.gravity;
  res.coriolis_torque = breakdown.coriolis;
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

void QueryFacade::handleGenerateSTrajectory(
    const rokae_xmate3_ros2::srv::GenerateSTrajectory::Request &req,
    rokae_xmate3_ros2::srv::GenerateSTrajectory::Response &res) const {
  std::vector<double> start(req.start_joint_pos.begin(), req.start_joint_pos.end());
  std::vector<double> target(req.target_joint_pos.begin(), req.target_joint_pos.end());
  const auto stamp = time_provider_();
  const double sample_dt = std::max(trajectory_dt_provider_(), 1e-3);
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(start[i]) || !std::isfinite(target[i])) {
      res.success = false;
      res.error_code = 3003;
      res.error_msg = "trajectory inputs must be finite";
      return;
    }
  }
  if (start.size() != 6 || target.size() != 6) {
    res.success = false;
    res.error_code = 3003;
    res.error_msg = "joint-space S-trajectory requires 6 joint values";
    return;
  }

  double max_delta = 0.0;
  for (size_t i = 0; i < start.size(); ++i) {
    max_delta = std::max(max_delta, std::fabs(target[i] - start[i]));
  }
  if (max_delta < 1e-9) {
    rokae_xmate3_ros2::msg::JointPos6 joint_msg;
    for (int i = 0; i < 6; ++i) {
      joint_msg.pos[i] = start[i];
    }
    res.trajectory_points = {joint_msg};
    res.total_time = 0.0;
    res.stamp = detail::ToBuiltinTime(stamp);
    res.success = true;
    res.error_code = 0;
    RetimerMetadata metadata;
    metadata.source_family = RetimerSourceFamily::s_trajectory;
    metadata.sample_dt = sample_dt;
    metadata.total_duration = 0.0;
    metadata.note = RetimerNote::degenerate_path;
    detail::append_retimer_diagnostic(
        data_store_state_, stamp, "generate_s_trajectory", metadata, "start equals target", &diagnostics_state_);
    return;
  }

  std::vector<std::vector<double>> trajectory_positions;
  double total_time = 0.0;
  bool cartesian_fallback_to_joint = false;
  std::optional<RetimerMetadata> retimer_metadata;
  if (req.is_cartesian) {
    const auto cartesian_retimed = buildApproximateCartesianSTrajectory(kinematics_, req, sample_dt);
    if (cartesian_retimed.empty()) {
      cartesian_fallback_to_joint = true;
      detail::append_retimer_diagnostic(
          data_store_state_,
          stamp,
          "generate_s_trajectory",
          cartesian_retimed.metadata,
          cartesian_retimed.samples.error_message.empty()
              ? "cartesian approximation failed; falling back to joint retimer"
              : cartesian_retimed.samples.error_message,
          &diagnostics_state_);
    } else {
      trajectory_positions = cartesian_retimed.samples.positions;
      total_time = cartesian_retimed.samples.total_time;
      retimer_metadata = cartesian_retimed.metadata;
    }
  }

  if (!req.is_cartesian || cartesian_fallback_to_joint) {
    auto retimed = retimeJointWithUnifiedConfig(
        start,
        target,
        sample_dt,
        req.max_velocity,
        req.max_acceleration,
        req.blend_radius,
        RetimerSourceFamily::s_trajectory);
    if (retimed.empty()) {
      res.success = false;
      res.error_code = 3003;
      res.error_msg = "trajectory planning failed";
      return;
    }
    if (cartesian_fallback_to_joint) {
      retimed.metadata.note = RetimerNote::cartesian_fallback_to_joint;
    }
    trajectory_positions = retimed.samples.positions;
    total_time = retimed.samples.total_time;
    retimer_metadata = retimed.metadata;
  }

  res.trajectory_points.clear();
  res.trajectory_points.reserve(trajectory_positions.size());
  for (const auto &point : trajectory_positions) {
    rokae_xmate3_ros2::msg::JointPos6 joint_msg;
    for (int j = 0; j < 6; ++j) {
      joint_msg.pos[j] = point[j];
    }
    res.trajectory_points.push_back(joint_msg);
  }
  res.total_time = total_time;
  res.stamp = detail::ToBuiltinTime(stamp);
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
  if (retimer_metadata.has_value()) {
    detail::append_retimer_diagnostic(
        data_store_state_, stamp, "generate_s_trajectory", *retimer_metadata, {}, &diagnostics_state_);
  }
}

void QueryFacade::handleValidateMotion(const rokae_xmate3_ros2::srv::ValidateMotion::Request &req,
                                       rokae_xmate3_ros2::srv::ValidateMotion::Response &res) const {
  std::vector<double> start_joints;
  if (req.use_start_joint_pos) {
    start_joints.assign(req.start_joint_pos.begin(), req.start_joint_pos.end());
  } else {
    std::array<double, 6> pos{};
    std::array<double, 6> vel{};
    std::array<double, 6> tau{};
    readAuthorityJointState(pos, vel, tau);
    start_joints = detail::snapshot_joints(pos);
  }

  if (start_joints.size() != 6 || !detail::is_finite_vector(start_joints)) {
    res.success = false;
    res.reachable = false;
    res.message = "validate_motion requires 6 finite start joint values";
    res.reject_reason = "unreachable_pose";
    return;
  }

  MotionRequest request;
  request.request_id = "validate_motion";
  request.start_joints = start_joints;
  request.default_speed = req.speed > 0 ? static_cast<double>(req.speed) : motion_options_state_.defaultSpeed();
  request.default_zone = req.zone >= 0 ? req.zone : motion_options_state_.defaultZone();
  request.speed_scale = (std::isfinite(req.speed_scale) && req.speed_scale > 0.0)
                            ? std::clamp(req.speed_scale, 0.05, 2.0)
                            : motion_options_state_.speedScale();
  request.strict_conf = req.strict_conf;
  request.avoid_singularity = req.avoid_singularity;
  const auto configured_soft_limit = motion_options_state_.softLimit();
  request.soft_limit_enabled = req.soft_limit_enabled || configured_soft_limit.enabled;
  request.soft_limits =
      req.soft_limit_enabled ? detail::soft_limits_from_request(req.soft_limits) : configured_soft_limit.limits;
  request.trajectory_dt = std::max(trajectory_dt_provider_(), 1e-3);

  MotionCommandSpec command;
  command.speed = request.default_speed;
  command.zone = request.default_zone;
  command.requested_conf.assign(req.conf_data.begin(), req.conf_data.end());
  const auto toolset = tooling_state_.toolset();
  switch (req.motion_kind) {
    case rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_ABSJ:
      command.kind = MotionKind::move_absj;
      command.target_joints.assign(req.target_joint_pos.begin(), req.target_joint_pos.end());
      break;
    case rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_J:
      command.kind = MotionKind::move_j;
      command.target_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.target_posture), toolset.tool_pose, toolset.wobj_pose);
      break;
    case rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_L:
      command.kind = MotionKind::move_l;
      command.target_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.target_posture), toolset.tool_pose, toolset.wobj_pose);
      break;
    case rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_C:
      command.kind = MotionKind::move_c;
      command.target_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.target_posture), toolset.tool_pose, toolset.wobj_pose);
      command.aux_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.aux_posture), toolset.tool_pose, toolset.wobj_pose);
      break;
    case rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_CF:
      command.kind = MotionKind::move_cf;
      command.target_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.target_posture), toolset.tool_pose, toolset.wobj_pose);
      command.aux_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.aux_posture), toolset.tool_pose, toolset.wobj_pose);
      command.angle = req.angle;
      break;
    case rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_SP:
      command.kind = MotionKind::move_sp;
      command.target_cartesian = pose_utils::convertEndInRefToFlangeInBase(
          detail::pose_from_array(req.target_posture), toolset.tool_pose, toolset.wobj_pose);
      command.angle = req.angle;
      command.radius = req.radius;
      command.radius_step = req.radius_step;
      command.direction = req.direction;
      break;
    default:
      res.success = false;
      res.reachable = false;
      res.message = "unsupported validate_motion kind";
      res.reject_reason = "unreachable_pose";
      return;
  }

  if ((!command.target_joints.empty() && !detail::is_finite_vector(command.target_joints)) ||
      (!command.target_cartesian.empty() && !detail::is_finite_vector(command.target_cartesian)) ||
      (!command.aux_cartesian.empty() && !detail::is_finite_vector(command.aux_cartesian))) {
    res.success = false;
    res.reachable = false;
    res.message = "validate_motion targets must be finite";
    res.reject_reason = "unreachable_pose";
    return;
  }

  request.commands.push_back(command);
  const auto preflight = runPlannerPreflight(request);
  MotionPlanner planner;
  const auto plan = planner.plan(request);
  res.success = true;
  res.reachable = preflight.ok && plan.valid();
  res.notes = preflight.notes;
  res.notes.insert(res.notes.end(), plan.notes.begin(), plan.notes.end());
  res.reject_reason = preflight.reject_reason;
  res.retimer_family = preflight.retimer_family;
  res.selected_branch = preflight.selected_branch;

  if (!preflight.ok) {
    res.estimated_duration = preflight.estimated_duration;
    res.message = preflight.detail.empty() ? std::string("planner preflight rejected request") : preflight.detail;
    diagnostics_state_.notePlanFailure(res.message);
    return;
  }

  if (plan.valid()) {
    res.estimated_duration = std::accumulate(
        plan.segments.begin(),
        plan.segments.end(),
        0.0,
        [](double total, const PlannedSegment &segment) { return total + segment.trajectory_total_time; });
    if (res.estimated_duration <= 0.0) {
      res.estimated_duration = preflight.estimated_duration;
    }
    if (!plan.segments.empty()) {
      res.retimer_family = preflight.retimer_family + std::string("/") + to_string(plan.segments.front().path_family);
    }
    res.message = "validation succeeded";
    diagnostics_state_.notePlanSummary(plan.explanation_summary.empty() ? summarize_plan_notes(plan) : plan.explanation_summary,
                                       plan.selected_candidate);
    for (auto it = plan.notes.rbegin(); it != plan.notes.rend(); ++it) {
      if (it->find("retimer[") != std::string::npos) {
        diagnostics_state_.noteRetimerNote(*it);
        break;
      }
    }
    if (req.motion_kind == rokae_xmate3_ros2::srv::ValidateMotion::Request::MOTION_MOVE_J) {
      std::vector<std::vector<double>> candidates;
      const auto seeded_fast = kinematics_.inverseKinematicsSeededFast(command.target_cartesian, start_joints);
      if (!seeded_fast.empty()) {
        candidates.push_back(seeded_fast);
      }
      const auto multi_branch = kinematics_.inverseKinematicsMultiSolution(command.target_cartesian, start_joints);
      candidates.insert(candidates.end(), multi_branch.begin(), multi_branch.end());
      ::gazebo::xMate3Kinematics::CartesianIkOptions options;
      options.requested_conf = command.requested_conf;
      options.strict_conf = request.strict_conf;
      options.avoid_singularity = request.avoid_singularity;
      options.soft_limit_enabled = request.soft_limit_enabled;
      options.soft_limits = request.soft_limits;
      const auto selected =
          kinematics_.selectBestIkSolution(candidates, command.target_cartesian, start_joints, options);
      if (!selected.branch_id.empty()) {
        res.selected_branch = selected.branch_id;
      }
      const auto &trace = kinematics_.lastTrace();
      if (!trace.request_kind.empty()) {
        res.notes.push_back("ik_trace.kind=" + trace.request_kind +
                            ";backend=" + trace.primary_backend +
                            ";fallback=" + (trace.fallback_used ? std::string("true") : std::string("false")) +
                            ";fallback_reason=" + trace.fallback_reason +
                            ";seed_source=" + trace.seed_source +
                            ";branch=" + trace.selected_branch +
                            ";continuity=" + std::to_string(trace.continuity_cost) +
                            ";singularity=" + std::to_string(trace.singularity_metric));
      }
    }
    return;
  }

  res.estimated_duration = 0.0;
  res.retimer_family = "none";
  res.message = plan.error_message;
  res.reject_reason = classify_motion_failure_reason(plan.error_message);
  diagnostics_state_.notePlanFailure(plan.error_message);
  diagnostics_state_.notePlanSummary(plan.explanation_summary.empty() ? plan.error_message : plan.explanation_summary,
                                     plan.selected_candidate);
}

void QueryFacade::handleMapCartesianToJointTorque(
    const rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Request &req,
    rokae_xmate3_ros2::srv::MapCartesianToJointTorque::Response &res) const {
  std::vector<double> joints(req.joint_pos.begin(), req.joint_pos.end());
  if (joints.size() != 6) {
    res.success = false;
    res.error_code = 6002;
    res.error_msg = "joint_pos size is invalid";
    return;
  }
  const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joints);
  Eigen::Matrix<double, 6, 1> wrench;
  for (int i = 0; i < 6; ++i) {
    wrench(i) = req.cart_force[i];
  }
  const Eigen::Matrix<double, 6, 1> tau = jacobian.transpose() * wrench;
  for (int i = 0; i < 6; ++i) {
    res.joint_torque[i] = tau(i);
  }
  res.stamp = detail::ToBuiltinTime(time_provider_());
  res.success = true;
  res.error_code = 0;
  res.error_msg.clear();
}

}  // namespace rokae_xmate3_ros2::runtime
