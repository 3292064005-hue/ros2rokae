#include "runtime/planner_core.hpp"

#include <utility>

#include "runtime/planning_utils.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

using ::gazebo::TrajectoryPlanner;

bool move_absj_violates_soft_limit(const std::vector<double> &joints,
                                   const std::array<std::array<double, 2>, 6> &soft_limits) {
  for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
    if (joints[i] < soft_limits[i][0] || joints[i] > soft_limits[i][1]) {
      return true;
    }
  }
  return false;
}

}  // namespace

MotionPlanner::MotionPlanner() : kinematics_(std::make_unique<::gazebo::xMate3Kinematics>()) {}

MotionPlan MotionPlanner::plan(const MotionRequest &request) const {
  MotionPlan plan;
  plan.request_id = request.request_id;

  if (request.commands.empty()) {
    plan.error_message = "motion request is empty";
    return plan;
  }
  if (request.start_joints.size() < 6) {
    plan.error_message = "start_joints must contain 6 values";
    return plan;
  }

  auto current_joints = request.start_joints;
  plan.segments.reserve(request.commands.size());

  for (const auto &cmd : request.commands) {
    PlannedSegment segment;
    segment.kind = cmd.kind;
    segment.speed = cmd.speed > 0 ? cmd.speed : request.default_speed;
    segment.zone = cmd.zone;
    segment.trajectory_dt = request.trajectory_dt;
    segment.target_cartesian = cmd.target_cartesian;
    segment.aux_cartesian = cmd.aux_cartesian;

    if (cmd.use_preplanned_trajectory) {
      if (cmd.preplanned_trajectory.empty()) {
        plan.error_message = "preplanned trajectory is empty";
        return plan;
      }
      segment.joint_trajectory = cmd.preplanned_trajectory;
      segment.trajectory_dt = cmd.preplanned_dt > 1e-9 ? cmd.preplanned_dt : request.trajectory_dt;
      segment.target_joints = cmd.target_joints.empty() ? cmd.preplanned_trajectory.back() : cmd.target_joints;
      current_joints = segment.target_joints;
      plan.segments.push_back(std::move(segment));
      continue;
    }

    switch (cmd.kind) {
      case MotionKind::move_absj:
        segment.target_joints = cmd.target_joints;
        if (request.soft_limit_enabled &&
            move_absj_violates_soft_limit(segment.target_joints, request.soft_limits)) {
          plan.error_message = "MoveAbsJ target violates soft limit";
          return plan;
        }
        segment.joint_trajectory = TrajectoryPlanner::planJointMove(
            current_joints, segment.target_joints, segment.speed, request.trajectory_dt);
        break;
      case MotionKind::move_j: {
        const auto candidate_solutions =
            kinematics_->inverseKinematicsMultiSolution(cmd.target_cartesian, current_joints);
        const auto selected = select_ik_solution(
            *kinematics_, candidate_solutions, cmd.target_cartesian, current_joints,
            cmd.requested_conf, request.strict_conf, request.avoid_singularity,
            request.soft_limit_enabled, request.soft_limits);
        if (!selected.success) {
          plan.error_message = "MoveJ planning failed: " + selected.message;
          return plan;
        }
        segment.target_joints = selected.joints;
        segment.joint_trajectory = TrajectoryPlanner::planJointMove(
            current_joints, segment.target_joints, segment.speed, request.trajectory_dt);
        break;
      }
      case MotionKind::move_l: {
        const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
        const auto cart_trajectory = TrajectoryPlanner::planCartesianLine(
            current_pose, cmd.target_cartesian, segment.speed, request.trajectory_dt);
        std::vector<double> last_joints;
        std::string error_message;
        if (!build_joint_trajectory_from_cartesian(
                *kinematics_, cart_trajectory, current_joints, cmd.requested_conf,
                request.strict_conf, request.avoid_singularity, request.soft_limit_enabled,
                request.soft_limits, segment.joint_trajectory, last_joints, error_message)) {
          plan.error_message = "MoveL planning failed: " + error_message;
          return plan;
        }
        segment.target_joints = last_joints;
        break;
      }
      case MotionKind::move_c: {
        const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
        const auto cart_trajectory = TrajectoryPlanner::planCircularArc(
            current_pose, cmd.aux_cartesian, cmd.target_cartesian, segment.speed,
            request.trajectory_dt);
        std::vector<double> last_joints;
        std::string error_message;
        if (!build_joint_trajectory_from_cartesian(
                *kinematics_, cart_trajectory, current_joints, cmd.requested_conf,
                request.strict_conf, request.avoid_singularity, request.soft_limit_enabled,
                request.soft_limits, segment.joint_trajectory, last_joints, error_message)) {
          plan.error_message = "MoveC planning failed: " + error_message;
          return plan;
        }
        segment.target_joints = last_joints;
        break;
      }
      case MotionKind::move_cf: {
        const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
        const auto cart_trajectory = TrajectoryPlanner::planCircularContinuous(
            current_pose, cmd.aux_cartesian, cmd.target_cartesian, cmd.angle, segment.speed,
            request.trajectory_dt);
        std::vector<double> last_joints;
        std::string error_message;
        if (!build_joint_trajectory_from_cartesian(
                *kinematics_, cart_trajectory, current_joints, cmd.requested_conf,
                request.strict_conf, request.avoid_singularity, request.soft_limit_enabled,
                request.soft_limits, segment.joint_trajectory, last_joints, error_message)) {
          plan.error_message = "MoveCF planning failed: " + error_message;
          return plan;
        }
        segment.target_joints = last_joints;
        break;
      }
      case MotionKind::move_sp: {
        const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
        const auto cart_trajectory = TrajectoryPlanner::planSpiralMove(
            current_pose, cmd.target_cartesian, cmd.radius, cmd.radius_step, cmd.angle,
            cmd.direction, segment.speed, request.trajectory_dt);
        std::vector<double> last_joints;
        std::string error_message;
        if (!build_joint_trajectory_from_cartesian(
                *kinematics_, cart_trajectory, current_joints, cmd.requested_conf,
                request.strict_conf, request.avoid_singularity, request.soft_limit_enabled,
                request.soft_limits, segment.joint_trajectory, last_joints, error_message)) {
          plan.error_message = "MoveSP planning failed: " + error_message;
          return plan;
        }
        segment.target_joints = last_joints;
        break;
      }
      default:
        plan.error_message = "unsupported motion kind";
        return plan;
    }

    if (segment.joint_trajectory.empty()) {
      plan.error_message = "planning produced an empty trajectory";
      return plan;
    }
    if (segment.target_joints.empty()) {
      segment.target_joints = segment.joint_trajectory.back();
    }
    current_joints = segment.target_joints;
    plan.segments.push_back(std::move(segment));
  }

  return plan;
}

}  // namespace rokae_xmate3_ros2::runtime
