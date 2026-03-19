#include "runtime/request_adapter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr int kOffsetNone = 0;
constexpr int kOffsetOffs = 1;

[[nodiscard]] double normalize_angle(double value) {
  while (value > M_PI) {
    value -= 2.0 * M_PI;
  }
  while (value < -M_PI) {
    value += 2.0 * M_PI;
  }
  return value;
}

[[nodiscard]] std::vector<double> msg_pose_to_vector(
    const rokae_xmate3_ros2::msg::CartesianPosition &pose) {
  return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
}

[[nodiscard]] std::vector<double> array_pose_to_vector(const std::array<double, 6> &pose) {
  return {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
}

[[nodiscard]] Eigen::Matrix4d pose_to_transform(const std::vector<double> &pose) {
  Eigen::Matrix3d rx;
  Eigen::Matrix3d ry;
  Eigen::Matrix3d rz;
  rx << 1, 0, 0,
        0, std::cos(pose[3]), -std::sin(pose[3]),
        0, std::sin(pose[3]), std::cos(pose[3]);
  ry << std::cos(pose[4]), 0, std::sin(pose[4]),
        0, 1, 0,
        -std::sin(pose[4]), 0, std::cos(pose[4]);
  rz << std::cos(pose[5]), -std::sin(pose[5]), 0,
        std::sin(pose[5]), std::cos(pose[5]), 0,
        0, 0, 1;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rz * ry * rx;
  transform(0, 3) = pose[0];
  transform(1, 3) = pose[1];
  transform(2, 3) = pose[2];
  return transform;
}

[[nodiscard]] std::vector<double> transform_to_pose(const Eigen::Matrix4d &transform) {
  std::vector<double> pose(6, 0.0);
  pose[0] = transform(0, 3);
  pose[1] = transform(1, 3);
  pose[2] = transform(2, 3);
  pose[4] = std::atan2(-transform(2, 0),
                       std::sqrt(transform(0, 0) * transform(0, 0) +
                                 transform(1, 0) * transform(1, 0)));
  if (std::fabs(pose[4] - M_PI / 2.0) < 1e-3) {
    pose[5] = 0.0;
    pose[3] = std::atan2(transform(1, 2), transform(1, 1));
  } else if (std::fabs(pose[4] + M_PI / 2.0) < 1e-3) {
    pose[5] = 0.0;
    pose[3] = std::atan2(-transform(1, 2), transform(1, 1));
  } else {
    pose[5] = std::atan2(transform(1, 0), transform(0, 0));
    pose[3] = std::atan2(transform(2, 1), transform(2, 2));
  }
  pose[3] = normalize_angle(pose[3]);
  pose[4] = normalize_angle(pose[4]);
  pose[5] = normalize_angle(pose[5]);
  return pose;
}

[[nodiscard]] std::vector<double> apply_offset(const std::vector<double> &pose,
                                               int offset_type,
                                               const std::array<double, 6> &offset_pose) {
  if (offset_type == kOffsetNone) {
    return pose;
  }
  const auto offset = array_pose_to_vector(offset_pose);
  if (offset_type == kOffsetOffs) {
    auto result = pose;
    for (size_t i = 0; i < result.size(); ++i) {
      result[i] += offset[i];
    }
    return result;
  }
  return transform_to_pose(pose_to_transform(pose) * pose_to_transform(offset));
}

}  // namespace

bool build_motion_request(const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
                          const MotionRequestContext &context,
                          MotionRequest &request,
                          std::string &error_message) {
  request = MotionRequest{};
  request.request_id = context.request_id;
  request.start_joints = context.start_joints;
  request.default_speed = context.default_speed;
  request.default_zone = context.default_zone;
  request.strict_conf = context.strict_conf;
  request.avoid_singularity = context.avoid_singularity;
  request.soft_limit_enabled = context.soft_limit_enabled;
  request.soft_limits = context.soft_limits;
  request.trajectory_dt = context.trajectory_dt;

  auto append_absj = [&](const auto &cmd) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_absj;
    spec.target_joints.assign(cmd.target.joints.begin(), cmd.target.joints.end());
    spec.speed = cmd.speed > 0 ? cmd.speed : context.default_speed;
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  };
  for (const auto &cmd : goal.absj_cmds) {
    append_absj(cmd);
  }

  for (const auto &cmd : goal.j_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_j;
    spec.target_cartesian = apply_offset(msg_pose_to_vector(cmd.target), cmd.offset_type, cmd.offset_pose);
    spec.requested_conf = cmd.target.conf_data;
    spec.speed = cmd.speed > 0 ? cmd.speed : context.default_speed;
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.l_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_l;
    spec.target_cartesian = apply_offset(msg_pose_to_vector(cmd.target), cmd.offset_type, cmd.offset_pose);
    spec.requested_conf = cmd.target.conf_data;
    spec.speed = cmd.speed > 0 ? cmd.speed : context.default_speed;
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.c_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_c;
    spec.target_cartesian = apply_offset(msg_pose_to_vector(cmd.target), cmd.target_offset_type, cmd.target_offset_pose);
    spec.aux_cartesian = apply_offset(msg_pose_to_vector(cmd.aux), cmd.aux_offset_type, cmd.aux_offset_pose);
    spec.requested_conf = cmd.target.conf_data;
    spec.speed = cmd.speed > 0 ? cmd.speed : context.default_speed;
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.cf_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_cf;
    spec.target_cartesian = apply_offset(msg_pose_to_vector(cmd.target), cmd.target_offset_type, cmd.target_offset_pose);
    spec.aux_cartesian = apply_offset(msg_pose_to_vector(cmd.aux), cmd.aux_offset_type, cmd.aux_offset_pose);
    spec.requested_conf = cmd.target.conf_data;
    spec.speed = cmd.speed > 0 ? cmd.speed : context.default_speed;
    spec.zone = cmd.zone;
    spec.angle = cmd.angle;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.sp_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_sp;
    spec.target_cartesian = apply_offset(msg_pose_to_vector(cmd.target), cmd.offset_type, cmd.offset_pose);
    spec.requested_conf = cmd.target.conf_data;
    spec.speed = cmd.speed > 0 ? cmd.speed : context.default_speed;
    spec.zone = cmd.zone;
    spec.radius = cmd.radius;
    spec.radius_step = cmd.radius_step;
    spec.angle = cmd.angle;
    spec.direction = cmd.direction;
    request.commands.push_back(std::move(spec));
  }

  if (request.commands.empty()) {
    error_message = "MoveAppend request is empty";
    return false;
  }
  if (request.start_joints.size() < 6) {
    error_message = "current joint snapshot is incomplete";
    return false;
  }
  return true;
}

bool build_replay_request(const std::vector<std::vector<double>> &recorded_path,
                          double rate,
                          const MotionRequestContext &context,
                          MotionRequest &request,
                          std::string &error_message) {
  if (recorded_path.empty()) {
    error_message = "Path is empty";
    return false;
  }
  if (context.start_joints.size() < 6) {
    error_message = "current joint snapshot is incomplete";
    return false;
  }

  request = MotionRequest{};
  request.request_id = context.request_id;
  request.start_joints = context.start_joints;
  request.default_speed = context.default_speed;
  request.default_zone = context.default_zone;
  request.strict_conf = context.strict_conf;
  request.avoid_singularity = context.avoid_singularity;
  request.soft_limit_enabled = context.soft_limit_enabled;
  request.soft_limits = context.soft_limits;
  request.trajectory_dt = context.trajectory_dt;

  MotionCommandSpec cmd;
  cmd.kind = MotionKind::move_absj;
  cmd.use_preplanned_trajectory = true;
  cmd.preplanned_trajectory = recorded_path;
  cmd.preplanned_dt = std::max(context.trajectory_dt, 1e-3) / std::max(rate, 0.05);
  cmd.target_joints = recorded_path.back();
  cmd.speed = static_cast<int>(context.default_speed * rate);
  cmd.zone = context.default_zone;
  request.commands.push_back(std::move(cmd));
  return true;
}

}  // namespace rokae_xmate3_ros2::runtime
