#include "runtime/request_adapter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "runtime/pose_utils.hpp"
#include "runtime/runtime_state.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr int kOffsetNone = 0;
constexpr int kOffsetOffs = 1;

constexpr double kMinNrtSpeedMmPerSec = 5.0;
constexpr double kMaxNrtSpeedMmPerSec = 4000.0;

[[nodiscard]] double resolve_nrt_speed_mm_per_s(double base_speed, double speed_scale) {
  const double scaled_speed = std::max(base_speed, kMinNrtSpeedMmPerSec) *
                              std::clamp(speed_scale, 0.05, 2.0);
  return std::clamp(scaled_speed, kMinNrtSpeedMmPerSec, kMaxNrtSpeedMmPerSec);
}

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

[[nodiscard]] std::vector<double> to_flange_pose_in_base(const std::vector<double> &pose,
                                                         const MotionRequestContext &context) {
  return pose_utils::convertEndInRefToFlangeInBase(pose, context.tool_pose, context.wobj_pose);
}

struct ReplayVectorSample {
  double time_from_start = 0.0;
  std::vector<double> position;
  std::vector<double> velocity;
};

[[nodiscard]] double sanitize_replay_time(double requested,
                                          double fallback_time,
                                          double min_step) {
  if (!std::isfinite(requested)) {
    return fallback_time;
  }
  return std::max(requested, fallback_time - min_step);
}

[[nodiscard]] std::vector<ReplayVectorSample> normalize_replay_samples(
    const ReplayPathAsset &asset,
    double fallback_dt) {
  std::vector<ReplayVectorSample> samples;
  samples.reserve(asset.samples.size());

  const double min_step = std::max(fallback_dt, 1e-3);
  double next_fallback_time = 0.0;
  double last_time = -min_step;
  for (const auto &sample : asset.samples) {
    ReplayVectorSample normalized;
    normalized.position.assign(sample.joint_position.begin(), sample.joint_position.end());
    normalized.velocity.assign(sample.joint_velocity.begin(), sample.joint_velocity.end());

    double time_from_start = sanitize_replay_time(sample.time_from_start_sec, next_fallback_time, min_step);
    if (time_from_start <= last_time) {
      time_from_start = last_time + min_step;
    }
    normalized.time_from_start = time_from_start;

    next_fallback_time = time_from_start + min_step;
    last_time = time_from_start;
    samples.push_back(std::move(normalized));
  }
  return samples;
}

[[nodiscard]] std::vector<double> interpolate_joint_vector(const std::vector<ReplayVectorSample> &samples,
                                                           double time_from_start,
                                                           bool velocity_space) {
  if (samples.empty()) {
    return {};
  }
  const auto &first = velocity_space ? samples.front().velocity : samples.front().position;
  if (samples.size() == 1 || time_from_start <= samples.front().time_from_start) {
    return first;
  }
  const auto &last = velocity_space ? samples.back().velocity : samples.back().position;
  if (time_from_start >= samples.back().time_from_start) {
    return last;
  }

  auto upper = std::lower_bound(
      samples.begin(), samples.end(), time_from_start,
      [](const ReplayVectorSample &sample, double value) { return sample.time_from_start < value; });
  if (upper == samples.begin()) {
    return first;
  }
  if (upper == samples.end()) {
    return last;
  }

  const auto &rhs = *upper;
  const auto &lhs = *(upper - 1);
  const double duration = std::max(rhs.time_from_start - lhs.time_from_start, 1e-9);
  const double alpha = std::clamp((time_from_start - lhs.time_from_start) / duration, 0.0, 1.0);

  const auto &lhs_vec = velocity_space ? lhs.velocity : lhs.position;
  const auto &rhs_vec = velocity_space ? rhs.velocity : rhs.position;
  std::vector<double> out(lhs_vec.size(), 0.0);
  for (std::size_t index = 0; index < out.size() && index < rhs_vec.size(); ++index) {
    out[index] = lhs_vec[index] + alpha * (rhs_vec[index] - lhs_vec[index]);
  }
  return out;
}

void populate_acceleration_trajectory(std::vector<std::vector<double>> &accelerations,
                                      const std::vector<std::vector<double>> &velocities,
                                      double dt) {
  accelerations.clear();
  if (velocities.empty()) {
    return;
  }
  const auto point_count = velocities.size();
  const auto axis_count = velocities.front().size();
  accelerations.assign(point_count, std::vector<double>(axis_count, 0.0));
  if (point_count < 2 || dt <= 1e-9) {
    return;
  }
  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    for (std::size_t axis = 0; axis < axis_count; ++axis) {
      if (point_index == 0) {
        accelerations[point_index][axis] =
            (velocities[1][axis] - velocities[0][axis]) / dt;
      } else if (point_index + 1 >= point_count) {
        accelerations[point_index][axis] =
            (velocities[point_index][axis] - velocities[point_index - 1][axis]) / dt;
      } else {
        accelerations[point_index][axis] =
            (velocities[point_index + 1][axis] - velocities[point_index - 1][axis]) / (2.0 * dt);
      }
    }
  }
}

}  // namespace

bool build_motion_request(const rokae_xmate3_ros2::action::MoveAppend::Goal &goal,
                          const MotionRequestContext &context,
                          MotionRequest &request,
                          std::string &error_message) {
  request = MotionRequest{};
  request.request_id = context.request_id;
  request.start_joints = context.start_joints;
  request.default_speed = resolve_nrt_speed_mm_per_s(context.default_speed, context.speed_scale);
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
    const double requested_speed = cmd.speed > 0 ? static_cast<double>(cmd.speed) : context.default_speed;
    spec.speed = resolve_nrt_speed_mm_per_s(requested_speed, context.speed_scale);
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  };
  for (const auto &cmd : goal.absj_cmds) {
    append_absj(cmd);
  }

  for (const auto &cmd : goal.j_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_j;
    spec.target_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.target), cmd.offset_type, cmd.offset_pose),
        context);
    spec.requested_conf = cmd.target.conf_data;
    const double requested_speed = cmd.speed > 0 ? static_cast<double>(cmd.speed) : context.default_speed;
    spec.speed = resolve_nrt_speed_mm_per_s(requested_speed, context.speed_scale);
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.l_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_l;
    spec.target_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.target), cmd.offset_type, cmd.offset_pose),
        context);
    spec.requested_conf = cmd.target.conf_data;
    const double requested_speed = cmd.speed > 0 ? static_cast<double>(cmd.speed) : context.default_speed;
    spec.speed = resolve_nrt_speed_mm_per_s(requested_speed, context.speed_scale);
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.c_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_c;
    spec.target_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.target), cmd.target_offset_type, cmd.target_offset_pose),
        context);
    spec.aux_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.aux), cmd.aux_offset_type, cmd.aux_offset_pose),
        context);
    spec.requested_conf = cmd.target.conf_data;
    const double requested_speed = cmd.speed > 0 ? static_cast<double>(cmd.speed) : context.default_speed;
    spec.speed = resolve_nrt_speed_mm_per_s(requested_speed, context.speed_scale);
    spec.zone = cmd.zone;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.cf_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_cf;
    spec.target_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.target), cmd.target_offset_type, cmd.target_offset_pose),
        context);
    spec.aux_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.aux), cmd.aux_offset_type, cmd.aux_offset_pose),
        context);
    spec.requested_conf = cmd.target.conf_data;
    const double requested_speed = cmd.speed > 0 ? static_cast<double>(cmd.speed) : context.default_speed;
    spec.speed = resolve_nrt_speed_mm_per_s(requested_speed, context.speed_scale);
    spec.zone = cmd.zone;
    spec.angle = cmd.angle;
    request.commands.push_back(std::move(spec));
  }

  for (const auto &cmd : goal.sp_cmds) {
    MotionCommandSpec spec;
    spec.kind = MotionKind::move_sp;
    spec.target_cartesian = to_flange_pose_in_base(
        apply_offset(msg_pose_to_vector(cmd.target), cmd.offset_type, cmd.offset_pose),
        context);
    spec.requested_conf = cmd.target.conf_data;
    const double requested_speed = cmd.speed > 0 ? static_cast<double>(cmd.speed) : context.default_speed;
    spec.speed = resolve_nrt_speed_mm_per_s(requested_speed, context.speed_scale);
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

bool build_replay_request(const ReplayPathAsset &replay_asset,
                          double rate,
                          const MotionRequestContext &context,
                          MotionRequest &request,
                          std::string &error_message) {
  if (replay_asset.samples.empty()) {
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
  request.default_speed = resolve_nrt_speed_mm_per_s(context.default_speed, context.speed_scale);
  request.default_zone = context.default_zone;
  request.strict_conf = context.strict_conf;
  request.avoid_singularity = context.avoid_singularity;
  request.soft_limit_enabled = context.soft_limit_enabled;
  request.soft_limits = context.soft_limits;
  request.trajectory_dt = context.trajectory_dt;

  const double rate_scale = std::max(rate, 0.05);
  const auto normalized_samples = normalize_replay_samples(replay_asset, context.trajectory_dt);
  if (normalized_samples.empty()) {
    error_message = "Path is empty";
    return false;
  }

  const double original_total_time = normalized_samples.back().time_from_start;
  const double scaled_total_time = original_total_time / rate_scale;
  const int interval_count =
      scaled_total_time > 1e-9
          ? std::max(1, static_cast<int>(std::ceil(scaled_total_time / std::max(context.trajectory_dt, 1e-3))))
          : 1;
  const double output_dt =
      scaled_total_time > 1e-9 ? scaled_total_time / static_cast<double>(interval_count)
                               : std::max(context.trajectory_dt, 1e-3);

  MotionCommandSpec cmd;
  cmd.kind = MotionKind::move_absj;
  cmd.use_preplanned_trajectory = true;
  cmd.preplanned_dt = output_dt;
  cmd.preplanned_trajectory.reserve(static_cast<std::size_t>(interval_count) + 1);
  cmd.preplanned_velocity_trajectory.reserve(static_cast<std::size_t>(interval_count) + 1);

  for (int index = 0; index <= interval_count; ++index) {
    const double scaled_time =
        scaled_total_time > 1e-9 ? output_dt * static_cast<double>(index) : 0.0;
    const double original_time = std::min(scaled_time * rate_scale, original_total_time);

    auto position = interpolate_joint_vector(normalized_samples, original_time, false);
    auto velocity = interpolate_joint_vector(normalized_samples, original_time, true);
    if (velocity.size() < position.size()) {
      velocity.resize(position.size(), 0.0);
    }
    for (auto &value : velocity) {
      value *= rate_scale;
    }

    cmd.preplanned_trajectory.push_back(std::move(position));
    cmd.preplanned_velocity_trajectory.push_back(std::move(velocity));
  }

  populate_acceleration_trajectory(
      cmd.preplanned_acceleration_trajectory, cmd.preplanned_velocity_trajectory, cmd.preplanned_dt);
  cmd.target_joints = cmd.preplanned_trajectory.back();
  cmd.speed = resolve_nrt_speed_mm_per_s(
      std::max(context.default_speed * rate, 1.0),
      context.speed_scale);
  cmd.zone = context.default_zone;
  request.commands.push_back(std::move(cmd));
  return true;
}

}  // namespace rokae_xmate3_ros2::runtime
