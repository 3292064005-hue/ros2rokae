#include "runtime/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "runtime/planner_preflight.hpp"
#include "runtime/planner_trace.hpp"
#include "runtime/planning_utils.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

using ::gazebo::TrajectoryPlanner;

constexpr double kMinNrtSpeedMmPerSec = 5.0;
constexpr double kMaxNrtSpeedMmPerSec = 4000.0;
constexpr double kMinBlendTrimMeters = 0.0005;
constexpr double kMinBlendAngleRad = 3.0 * M_PI / 180.0;
constexpr double kMaxBlendAngleRad = 170.0 * M_PI / 180.0;
constexpr double kBlendChordScale = 0.5;
constexpr double kPoseEpsilon = 1e-9;
constexpr double kTrimEpsilon = 1e-6;

bool move_absj_violates_soft_limit(const std::vector<double> &joints,
                                   const std::array<std::array<double, 2>, 6> &soft_limits) {
  for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
    if (joints[i] < soft_limits[i][0] || joints[i] > soft_limits[i][1]) {
      return true;
    }
  }
  return false;
}

void apply_unified_trajectory(PlannedSegment &segment, const UnifiedTrajectoryResult &trajectory) {
  segment.joint_trajectory = trajectory.samples.positions;
  segment.joint_velocity_trajectory = trajectory.samples.velocities;
  segment.joint_acceleration_trajectory = trajectory.samples.accelerations;
  segment.trajectory_dt = trajectory.samples.sample_dt;
  segment.trajectory_total_time = trajectory.samples.total_time;
}

void append_retimer_note(std::vector<std::string> &notes,
                         std::size_t segment_index,
                         const RetimerMetadata &metadata) {
  if (metadata.note == RetimerNote::nominal) {
    return;
  }
  notes.push_back(describeRetimerMetadata(metadata, "segment " + std::to_string(segment_index)));
}

enum class BlendFamily { none, joint, cartesian_lookahead };

BlendFamily blend_family(MotionKind kind) {
  switch (kind) {
    case MotionKind::move_absj:
    case MotionKind::move_j:
      return BlendFamily::joint;
    case MotionKind::move_l:
    case MotionKind::move_c:
    case MotionKind::move_cf:
    case MotionKind::move_sp:
      return BlendFamily::cartesian_lookahead;
    default:
      return BlendFamily::none;
  }
}

PathFamily path_family(MotionKind kind) {
  switch (kind) {
    case MotionKind::move_absj:
    case MotionKind::move_j:
      return PathFamily::joint;
    case MotionKind::move_l:
      return PathFamily::cartesian_line;
    case MotionKind::move_c:
      return PathFamily::cartesian_arc;
    case MotionKind::move_cf:
      return PathFamily::cartesian_continuous_circle;
    case MotionKind::move_sp:
      return PathFamily::cartesian_spiral;
    default:
      return PathFamily::none;
  }
}

bool is_cartesian_lookahead_kind(MotionKind kind) {
  return kind == MotionKind::move_l || kind == MotionKind::move_c ||
         kind == MotionKind::move_cf || kind == MotionKind::move_sp;
}

std::string mixed_mode_fallback_note(std::size_t current_index, std::size_t next_index) {
  return "mixed-mode junction between segments " + std::to_string(current_index) +
         " and " + std::to_string(next_index) + " fell back to stop-point";
}

std::string zone_fallback_note(std::size_t current_index,
                               std::size_t next_index,
                               const std::string &reason = {}) {
  std::string note = "zone blend between segments " + std::to_string(current_index) + " and " +
                     std::to_string(next_index) + " fell back to stop-point";
  if (!reason.empty()) {
    note += ": " + reason;
  }
  return note;
}

double clamp_cartesian_speed_mps(double speed_mm_per_s) {
  return std::clamp(speed_mm_per_s, kMinNrtSpeedMmPerSec, kMaxNrtSpeedMmPerSec) / 1000.0;
}

int determine_interval_count(double total_time,
                             double requested_dt,
                             double path_metric,
                             double max_path_step,
                             double orientation_metric) {
  const auto config = TrajectoryPlanner::config();
  if (total_time <= 1e-9) {
    return 1;
  }

  const double clamped_max_dt = std::clamp(requested_dt, config.min_sample_dt, config.max_sample_dt);
  const int time_intervals = std::max(1, static_cast<int>(std::ceil(total_time / clamped_max_dt)));
  const int path_intervals =
      max_path_step > 1e-9 ? std::max(1, static_cast<int>(std::ceil(path_metric / max_path_step))) : 1;
  const int orientation_intervals =
      config.max_orientation_step_rad > 1e-9
          ? std::max(1, static_cast<int>(std::ceil(orientation_metric / config.max_orientation_step_rad)))
          : 1;
  const int min_dt_intervals =
      std::max(1, static_cast<int>(std::ceil(total_time / config.min_sample_dt)));
  return std::clamp(std::max({time_intervals, path_intervals, orientation_intervals}), 1, min_dt_intervals);
}

struct PoseSample {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

PoseSample pose_from_vector(const std::vector<double> &pose) {
  const auto transform = pose_utils::poseToIsometry(pose);
  PoseSample sample;
  sample.position = transform.translation();
  sample.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return sample;
}

std::vector<double> vector_from_pose(const PoseSample &pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.orientation.normalized().toRotationMatrix();
  transform.translation() = pose.position;
  return pose_utils::isometryToPose(transform);
}

double orientation_distance(const PoseSample &lhs, const PoseSample &rhs) {
  return lhs.orientation.angularDistance(rhs.orientation);
}

double signed_angle_around_axis(const Eigen::Vector3d &from,
                                const Eigen::Vector3d &to,
                                const Eigen::Vector3d &axis) {
  const Eigen::Vector3d from_n = from.normalized();
  const Eigen::Vector3d to_n = to.normalized();
  const double sin_term = axis.normalized().dot(from_n.cross(to_n));
  const double cos_term = std::clamp(from_n.dot(to_n), -1.0, 1.0);
  return std::atan2(sin_term, cos_term);
}

bool compute_circle_params(const Eigen::Vector3d &p1,
                           const Eigen::Vector3d &p2,
                           const Eigen::Vector3d &p3,
                           Eigen::Vector3d &center,
                           double &radius,
                           Eigen::Vector3d &axis) {
  const Eigen::Vector3d u = p2 - p1;
  const Eigen::Vector3d v = p3 - p1;
  const Eigen::Vector3d w = u.cross(v);
  const double w_sq = w.squaredNorm();
  if (w_sq < 1e-12) {
    return false;
  }

  axis = w.normalized();
  const Eigen::Vector3d center_offset =
      (u.squaredNorm() * v.cross(w) + v.squaredNorm() * w.cross(u)) / (2.0 * w_sq);
  center = p1 + center_offset;
  radius = (p1 - center).norm();
  return std::isfinite(radius) && radius > 1e-9;
}

struct CartesianPrimitive {
  MotionKind kind = MotionKind::none;
  PathFamily family = PathFamily::none;
  PoseSample start_pose;
  PoseSample end_pose;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d start_radius_vector = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d local_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d spiral_axis_vector = Eigen::Vector3d::Zero();
  double spiral_axis_length = 0.0;
  double spiral_radius = 0.0;
  double spiral_radius_step = 0.0;
  double spiral_radius_angle = 0.0;
  double total_angle = 0.0;
  double length_m = 0.0;

  PoseSample sampleAtArcLength(double arc_length) const {
    const double clamped_arc = std::clamp(arc_length, 0.0, length_m);
    const double alpha = length_m > kPoseEpsilon ? clamped_arc / length_m : 0.0;

    PoseSample sample;
    if (kind == MotionKind::move_l) {
      sample.position = start_pose.position + alpha * (end_pose.position - start_pose.position);
    } else if (kind == MotionKind::move_sp) {
      const double current_angle = alpha * total_angle;
      const double current_radius = spiral_radius + spiral_radius_step * alpha * spiral_radius_angle;
      const Eigen::Vector3d axis_position = start_pose.position + alpha * spiral_axis_vector;
      sample.position =
          axis_position +
          current_radius * (std::cos(current_angle) * local_x + std::sin(current_angle) * local_y);
    } else {
      const double current_angle = alpha * total_angle;
      sample.position = center + Eigen::AngleAxisd(current_angle, axis) * start_radius_vector;
    }
    sample.orientation = start_pose.orientation.slerp(alpha, end_pose.orientation).normalized();
    return sample;
  }

  Eigen::Vector3d tangentAtArcLength(double arc_length) const {
    if (kind == MotionKind::move_l) {
      const Eigen::Vector3d delta = end_pose.position - start_pose.position;
      return delta.norm() > kPoseEpsilon ? delta.normalized() : Eigen::Vector3d::UnitX();
    }
    if (kind == MotionKind::move_sp) {
      const double clamped_arc = std::clamp(arc_length, 0.0, length_m);
      const double alpha = length_m > kPoseEpsilon ? clamped_arc / length_m : 0.0;
      const double current_angle = alpha * total_angle;
      const double current_radius = spiral_radius + spiral_radius_step * alpha * spiral_radius_angle;
      const double radius_rate = spiral_radius_step * spiral_radius_angle;
      const Eigen::Vector3d radial =
          std::cos(current_angle) * local_x + std::sin(current_angle) * local_y;
      const Eigen::Vector3d tangential =
          -std::sin(current_angle) * local_x + std::cos(current_angle) * local_y;
      Eigen::Vector3d tangent =
          spiral_axis_vector + radius_rate * radial + current_radius * total_angle * tangential;
      return tangent.norm() > kPoseEpsilon ? tangent.normalized() : Eigen::Vector3d::UnitX();
    }

    const double clamped_arc = std::clamp(arc_length, 0.0, length_m);
    const double alpha = length_m > kPoseEpsilon ? clamped_arc / length_m : 0.0;
    const Eigen::Vector3d radius_vector =
        Eigen::AngleAxisd(alpha * total_angle, axis) * start_radius_vector;
    Eigen::Vector3d tangent = axis.cross(radius_vector);
    if (total_angle < 0.0) {
      tangent = -tangent;
    }
    return tangent.norm() > kPoseEpsilon ? tangent.normalized() : Eigen::Vector3d::UnitX();
  }
};

bool build_line_primitive(const PoseSample &start_pose,
                          const std::vector<double> &target_pose,
                          CartesianPrimitive &primitive,
                          std::string &error_message) {
  primitive.kind = MotionKind::move_l;
  primitive.family = PathFamily::cartesian_line;
  primitive.start_pose = start_pose;
  primitive.end_pose = pose_from_vector(target_pose);
  primitive.length_m = (primitive.end_pose.position - primitive.start_pose.position).norm();
  if (primitive.length_m < kPoseEpsilon &&
      orientation_distance(primitive.start_pose, primitive.end_pose) < kPoseEpsilon) {
    primitive.length_m = 0.0;
    error_message.clear();
    return true;
  }
  return true;
}

bool build_spiral_primitive(const PoseSample &start_pose,
                            const std::vector<double> &target_pose,
                            double radius,
                            double radius_step,
                            double angle,
                            bool direction,
                            CartesianPrimitive &primitive,
                            std::string &error_message) {
  if (std::abs(angle) < 1e-6 || radius < 0.0) {
    error_message = "MoveSP parameters are invalid";
    return false;
  }

  primitive.kind = MotionKind::move_sp;
  primitive.family = PathFamily::cartesian_spiral;
  primitive.start_pose = start_pose;
  primitive.end_pose = pose_from_vector(target_pose);

  const Eigen::Vector3d axis_end(target_pose[0], target_pose[1], target_pose[2]);
  primitive.spiral_axis_vector = axis_end - primitive.start_pose.position;
  primitive.spiral_axis_length = primitive.spiral_axis_vector.norm();
  if (primitive.spiral_axis_length < kPoseEpsilon) {
    error_message = "MoveSP spiral axis is too short";
    return false;
  }

  primitive.axis = primitive.spiral_axis_vector.normalized();
  if (std::abs(primitive.axis.dot(Eigen::Vector3d::UnitZ())) < 0.999) {
    primitive.local_x = primitive.axis.cross(Eigen::Vector3d::UnitZ()).normalized();
  } else {
    primitive.local_x = primitive.axis.cross(Eigen::Vector3d::UnitX()).normalized();
  }
  primitive.local_y = primitive.axis.cross(primitive.local_x).normalized();
  primitive.spiral_radius = radius;
  primitive.spiral_radius_step = radius_step;
  primitive.spiral_radius_angle = angle;
  primitive.total_angle = direction ? angle : -angle;

  const double average_radius = radius + radius_step * angle * 0.5;
  primitive.length_m = primitive.spiral_axis_length + std::abs(average_radius * angle);
  if (primitive.length_m < kPoseEpsilon) {
    error_message = "MoveSP spiral length is too short";
    return false;
  }

  primitive.end_pose = primitive.sampleAtArcLength(primitive.length_m);
  primitive.end_pose.orientation = pose_from_vector(target_pose).orientation;
  return true;
}

bool build_arc_primitive(const PoseSample &start_pose,
                         const std::vector<double> &aux_pose,
                         const std::vector<double> &target_pose,
                         std::optional<double> explicit_angle,
                         MotionKind kind,
                         CartesianPrimitive &primitive,
                         std::string &error_message) {
  primitive.kind = kind;
  primitive.family =
      kind == MotionKind::move_cf ? PathFamily::cartesian_continuous_circle : PathFamily::cartesian_arc;
  primitive.start_pose = start_pose;
  primitive.end_pose = pose_from_vector(target_pose);

  const PoseSample aux = pose_from_vector(aux_pose);
  const Eigen::Vector3d p1 = primitive.start_pose.position;
  const Eigen::Vector3d p2 = aux.position;
  const Eigen::Vector3d p3 = primitive.end_pose.position;

  double radius = 0.0;
  if (!compute_circle_params(p1, p2, p3, primitive.center, radius, primitive.axis)) {
    error_message = kind == MotionKind::move_cf ? "MoveCF circle parameters are invalid"
                                                : "MoveC circle parameters are invalid";
    return false;
  }

  primitive.start_radius_vector = p1 - primitive.center;
  if (primitive.start_radius_vector.norm() < kPoseEpsilon) {
    error_message = kind == MotionKind::move_cf ? "MoveCF start radius vector is invalid"
                                                : "MoveC start radius vector is invalid";
    return false;
  }

  if (explicit_angle.has_value()) {
    primitive.total_angle = *explicit_angle;
  } else {
    Eigen::Vector3d v_aux = p2 - primitive.center;
    Eigen::Vector3d v_end = p3 - primitive.center;
    double aux_angle = signed_angle_around_axis(primitive.start_radius_vector, v_aux, primitive.axis);
    double total_angle = signed_angle_around_axis(primitive.start_radius_vector, v_end, primitive.axis);
    if (aux_angle < 0.0) {
      aux_angle += 2.0 * M_PI;
    }
    if (total_angle < 0.0) {
      total_angle += 2.0 * M_PI;
    }
    if (total_angle < aux_angle) {
      total_angle += 2.0 * M_PI;
    }
    primitive.total_angle = total_angle;
  }

  if (std::abs(primitive.total_angle) < kPoseEpsilon) {
    error_message = kind == MotionKind::move_cf ? "MoveCF sweep angle is too small"
                                                : "MoveC sweep angle is too small";
    return false;
  }

  primitive.length_m = radius * std::abs(primitive.total_angle);
  return primitive.length_m > kPoseEpsilon;
}

struct HermiteBlendCurve {
  PoseSample start_pose;
  PoseSample end_pose;
  Eigen::Vector3d start_tangent = Eigen::Vector3d::UnitX();
  Eigen::Vector3d end_tangent = Eigen::Vector3d::UnitX();
  double chord_length_m = 0.0;
  double length_m = 0.0;

  PoseSample sample(double u) const {
    const double clamped_u = std::clamp(u, 0.0, 1.0);
    const double u2 = clamped_u * clamped_u;
    const double u3 = u2 * clamped_u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + clamped_u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;
    const double tangent_scale = kBlendChordScale * chord_length_m;

    PoseSample pose;
    pose.position = h00 * start_pose.position +
                    h10 * tangent_scale * start_tangent +
                    h01 * end_pose.position +
                    h11 * tangent_scale * end_tangent;
    pose.orientation =
        start_pose.orientation.slerp(clamped_u, end_pose.orientation).normalized();
    return pose;
  }
};

double estimate_blend_length(HermiteBlendCurve &curve) {
  constexpr int kSamples = 24;
  double length = 0.0;
  PoseSample previous = curve.sample(0.0);
  for (int index = 1; index <= kSamples; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(kSamples);
    PoseSample current = curve.sample(u);
    length += (current.position - previous.position).norm();
    previous = current;
  }
  curve.length_m = length;
  return curve.length_m;
}

struct CartesianRunSegment {
  PlannedSegment segment;
  CartesianPrimitive primitive;
  std::vector<int> requested_conf;
  double speed_mps = 0.0;
  double blend_speed_mps = 0.0;
  std::optional<HermiteBlendCurve> blend_to_next_curve;
};

bool try_build_path_blend(CartesianRunSegment &current,
                          CartesianRunSegment &next,
                          std::string &reason) {
  const auto outgoing = current.primitive.tangentAtArcLength(current.primitive.length_m);
  const auto incoming = next.primitive.tangentAtArcLength(0.0);
  const double dot = std::clamp(outgoing.dot(incoming), -1.0, 1.0);
  const double turn_angle = std::acos(dot);
  if (turn_angle < kMinBlendAngleRad) {
    reason = "junction angle too small";
    return false;
  }
  if (turn_angle > kMaxBlendAngleRad) {
    reason = "junction angle too sharp";
    return false;
  }

  const double zone_m = static_cast<double>(current.segment.zone) / 1000.0;
  const double raw_trim = zone_m * std::tan(turn_angle * 0.5);
  const double trim_limit = 0.4 * std::min(current.primitive.length_m, next.primitive.length_m);
  const double trim = std::clamp(raw_trim, 0.0, trim_limit);
  if (trim < kMinBlendTrimMeters) {
    reason = "trim below minimum";
    return false;
  }

  const double current_available = current.primitive.length_m - current.segment.path_entry_trim_m;
  const double next_available = next.primitive.length_m - next.segment.path_exit_trim_m;
  if (current_available <= 2.0 * trim + kTrimEpsilon) {
    reason = "insufficient remaining length in current segment";
    return false;
  }
  if (next_available <= 2.0 * trim + kTrimEpsilon) {
    reason = "insufficient remaining length in next segment";
    return false;
  }

  HermiteBlendCurve curve;
  curve.start_pose = current.primitive.sampleAtArcLength(current.primitive.length_m - trim);
  curve.end_pose = next.primitive.sampleAtArcLength(trim);
  curve.start_tangent =
      current.primitive.tangentAtArcLength(current.primitive.length_m - trim).normalized();
  curve.end_tangent = next.primitive.tangentAtArcLength(trim).normalized();
  curve.chord_length_m = (curve.end_pose.position - curve.start_pose.position).norm();
  if (curve.chord_length_m < kMinBlendTrimMeters) {
    reason = "blend chord too short";
    return false;
  }
  if (estimate_blend_length(curve) < kMinBlendTrimMeters) {
    reason = "blend curve too short";
    return false;
  }

  current.segment.path_exit_trim_m = trim;
  next.segment.path_entry_trim_m = trim;
  current.segment.path_blended = true;
  next.segment.path_blended = true;
  current.segment.blend_to_next = true;
  current.segment.blend_length_m = curve.length_m;
  current.blend_speed_mps = std::min(current.speed_mps, next.speed_mps);
  current.blend_to_next_curve = curve;
  return true;
}

PoseSample segment_final_pose(const CartesianRunSegment &segment) {
  if (segment.blend_to_next_curve.has_value()) {
    return segment.blend_to_next_curve->end_pose;
  }
  return segment.primitive.sampleAtArcLength(segment.primitive.length_m - segment.segment.path_exit_trim_m);
}

bool sample_cartesian_segment(CartesianRunSegment &segment,
                              double requested_dt,
                              std::vector<std::vector<double>> &cartesian_points) {
  cartesian_points.clear();

  const double core_length =
      std::max(segment.primitive.length_m - segment.segment.path_entry_trim_m - segment.segment.path_exit_trim_m, 0.0);
  const double blend_length = segment.blend_to_next_curve.has_value() ? segment.blend_to_next_curve->length_m : 0.0;
  const double total_path_length = core_length + blend_length;
  segment.segment.path_length_m = total_path_length;

  const double core_time = core_length > kPoseEpsilon ? core_length / segment.speed_mps : 0.0;
  const double blend_speed =
      segment.blend_to_next_curve.has_value() ? segment.blend_speed_mps : segment.speed_mps;
  const double blend_time =
      (segment.blend_to_next_curve.has_value() && blend_length > kPoseEpsilon && blend_speed > kPoseEpsilon)
          ? blend_length / blend_speed
          : 0.0;
  const double total_time = core_time + blend_time;
  const auto planner_config = TrajectoryPlanner::config();

  const PoseSample start_pose = segment.primitive.sampleAtArcLength(segment.segment.path_entry_trim_m);
  const PoseSample final_pose = segment_final_pose(segment);
  const double orientation_metric = orientation_distance(start_pose, final_pose);
  const int interval_count =
      determine_interval_count(
          total_time, requested_dt, total_path_length, planner_config.max_cartesian_step_m, orientation_metric);
  const double resolved_dt =
      std::clamp(requested_dt, planner_config.min_sample_dt, planner_config.max_sample_dt);

  if (total_path_length <= kPoseEpsilon || total_time <= kPoseEpsilon || interval_count <= 0) {
    cartesian_points.push_back(vector_from_pose(final_pose));
    segment.segment.trajectory_total_time = 0.0;
    segment.segment.trajectory_dt = resolved_dt;
    return true;
  }

  const double quantized_total_time = static_cast<double>(interval_count) * resolved_dt;
  cartesian_points.reserve(static_cast<std::size_t>(interval_count) + 1);
  for (int index = 0; index <= interval_count; ++index) {
    const double current_time = static_cast<double>(index) * resolved_dt;
    PoseSample sample;
    if (segment.blend_to_next_curve.has_value() && current_time > core_time + kPoseEpsilon) {
      const double u = blend_time > kPoseEpsilon ? (current_time - core_time) / blend_time : 1.0;
      sample = segment.blend_to_next_curve->sample(u);
    } else if (core_time > kPoseEpsilon) {
      const double progress = std::clamp(current_time / core_time, 0.0, 1.0);
      const double arc_length = segment.segment.path_entry_trim_m + progress * core_length;
      sample = segment.primitive.sampleAtArcLength(arc_length);
    } else {
      sample = segment.blend_to_next_curve.has_value()
                   ? segment.blend_to_next_curve->sample(0.0)
                   : final_pose;
    }
    cartesian_points.push_back(vector_from_pose(sample));
  }

  segment.segment.trajectory_total_time = quantized_total_time;
  segment.segment.trajectory_dt = resolved_dt;
  return true;
}

bool build_cartesian_run(::gazebo::xMate3Kinematics &kinematics,
                         const MotionRequest &request,
                         const std::vector<MotionCommandSpec> &commands,
                         std::size_t global_offset,
                         std::vector<double> &current_joints,
                         std::vector<PlannedSegment> &segments,
                         std::vector<std::string> &notes,
                         std::string &error_message) {
  std::vector<CartesianRunSegment> run_segments;
  run_segments.reserve(commands.size());

  PoseSample current_pose = pose_from_vector(kinematics.forwardKinematicsRPY(current_joints));
  for (const auto &cmd : commands) {
    CartesianRunSegment run_segment;
    run_segment.segment.kind = cmd.kind;
    run_segment.segment.speed = cmd.speed > 0.0 ? cmd.speed : request.default_speed;
    run_segment.segment.zone = cmd.zone;
    run_segment.segment.trajectory_dt = request.trajectory_dt;
    run_segment.segment.target_cartesian = cmd.target_cartesian;
    run_segment.segment.aux_cartesian = cmd.aux_cartesian;
    run_segment.segment.path_family = path_family(cmd.kind);
    run_segment.requested_conf = cmd.requested_conf;
    run_segment.speed_mps = clamp_cartesian_speed_mps(run_segment.segment.speed);

    bool primitive_ok = false;
    switch (cmd.kind) {
      case MotionKind::move_l:
        primitive_ok =
            build_line_primitive(current_pose, cmd.target_cartesian, run_segment.primitive, error_message);
        break;
      case MotionKind::move_c:
        primitive_ok = build_arc_primitive(current_pose,
                                           cmd.aux_cartesian,
                                           cmd.target_cartesian,
                                           std::nullopt,
                                           cmd.kind,
                                           run_segment.primitive,
                                           error_message);
        break;
      case MotionKind::move_cf:
        primitive_ok = build_arc_primitive(current_pose,
                                           cmd.aux_cartesian,
                                           cmd.target_cartesian,
                                           cmd.angle,
                                           cmd.kind,
                                           run_segment.primitive,
                                           error_message);
        break;
      case MotionKind::move_sp:
        primitive_ok = build_spiral_primitive(current_pose,
                                              cmd.target_cartesian,
                                              cmd.radius,
                                              cmd.radius_step,
                                              cmd.angle,
                                              cmd.direction,
                                              run_segment.primitive,
                                              error_message);
        break;
      default:
        error_message = "unsupported Cartesian lookahead motion kind";
        return false;
    }

    if (!primitive_ok) {
      return false;
    }
    current_pose = run_segment.primitive.end_pose;
    run_segments.push_back(std::move(run_segment));
  }

  for (std::size_t index = 0; index + 1 < run_segments.size(); ++index) {
    if (run_segments[index].segment.zone <= 0) {
      continue;
    }
    std::string reason;
    if (!try_build_path_blend(run_segments[index], run_segments[index + 1], reason)) {
      notes.push_back(zone_fallback_note(global_offset + index, global_offset + index + 1, reason));
    }
  }

  std::vector<double> ik_seed = current_joints;
  for (auto &run_segment : run_segments) {
    run_segment.segment.path_blended =
        run_segment.segment.path_blended ||
        run_segment.segment.path_entry_trim_m > 0.0 ||
        run_segment.segment.path_exit_trim_m > 0.0 ||
        run_segment.segment.blend_to_next;

    std::vector<std::vector<double>> cartesian_points;
    if (!sample_cartesian_segment(run_segment, request.trajectory_dt, cartesian_points)) {
      error_message = "failed to sample Cartesian segment";
      return false;
    }

    std::vector<double> last_joints;
    if (!build_joint_trajectory_from_cartesian(kinematics,
                                               cartesian_points,
                                               ik_seed,
                                               run_segment.requested_conf,
                                               request.strict_conf,
                                               request.avoid_singularity,
                                               request.soft_limit_enabled,
                                               request.soft_limits,
                                               run_segment.segment.joint_trajectory,
                                               last_joints,
                                               error_message)) {
      return false;
    }

    const auto retimed = retimeJointPathWithUnifiedSpeed(
        run_segment.segment.joint_trajectory,
        run_segment.segment.trajectory_dt,
        run_segment.segment.speed,
        RetimerSourceFamily::cartesian,
        request.speed_scale);
    if (retimed.empty()) {
      error_message = retimed.samples.error_message.empty() ? "failed to retime Cartesian joint path"
                                                            : retimed.samples.error_message;
      return false;
    }
    apply_unified_trajectory(run_segment.segment, retimed);
    append_retimer_note(notes, global_offset + segments.size(), retimed.metadata);
    run_segment.segment.target_joints = last_joints;
    run_segment.segment.target_cartesian = vector_from_pose(segment_final_pose(run_segment));
    const auto retimed_cartesian_points =
        resampleCartesianPosePath(cartesian_points, run_segment.segment.joint_trajectory.size());
    if (!project_joint_derivatives_from_cartesian(kinematics,
                                                  retimed_cartesian_points,
                                                  run_segment.segment.joint_trajectory,
                                                  run_segment.segment.trajectory_dt,
                                                  run_segment.segment.joint_velocity_trajectory,
                                                  run_segment.segment.joint_acceleration_trajectory)) {
      run_segment.segment.joint_velocity_trajectory = retimed.samples.velocities;
      run_segment.segment.joint_acceleration_trajectory = retimed.samples.accelerations;
    }
    ik_seed = last_joints;
    current_joints = last_joints;
    segments.push_back(std::move(run_segment.segment));
  }

  return true;
}

void populate_joint_derivatives(PlannedSegment &segment) {
  if (segment.joint_trajectory.empty()) {
    segment.joint_velocity_trajectory.clear();
    segment.joint_acceleration_trajectory.clear();
    return;
  }

  if (segment.joint_velocity_trajectory.size() == segment.joint_trajectory.size() &&
      segment.joint_acceleration_trajectory.size() == segment.joint_trajectory.size()) {
    return;
  }

  segment.joint_velocity_trajectory.clear();
  segment.joint_acceleration_trajectory.clear();

  const auto point_count = segment.joint_trajectory.size();
  const auto axis_count = segment.joint_trajectory.front().size();
  segment.joint_velocity_trajectory.assign(point_count, std::vector<double>(axis_count, 0.0));
  segment.joint_acceleration_trajectory.assign(point_count, std::vector<double>(axis_count, 0.0));

  if (point_count < 2 || segment.trajectory_dt <= 1e-9) {
    return;
  }

  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    for (std::size_t axis = 0; axis < axis_count; ++axis) {
      if (point_index == 0) {
        segment.joint_velocity_trajectory[point_index][axis] =
            (segment.joint_trajectory[1][axis] - segment.joint_trajectory[0][axis]) / segment.trajectory_dt;
      } else if (point_index + 1 >= point_count) {
        segment.joint_velocity_trajectory[point_index][axis] =
            (segment.joint_trajectory[point_index][axis] - segment.joint_trajectory[point_index - 1][axis]) /
            segment.trajectory_dt;
      } else {
        segment.joint_velocity_trajectory[point_index][axis] =
            (segment.joint_trajectory[point_index + 1][axis] - segment.joint_trajectory[point_index - 1][axis]) /
            (2.0 * segment.trajectory_dt);
      }
    }
  }

  if (point_count < 3) {
    return;
  }
  for (std::size_t point_index = 0; point_index < point_count; ++point_index) {
    for (std::size_t axis = 0; axis < axis_count; ++axis) {
      if (point_index == 0) {
        segment.joint_acceleration_trajectory[point_index][axis] =
            (segment.joint_trajectory[2][axis] - 2.0 * segment.joint_trajectory[1][axis] +
             segment.joint_trajectory[0][axis]) /
            (segment.trajectory_dt * segment.trajectory_dt);
      } else if (point_index + 1 >= point_count) {
        segment.joint_acceleration_trajectory[point_index][axis] =
            (segment.joint_trajectory[point_count - 1][axis] - 2.0 * segment.joint_trajectory[point_count - 2][axis] +
             segment.joint_trajectory[point_count - 3][axis]) /
            (segment.trajectory_dt * segment.trajectory_dt);
      } else {
        segment.joint_acceleration_trajectory[point_index][axis] =
            (segment.joint_trajectory[point_index + 1][axis] - 2.0 * segment.joint_trajectory[point_index][axis] +
             segment.joint_trajectory[point_index - 1][axis]) /
            (segment.trajectory_dt * segment.trajectory_dt);
      }
    }
  }
}

std::vector<double> hermite_joint_blend(const std::vector<double> &p0,
                                        const std::vector<double> &p1,
                                        const std::vector<double> &m0,
                                        const std::vector<double> &m1,
                                        double s,
                                        double duration) {
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;

  std::vector<double> point(p0.size(), 0.0);
  for (size_t i = 0; i < p0.size() && i < p1.size() && i < m0.size() && i < m1.size(); ++i) {
    point[i] = h00 * p0[i] + h10 * duration * m0[i] + h01 * p1[i] + h11 * duration * m1[i];
  }
  return point;
}

bool blend_joint_segment_pair(PlannedSegment &current, PlannedSegment &next) {
  if (current.zone <= 0) {
    return false;
  }
  if (current.joint_trajectory.size() < 4 || next.joint_trajectory.size() < 4) {
    return false;
  }

  const double current_dt = current.trajectory_dt;
  const double next_dt = next.trajectory_dt;
  if (current_dt <= 1e-9 || next_dt <= 1e-9) {
    return false;
  }
  const double effective_dt = std::min(current_dt, next_dt);
  const double effective_speed_mm_per_s = std::max(std::min(current.speed, next.speed), 5.0);
  const double blend_time =
      std::clamp(static_cast<double>(current.zone) / effective_speed_mm_per_s, 0.02, 0.18);
  int blend_samples = static_cast<int>(std::lround(blend_time / effective_dt));
  blend_samples = std::max(blend_samples, 2);
  blend_samples = std::min(blend_samples, static_cast<int>(current.joint_trajectory.size()) - 3);
  blend_samples = std::min(blend_samples, static_cast<int>(next.joint_trajectory.size()) - 3);
  if (blend_samples < 2) {
    return false;
  }

  const size_t current_keep = current.joint_trajectory.size() - 1 - static_cast<size_t>(blend_samples);
  const size_t next_start = static_cast<size_t>(blend_samples);
  if (current_keep < 1 || next_start + 1 >= next.joint_trajectory.size()) {
    return false;
  }

  const auto &p0 = current.joint_trajectory[current_keep];
  const auto &p1 = next.joint_trajectory[next_start];
  const auto &prev_before = current.joint_trajectory[current_keep - 1];
  const auto &prev_after = current.joint_trajectory[current_keep + 1];
  const auto &next_before = next.joint_trajectory[next_start - 1];
  const auto &next_after = next.joint_trajectory[next_start + 1];

  std::vector<double> m0(p0.size(), 0.0);
  std::vector<double> m1(p1.size(), 0.0);
  for (size_t i = 0; i < p0.size(); ++i) {
    m0[i] = (prev_after[i] - prev_before[i]) / (2.0 * effective_dt);
    m1[i] = (next_after[i] - next_before[i]) / (2.0 * effective_dt);
  }

  const int bridge_samples = blend_samples * 2;
  const double bridge_duration = bridge_samples * effective_dt;

  std::vector<std::vector<double>> merged_current;
  merged_current.reserve(current_keep + bridge_samples + 2);
  merged_current.insert(merged_current.end(),
                        current.joint_trajectory.begin(),
                        current.joint_trajectory.begin() + static_cast<long>(current_keep) + 1);
  for (int step = 1; step <= bridge_samples; ++step) {
    const double s = static_cast<double>(step) / static_cast<double>(bridge_samples);
    merged_current.push_back(hermite_joint_blend(p0, p1, m0, m1, s, bridge_duration));
  }

  std::vector<std::vector<double>> trimmed_next;
  trimmed_next.reserve(next.joint_trajectory.size() - next_start);
  trimmed_next.insert(trimmed_next.end(),
                      next.joint_trajectory.begin() + static_cast<long>(next_start),
                      next.joint_trajectory.end());
  if (trimmed_next.size() < 2) {
    return false;
  }

  current.joint_trajectory = std::move(merged_current);
  current.trajectory_dt = effective_dt;
  current.trajectory_total_time =
      (current.joint_trajectory.size() > 1) ? (current.joint_trajectory.size() - 1) * current.trajectory_dt : 0.0;
  current.target_joints = current.joint_trajectory.back();
  current.blend_to_next = true;

  next.joint_trajectory = std::move(trimmed_next);
  next.trajectory_dt = effective_dt;
  next.trajectory_total_time =
      (next.joint_trajectory.size() > 1) ? (next.joint_trajectory.size() - 1) * next.trajectory_dt : 0.0;
  return true;
}

void apply_joint_zone_blending(std::vector<PlannedSegment> &segments, std::vector<std::string> &notes) {
  if (segments.size() < 2) {
    return;
  }

  for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
    if (segments[i].zone <= 0) {
      continue;
    }
    const auto current_family = blend_family(segments[i].kind);
    const auto next_family = blend_family(segments[i + 1].kind);

    if (current_family == BlendFamily::joint && next_family == BlendFamily::joint) {
      if (!blend_joint_segment_pair(segments[i], segments[i + 1])) {
        notes.push_back(zone_fallback_note(i, i + 1));
      }
      continue;
    }

    if (current_family != BlendFamily::none && next_family != BlendFamily::none &&
        current_family != next_family) {
      notes.insert(notes.begin(), mixed_mode_fallback_note(i, i + 1));
      continue;
    }

    if (current_family == BlendFamily::cartesian_lookahead && next_family == BlendFamily::none) {
      notes.insert(notes.begin(), mixed_mode_fallback_note(i, i + 1));
    }
  }
}

}  // namespace

MotionPlanner::MotionPlanner() : kinematics_(std::make_unique<::gazebo::xMate3Kinematics>()) {}

MotionPlan MotionPlanner::plan(const MotionRequest &request) const {
  MotionPlan plan;
  plan.request_id = request.request_id;

  const auto preflight = runPlannerPreflight(request);
  if (!preflight.ok) {
    plan.error_message = format_motion_failure(preflight.reject_reason, preflight.detail);
    appendPlannerTrace(plan.notes, preflight, request.request_id);
    return plan;
  }
  appendPlannerTrace(plan.notes, preflight, request.request_id);

  auto current_joints = request.start_joints;
  plan.segments.reserve(request.commands.size());

  std::size_t index = 0;
  while (index < request.commands.size()) {
    const auto &cmd = request.commands[index];

    if (!cmd.use_preplanned_trajectory && is_cartesian_lookahead_kind(cmd.kind)) {
      std::size_t run_end = index;
      while (run_end < request.commands.size() &&
             !request.commands[run_end].use_preplanned_trajectory &&
             is_cartesian_lookahead_kind(request.commands[run_end].kind)) {
        ++run_end;
      }

      std::string error_message;
      std::vector<MotionCommandSpec> run_commands(request.commands.begin() + static_cast<long>(index),
                                                  request.commands.begin() + static_cast<long>(run_end));
      if (!build_cartesian_run(*kinematics_,
                               request,
                               run_commands,
                               index,
                               current_joints,
                               plan.segments,
                               plan.notes,
                               error_message)) {
        const auto detail = error_message.empty() ? std::string("Cartesian run planning failed") : error_message;
        plan.error_message = format_motion_failure(classify_motion_failure_reason(detail), detail);
        return plan;
      }
      index = run_end;
      continue;
    }

    PlannedSegment segment;
    segment.kind = cmd.kind;
    segment.speed = cmd.speed > 0 ? cmd.speed : request.default_speed;
    segment.zone = cmd.zone;
    segment.trajectory_dt = request.trajectory_dt;
    segment.target_cartesian = cmd.target_cartesian;
    segment.aux_cartesian = cmd.aux_cartesian;
    segment.path_family = path_family(cmd.kind);

    if (cmd.use_preplanned_trajectory) {
      if (cmd.preplanned_trajectory.empty()) {
        plan.error_message = format_motion_failure("unreachable_pose", "preplanned trajectory is empty");
        return plan;
      }
      segment.joint_trajectory = cmd.preplanned_trajectory;
      segment.joint_velocity_trajectory = cmd.preplanned_velocity_trajectory;
      segment.joint_acceleration_trajectory = cmd.preplanned_acceleration_trajectory;
      segment.trajectory_dt = cmd.preplanned_dt > 1e-9 ? cmd.preplanned_dt : request.trajectory_dt;
      segment.trajectory_total_time =
          segment.joint_trajectory.size() > 1 ? (segment.joint_trajectory.size() - 1) * segment.trajectory_dt : 0.0;
      populate_joint_derivatives(segment);
      segment.target_joints = cmd.target_joints.empty() ? cmd.preplanned_trajectory.back() : cmd.target_joints;
      current_joints = segment.target_joints;
      plan.segments.push_back(std::move(segment));
      ++index;
      continue;
    }

    switch (cmd.kind) {
      case MotionKind::move_absj: {
        segment.target_joints = cmd.target_joints;
        if (request.soft_limit_enabled &&
            move_absj_violates_soft_limit(segment.target_joints, request.soft_limits)) {
          plan.error_message = format_motion_failure("soft_limit_violation", "MoveAbsJ target violates soft limit");
          return plan;
        }
        const auto trajectory = retimeJointPathWithUnifiedSpeed(
            {current_joints, segment.target_joints},
            request.trajectory_dt,
            segment.speed,
            RetimerSourceFamily::joint,
            request.speed_scale);
        if (trajectory.empty()) {
          const auto detail =
              trajectory.samples.error_message.empty() ? std::string("MoveAbsJ retiming failed")
                                                       : trajectory.samples.error_message;
          plan.error_message = format_motion_failure(classify_motion_failure_reason(detail), detail);
          return plan;
        }
        apply_unified_trajectory(segment, trajectory);
        append_retimer_note(plan.notes, plan.segments.size(), trajectory.metadata);
        break;
      }
      case MotionKind::move_j: {
        std::vector<std::vector<double>> candidate_solutions;
        const auto seeded_fast = kinematics_->inverseKinematicsSeededFast(cmd.target_cartesian, current_joints);
        if (!seeded_fast.empty()) {
          candidate_solutions.push_back(seeded_fast);
        }
        const auto multi_branch = kinematics_->inverseKinematicsMultiSolution(cmd.target_cartesian, current_joints);
        candidate_solutions.insert(candidate_solutions.end(), multi_branch.begin(), multi_branch.end());
        ::gazebo::xMate3Kinematics::CartesianIkOptions ik_options;
        ik_options.requested_conf = cmd.requested_conf;
        ik_options.strict_conf = request.strict_conf;
        ik_options.avoid_singularity = request.avoid_singularity;
        ik_options.soft_limit_enabled = request.soft_limit_enabled;
        ik_options.soft_limits = request.soft_limits;
        const auto selected =
            kinematics_->selectBestIkSolution(candidate_solutions, cmd.target_cartesian, current_joints, ik_options);
        if (!selected.success) {
          const auto detail = "MoveJ planning failed: " + selected.message;
          plan.error_message = format_motion_failure(classify_motion_failure_reason(selected.message), detail);
          return plan;
        }
        segment.target_joints = selected.joints;
        const auto trajectory = retimeJointPathWithUnifiedSpeed(
            {current_joints, segment.target_joints},
            request.trajectory_dt,
            segment.speed,
            RetimerSourceFamily::joint,
            request.speed_scale);
        if (trajectory.empty()) {
          const auto detail =
              trajectory.samples.error_message.empty() ? std::string("MoveJ retiming failed")
                                                       : trajectory.samples.error_message;
          plan.error_message = format_motion_failure(classify_motion_failure_reason(detail), detail);
          return plan;
        }
        apply_unified_trajectory(segment, trajectory);
        append_retimer_note(plan.notes, plan.segments.size(), trajectory.metadata);
        if (!selected.note.empty()) {
          plan.notes.push_back(selected.note + " branch=" + selected.branch_id);
        }
        break;
      }
      case MotionKind::move_sp: {
        const auto current_pose = kinematics_->forwardKinematicsRPY(current_joints);
        const auto cart_trajectory = TrajectoryPlanner::planSpiralMove(
            current_pose,
            cmd.target_cartesian,
            cmd.radius,
            cmd.radius_step,
            cmd.angle,
            cmd.direction,
            segment.speed,
            request.trajectory_dt);
        segment.trajectory_dt = cart_trajectory.sample_dt;
        segment.trajectory_total_time = cart_trajectory.total_time;
        std::vector<double> last_joints;
        std::string error_message;
        if (!build_joint_trajectory_from_cartesian(*kinematics_,
                                                   cart_trajectory.points,
                                                   current_joints,
                                                   cmd.requested_conf,
                                                   request.strict_conf,
                                                   request.avoid_singularity,
                                                   request.soft_limit_enabled,
                                                   request.soft_limits,
                                                   segment.joint_trajectory,
                                                   last_joints,
                                                   error_message)) {
          const auto detail = "MoveSP planning failed: " + error_message;
          plan.error_message = format_motion_failure(classify_motion_failure_reason(error_message), detail);
          return plan;
        }
        const auto retimed = retimeJointPathWithUnifiedSpeed(
            segment.joint_trajectory,
            request.trajectory_dt,
            segment.speed,
            RetimerSourceFamily::cartesian,
            request.speed_scale);
        if (retimed.empty()) {
          const auto detail =
              "MoveSP retiming failed: " +
              (retimed.samples.error_message.empty() ? std::string("unified retimer produced no samples")
                                                     : retimed.samples.error_message);
          plan.error_message = format_motion_failure(classify_motion_failure_reason(detail), detail);
          return plan;
        }
        apply_unified_trajectory(segment, retimed);
        append_retimer_note(plan.notes, plan.segments.size(), retimed.metadata);
        const auto retimed_cartesian_points =
            resampleCartesianPosePath(cart_trajectory.points, segment.joint_trajectory.size());
        if (!project_joint_derivatives_from_cartesian(*kinematics_,
                                                      retimed_cartesian_points,
                                                      segment.joint_trajectory,
                                                      segment.trajectory_dt,
                                                      segment.joint_velocity_trajectory,
                                                      segment.joint_acceleration_trajectory)) {
          segment.joint_velocity_trajectory = retimed.samples.velocities;
          segment.joint_acceleration_trajectory = retimed.samples.accelerations;
        }
        segment.target_joints = last_joints;
        segment.path_length_m = 0.0;
        break;
      }
      default:
        plan.error_message = format_motion_failure("unreachable_pose", "unsupported motion kind");
        return plan;
    }

    if (segment.joint_trajectory.empty()) {
      plan.error_message = format_motion_failure("unreachable_pose", "planning produced an empty trajectory");
      return plan;
    }
    if (segment.target_joints.empty()) {
      segment.target_joints = segment.joint_trajectory.back();
    }
    current_joints = segment.target_joints;
    plan.segments.push_back(std::move(segment));
    ++index;
  }

  apply_joint_zone_blending(plan.segments, plan.notes);
  for (auto &segment : plan.segments) {
    populate_joint_derivatives(segment);
  }
  return plan;
}

void MotionPlanner::resetDebugCounters() const {
  kinematics_->resetDebugCounters();
}

::gazebo::xMate3Kinematics::DebugCounters MotionPlanner::debugCounters() const {
  return kinematics_->debugCounters();
}

}  // namespace rokae_xmate3_ros2::runtime
