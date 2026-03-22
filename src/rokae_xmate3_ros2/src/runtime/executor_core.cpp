#include "runtime/executor_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kVelocityEstimateTimeConstant = 0.03;
constexpr double kAccelerationEstimateTimeConstant = 0.05;
constexpr double kVelocitySpikeClampRad = 6.0;
constexpr double kAccelerationSpikeClampRad = 30.0;
constexpr int kSettleCyclesRequired = 12;
constexpr int kSettleCyclesMax = 500;
constexpr int kSoftSettleWarmupCycles = 35;
constexpr int kStrongSettleStartCycles = 90;
constexpr double kHoldVelocityAssistGain = 2.2;
constexpr double kStrongHoldVelocityAssistGain = 3.8;
constexpr double kHoldVelocityAssistLimit = 0.18;
constexpr double kStrongHoldVelocityAssistLimit = 0.38;
constexpr std::array<double, 6> kIntegralClamp = {0.70, 0.70, 0.65, 0.42, 0.32, 0.22};
constexpr std::array<double, 6> kVelocityFeedforward = {1.5, 1.7, 1.5, 0.8, 0.5, 0.2};
constexpr std::array<double, 6> kAccelerationFeedforward = {1.2, 1.4, 1.1, 0.5, 0.35, 0.15};

struct TrajectoryTrackingSample {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  bool finished = false;
};

double normalize_angle(double value) {
  constexpr double kPi = 3.14159265358979323846;
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

TrajectoryTrackingSample sampleJointTrajectory(const std::vector<std::vector<double>> &trajectory,
                                               double elapsed_time,
                                               double sample_dt,
                                               double total_time) {
  TrajectoryTrackingSample sample;
  if (trajectory.empty()) {
    sample.finished = true;
    return sample;
  }

  sample.position = trajectory.back();
  sample.velocity.assign(sample.position.size(), 0.0);
  sample.acceleration.assign(sample.position.size(), 0.0);

  if (trajectory.size() == 1 || sample_dt <= 1e-9) {
    sample.finished = true;
    return sample;
  }

  const double derived_total_time = static_cast<double>(trajectory.size() - 1) * sample_dt;
  const double max_time = total_time > 1e-9 ? total_time : derived_total_time;
  const double clamped_time = std::clamp(elapsed_time, 0.0, max_time);
  const double sample_index = clamped_time / sample_dt;
  const size_t index = static_cast<size_t>(std::floor(sample_index));
  if (index >= trajectory.size() - 1) {
    sample.finished = clamped_time >= max_time;
    return sample;
  }

  const double alpha = sample_index - static_cast<double>(index);
  sample.position.resize(trajectory[index].size(), 0.0);
  sample.velocity.resize(trajectory[index].size(), 0.0);
  sample.acceleration.resize(trajectory[index].size(), 0.0);

  auto estimate_tangent = [&](size_t point_index, size_t axis_index) {
    if (trajectory.size() == 2) {
      return (trajectory[1][axis_index] - trajectory[0][axis_index]) / sample_dt;
    }
    if (point_index == 0) {
      return (trajectory[1][axis_index] - trajectory[0][axis_index]) / sample_dt;
    }
    if (point_index + 1 >= trajectory.size()) {
      return (trajectory[point_index][axis_index] - trajectory[point_index - 1][axis_index]) / sample_dt;
    }
    return (trajectory[point_index + 1][axis_index] - trajectory[point_index - 1][axis_index]) / (2.0 * sample_dt);
  };

  const double u = alpha;
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
  const double h10 = u3 - 2.0 * u2 + u;
  const double h01 = -2.0 * u3 + 3.0 * u2;
  const double h11 = u3 - u2;
  const double dh00 = 6.0 * u2 - 6.0 * u;
  const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
  const double dh01 = -6.0 * u2 + 6.0 * u;
  const double dh11 = 3.0 * u2 - 2.0 * u;
  const double ddh00 = 12.0 * u - 6.0;
  const double ddh10 = 6.0 * u - 4.0;
  const double ddh01 = -12.0 * u + 6.0;
  const double ddh11 = 6.0 * u - 2.0;
  for (size_t i = 0; i < trajectory[index].size(); ++i) {
    const double p0 = trajectory[index][i];
    const double p1 = trajectory[index + 1][i];
    const double m0 = estimate_tangent(index, i);
    const double m1 = estimate_tangent(index + 1, i);
    sample.position[i] = h00 * p0 + h10 * sample_dt * m0 + h01 * p1 + h11 * sample_dt * m1;
    sample.velocity[i] = (dh00 / sample_dt) * p0 + dh10 * m0 + (dh01 / sample_dt) * p1 + dh11 * m1;
    sample.acceleration[i] =
        (ddh00 / (sample_dt * sample_dt)) * p0 + (ddh10 / sample_dt) * m0 +
        (ddh01 / (sample_dt * sample_dt)) * p1 + (ddh11 / sample_dt) * m1;
  }
  sample.finished = clamped_time >= max_time;
  return sample;
}

}  // namespace

MotionExecutor::MotionExecutor(MotionExecutorConfig config)
    : config_(std::move(config)) {
  hold_position_.resize(6, 0.0);
}

void MotionExecutor::setConfig(const MotionExecutorConfig &config) {
  config_ = config;
}

const MotionExecutorConfig &MotionExecutor::config() const noexcept {
  return config_;
}

void MotionExecutor::resetControllerState() {
  effort_error_integral_.fill(0.0);
  effort_last_velocity_.fill(0.0);
  effort_filtered_velocity_.fill(0.0);
  effort_filtered_acceleration_.fill(0.0);
  effort_last_command_.fill(0.0);
  effort_velocity_initialized_ = false;
}

void MotionExecutor::reset() {
  active_plan_.reset();
  active_segment_.reset();
  segment_index_ = 0;
  hold_position_initialized_ = false;
  resetControllerState();
}

void MotionExecutor::ensureHoldPosition(const RobotSnapshot &snapshot) {
  if (hold_position_initialized_) {
    return;
  }
  hold_position_.assign(snapshot.joint_position.begin(), snapshot.joint_position.end());
  hold_position_initialized_ = true;
}

void MotionExecutor::stop(const RobotSnapshot &snapshot) {
  ensureHoldPosition(snapshot);
  hold_position_.assign(snapshot.joint_position.begin(), snapshot.joint_position.end());
  active_plan_.reset();
  active_segment_.reset();
  segment_index_ = 0;
  resetControllerState();
}

void MotionExecutor::loadPlan(MotionPlan plan) {
  active_plan_ = std::move(plan);
  active_segment_.reset();
  segment_index_ = 0;
  resetControllerState();
}

bool MotionExecutor::hasActivePlan() const noexcept {
  return active_plan_.has_value();
}

std::size_t MotionExecutor::currentSegmentIndex() const noexcept {
  return segment_index_;
}

std::size_t MotionExecutor::totalSegments() const noexcept {
  return active_plan_ ? active_plan_->segments.size() : 0U;
}

void MotionExecutor::startCurrentSegment() {
  if (!active_plan_ || segment_index_ >= active_plan_->segments.size()) {
    active_segment_.reset();
    return;
  }
  ActiveSegment segment;
  segment.segment = active_plan_->segments[segment_index_];
  active_segment_ = std::move(segment);
  resetControllerState();
}

std::vector<double> MotionExecutor::buildTerminalVelocityAssist(const RobotSnapshot &snapshot,
                                                                const std::vector<double> &target_position,
                                                                double gain,
                                                                double velocity_limit) const {
  std::vector<double> assisted_velocity(target_position.size(), 0.0);
  for (int i = 0; i < 6 && i < static_cast<int>(target_position.size()); ++i) {
    const double pos_error = target_position[i] - snapshot.joint_position[i];
    if (std::fabs(pos_error) < 1e-4) {
      assisted_velocity[i] = 0.0;
      continue;
    }
    assisted_velocity[i] = std::clamp(pos_error * gain, -velocity_limit, velocity_limit);
  }
  return assisted_velocity;
}

ControlCommand MotionExecutor::makeControlCommand(const std::array<double, 6> &effort) const {
  ControlCommand command;
  command.effort = effort;
  command.has_effort = true;
  return command;
}

MotionExecutor::TrackingStatus MotionExecutor::applyEffortTracking(
    const RobotSnapshot &snapshot,
    double dt,
    const std::vector<double> &target_position,
    const std::vector<double> &target_velocity,
    const std::vector<double> &target_acceleration,
    const std::array<double, 6> &kp,
    const std::array<double, 6> &kd,
    const std::array<double, 6> &ki,
    double position_tolerance,
    double velocity_tolerance) {
  TrackingStatus status;
  status.settled = true;
  const double safe_dt = std::clamp(dt, 1e-4, 0.05);
  const double velocity_alpha = std::clamp(safe_dt / (kVelocityEstimateTimeConstant + safe_dt), 0.05, 1.0);
  const double acceleration_alpha =
      std::clamp(safe_dt / (kAccelerationEstimateTimeConstant + safe_dt), 0.05, 1.0);

  if (!effort_velocity_initialized_) {
    effort_last_position_ = snapshot.joint_position;
    effort_last_velocity_.fill(0.0);
    effort_filtered_velocity_.fill(0.0);
    effort_filtered_acceleration_.fill(0.0);
    effort_velocity_initialized_ = true;
  }

  std::array<double, 6> effort{};
  for (int i = 0; i < 6; ++i) {
    const double current_pos = snapshot.joint_position[i];
    const double delta_pos = normalize_angle(current_pos - effort_last_position_[i]);
    const double estimated_velocity = std::clamp(delta_pos / safe_dt,
                                                 -kVelocitySpikeClampRad,
                                                 kVelocitySpikeClampRad);
    const double measured_velocity = snapshot.joint_velocity[i];
    const bool measured_velocity_valid =
        std::isfinite(measured_velocity) && std::fabs(measured_velocity) <= kVelocitySpikeClampRad * 2.0;
    const double raw_velocity = measured_velocity_valid ? measured_velocity : estimated_velocity;
    effort_filtered_velocity_[i] =
        velocity_alpha * raw_velocity + (1.0 - velocity_alpha) * effort_filtered_velocity_[i];
    const double estimated_acceleration = std::clamp(
        (effort_filtered_velocity_[i] - effort_last_velocity_[i]) / safe_dt,
        -kAccelerationSpikeClampRad,
        kAccelerationSpikeClampRad);
    effort_filtered_acceleration_[i] =
        acceleration_alpha * estimated_acceleration +
        (1.0 - acceleration_alpha) * effort_filtered_acceleration_[i];
    effort_last_velocity_[i] = effort_filtered_velocity_[i];
    effort_last_position_[i] = current_pos;

    const double current_vel = effort_filtered_velocity_[i];
    const double current_acc = effort_filtered_acceleration_[i];
    const double desired_pos = i < static_cast<int>(target_position.size()) ? target_position[i] : current_pos;
    const double desired_vel = i < static_cast<int>(target_velocity.size()) ? target_velocity[i] : 0.0;
    const double desired_acc = i < static_cast<int>(target_acceleration.size()) ? target_acceleration[i] : 0.0;
    const double pos_error = desired_pos - current_pos;
    const double vel_error = desired_vel - current_vel;
    const double acc_error = desired_acc - current_acc;

    effort_error_integral_[i] = std::clamp(
        effort_error_integral_[i] + pos_error * safe_dt,
        -kIntegralClamp[i],
        kIntegralClamp[i]);

    double static_friction = 0.0;
    if (std::fabs(desired_vel) > 1e-3) {
      static_friction = std::copysign(config_.static_friction[i], desired_vel);
    } else if (std::fabs(pos_error) > 1e-3) {
      static_friction = std::copysign(config_.static_friction[i] * 0.5, pos_error);
    }

    double axis_effort =
        kp[i] * pos_error +
        kd[i] * vel_error +
        ki[i] * effort_error_integral_[i] +
        kVelocityFeedforward[i] * desired_vel +
        kAccelerationFeedforward[i] * acc_error +
        config_.gravity_compensation[i] * std::sin(current_pos) +
        static_friction;
    const double max_effort_delta = config_.torque_rate_limit[i] * safe_dt;
    axis_effort = std::clamp(axis_effort,
                             effort_last_command_[i] - max_effort_delta,
                             effort_last_command_[i] + max_effort_delta);
    axis_effort = std::clamp(axis_effort, -config_.effort_limit[i], config_.effort_limit[i]);
    effort[i] = axis_effort;
    effort_last_command_[i] = axis_effort;

    status.max_position_error = std::max(status.max_position_error, std::fabs(pos_error));
    status.max_velocity_error = std::max(status.max_velocity_error, std::fabs(vel_error));
    const bool pos_ok = std::fabs(pos_error) <= position_tolerance;
    const bool vel_ok = std::fabs(vel_error) <= velocity_tolerance;
    status.settled = status.settled && pos_ok && vel_ok;
  }

  status.command = makeControlCommand(effort);
  return status;
}

ExecutionStep MotionExecutor::holdPositionStep(const RobotSnapshot &snapshot, double dt) {
  ensureHoldPosition(snapshot);
  const std::vector<double> zero_velocity(hold_position_.size(), 0.0);
  const std::vector<double> zero_acceleration(hold_position_.size(), 0.0);
  const auto tracking = applyEffortTracking(snapshot, dt, hold_position_, zero_velocity,
                                            zero_acceleration,
                                            config_.hold_track_kp, config_.hold_track_kd, config_.hold_track_ki,
                                            config_.final_position_tolerance_rad,
                                            config_.final_velocity_tolerance_rad);
  ExecutionStep step;
  step.command = tracking.command;
  step.state = ExecutionState::idle;
  return step;
}

ExecutionStep MotionExecutor::tick(const RobotSnapshot &snapshot, double dt, double trajectory_time_scale) {
  ensureHoldPosition(snapshot);

  if (!active_plan_) {
    return holdPositionStep(snapshot, dt);
  }

  if (!active_segment_) {
    startCurrentSegment();
    if (!active_segment_) {
      hold_position_.assign(snapshot.joint_position.begin(), snapshot.joint_position.end());
      active_plan_.reset();
      ExecutionStep step = holdPositionStep(snapshot, dt);
      step.state = ExecutionState::completed;
      step.plan_completed = true;
      step.terminal_success = true;
      return step;
    }
  }

  ExecutionStep step;
  step.current_segment_index = segment_index_;
  step.completed_segments = segment_index_;

  auto &segment = *active_segment_;
  if (!segment.segment.target_joints.empty()) {
    hold_position_ = segment.segment.target_joints;
  }

  if (!segment.in_fine_tuning && !segment.segment.joint_trajectory.empty()) {
    const double clamped_scale = std::clamp(trajectory_time_scale, 0.05, 2.0);
    const double trajectory_dt = std::clamp(dt * clamped_scale, 1e-4, 0.05);
    segment.trajectory_elapsed += trajectory_dt;
    const auto sample = sampleJointTrajectory(segment.segment.joint_trajectory,
                                              segment.trajectory_elapsed,
                                              segment.segment.trajectory_dt,
                                              segment.segment.trajectory_total_time);
    std::vector<double> scaled_velocity = sample.velocity;
    std::vector<double> scaled_acceleration = sample.acceleration;
    for (double &value : scaled_velocity) {
      value *= clamped_scale;
    }
    for (double &value : scaled_acceleration) {
      value *= clamped_scale * clamped_scale;
    }
    const auto tracking = applyEffortTracking(snapshot, dt, sample.position, scaled_velocity,
                                              scaled_acceleration,
                                              config_.active_track_kp,
                                              config_.active_track_kd,
                                              config_.active_track_ki,
                                              config_.tracking_position_tolerance_rad,
                                              config_.tracking_velocity_tolerance_rad);
    step.command = tracking.command;
    step.state = ExecutionState::executing;
    if (sample.finished) {
      if (segment.segment.blend_to_next && active_plan_ && segment_index_ + 1 < active_plan_->segments.size()) {
        ++segment_index_;
        step.completed_segments = segment_index_;
        step.state = ExecutionState::queued;
        step.message = "blending to next segment";
        active_segment_.reset();
        resetControllerState();
        return step;
      }
      segment.in_fine_tuning = true;
      segment.fine_tuning_steps = 0;
      segment.settle_attempts = 0;
      resetControllerState();
    }
    return step;
  }

  segment.in_fine_tuning = true;
  ++segment.settle_attempts;
  const bool use_strong_settle = segment.settle_attempts >= kStrongSettleStartCycles;
  if (segment.settle_attempts == kStrongSettleStartCycles) {
    segment.fine_tuning_steps = 0;
    resetControllerState();
  }

  const auto &hold_kp = use_strong_settle ? config_.strong_hold_track_kp : config_.hold_track_kp;
  const auto &hold_kd = use_strong_settle ? config_.strong_hold_track_kd : config_.hold_track_kd;
  const auto &hold_ki = use_strong_settle ? config_.strong_hold_track_ki : config_.hold_track_ki;
  const auto assisted_velocity = buildTerminalVelocityAssist(
      snapshot,
      segment.segment.target_joints,
      use_strong_settle ? kStrongHoldVelocityAssistGain : kHoldVelocityAssistGain,
      use_strong_settle ? kStrongHoldVelocityAssistLimit : kHoldVelocityAssistLimit);
  const std::vector<double> zero_acceleration(segment.segment.target_joints.size(), 0.0);
  const auto tracking = applyEffortTracking(snapshot, dt, segment.segment.target_joints,
                                            assisted_velocity, zero_acceleration,
                                            hold_kp, hold_kd, hold_ki,
                                            config_.final_position_tolerance_rad,
                                            config_.final_velocity_tolerance_rad);
  step.command = tracking.command;
  step.state = ExecutionState::settling;

  const bool soft_settled =
      segment.settle_attempts >= kSoftSettleWarmupCycles &&
      tracking.max_position_error <= config_.soft_final_position_tolerance_rad &&
      tracking.max_velocity_error <= config_.soft_final_velocity_tolerance_rad;
  if (tracking.settled || soft_settled) {
    ++segment.fine_tuning_steps;
  } else {
    segment.fine_tuning_steps = 0;
  }

  if (segment.fine_tuning_steps >= kSettleCyclesRequired ||
      segment.settle_attempts >= kSettleCyclesMax) {
    ++segment_index_;
    step.completed_segments = segment_index_;
    active_segment_.reset();
    resetControllerState();

    if (segment_index_ >= active_plan_->segments.size()) {
      active_plan_.reset();
      const bool settled_without_fallback = segment.settle_attempts < kSettleCyclesMax;
      step.state = settled_without_fallback ? ExecutionState::completed : ExecutionState::completed_relaxed;
      step.plan_completed = true;
      step.terminal_success = settled_without_fallback;
      step.message = settled_without_fallback
                         ? "plan completed"
                         : "completed_with_relaxed_settle";
    } else {
      step.state = ExecutionState::queued;
    }
  }

  return step;
}

}  // namespace rokae_xmate3_ros2::runtime
