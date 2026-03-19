#include "runtime/executor_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rokae_xmate3_ros2::runtime {
namespace {

constexpr double kTrackingPositionToleranceRad = 0.03;
constexpr double kTrackingVelocityToleranceRad = 0.40;
constexpr double kFinalPositionToleranceRad = 0.01;
constexpr double kFinalVelocityToleranceRad = 0.08;
constexpr double kSoftFinalPositionToleranceRad = 0.02;
constexpr double kSoftFinalVelocityToleranceRad = 0.18;
constexpr double kVelocityEstimateTimeConstant = 0.03;
constexpr double kVelocitySpikeClampRad = 6.0;
constexpr int kSettleCyclesRequired = 12;
constexpr int kSettleCyclesMax = 500;
constexpr int kSoftSettleWarmupCycles = 35;
constexpr int kStrongSettleStartCycles = 90;
constexpr double kHoldVelocityAssistGain = 2.2;
constexpr double kStrongHoldVelocityAssistGain = 3.8;
constexpr double kHoldVelocityAssistLimit = 0.18;
constexpr double kStrongHoldVelocityAssistLimit = 0.38;
constexpr std::array<double, 6> kActiveTrackKp = {260.0, 280.0, 240.0, 120.0, 90.0, 45.0};
constexpr std::array<double, 6> kActiveTrackKd = {28.0, 30.0, 24.0, 12.0, 8.0, 3.5};
constexpr std::array<double, 6> kActiveTrackKi = {10.0, 12.0, 10.0, 4.0, 2.0, 1.0};
constexpr std::array<double, 6> kHoldTrackKp = {380.0, 400.0, 350.0, 170.0, 125.0, 60.0};
constexpr std::array<double, 6> kHoldTrackKd = {38.0, 40.0, 32.0, 16.0, 11.0, 5.5};
constexpr std::array<double, 6> kHoldTrackKi = {24.0, 26.0, 22.0, 9.0, 4.5, 2.0};
constexpr std::array<double, 6> kStrongHoldTrackKp = {560.0, 580.0, 500.0, 235.0, 170.0, 82.0};
constexpr std::array<double, 6> kStrongHoldTrackKd = {54.0, 56.0, 45.0, 22.0, 15.0, 7.2};
constexpr std::array<double, 6> kStrongHoldTrackKi = {40.0, 42.0, 34.0, 14.0, 7.0, 3.0};
constexpr std::array<double, 6> kIntegralClamp = {0.70, 0.70, 0.65, 0.42, 0.32, 0.22};
constexpr std::array<double, 6> kVelocityFeedforward = {1.5, 1.7, 1.5, 0.8, 0.5, 0.2};
constexpr std::array<double, 6> kGravityCompensation = {0.0, 10.0, 7.0, 1.5, 0.8, 0.2};
constexpr std::array<double, 6> kStaticFrictionComp = {0.8, 1.0, 0.8, 0.3, 0.2, 0.1};
constexpr std::array<double, 6> kEffortLimit = {300.0, 300.0, 300.0, 300.0, 300.0, 300.0};

struct TrajectoryTrackingSample {
  std::vector<double> position;
  std::vector<double> velocity;
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
                                               double sample_dt) {
  TrajectoryTrackingSample sample;
  if (trajectory.empty()) {
    sample.finished = true;
    return sample;
  }

  sample.position = trajectory.back();
  sample.velocity.assign(sample.position.size(), 0.0);

  if (trajectory.size() == 1 || sample_dt <= 1e-9) {
    sample.finished = true;
    return sample;
  }

  const double max_time = static_cast<double>(trajectory.size() - 1) * sample_dt;
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
  for (size_t i = 0; i < trajectory[index].size(); ++i) {
    const double p0 = trajectory[index][i];
    const double p1 = trajectory[index + 1][i];
    sample.position[i] = p0 + alpha * (p1 - p0);
    sample.velocity[i] = (p1 - p0) / sample_dt;
  }
  sample.finished = clamped_time >= max_time;
  return sample;
}

}  // namespace

MotionExecutor::MotionExecutor() {
  hold_position_.resize(6, 0.0);
}

void MotionExecutor::resetControllerState() {
  effort_error_integral_.fill(0.0);
  effort_filtered_velocity_.fill(0.0);
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
    const std::array<double, 6> &kp,
    const std::array<double, 6> &kd,
    const std::array<double, 6> &ki,
    double position_tolerance,
    double velocity_tolerance) {
  TrackingStatus status;
  status.settled = true;
  const double safe_dt = std::clamp(dt, 1e-4, 0.05);
  const double velocity_alpha = std::clamp(safe_dt / (kVelocityEstimateTimeConstant + safe_dt), 0.05, 1.0);

  if (!effort_velocity_initialized_) {
    effort_last_position_ = snapshot.joint_position;
    effort_filtered_velocity_.fill(0.0);
    effort_velocity_initialized_ = true;
  }

  std::array<double, 6> effort{};
  for (int i = 0; i < 6; ++i) {
    const double current_pos = snapshot.joint_position[i];
    const double delta_pos = normalize_angle(current_pos - effort_last_position_[i]);
    const double estimated_velocity = std::clamp(delta_pos / safe_dt,
                                                 -kVelocitySpikeClampRad,
                                                 kVelocitySpikeClampRad);
    effort_filtered_velocity_[i] =
        velocity_alpha * estimated_velocity + (1.0 - velocity_alpha) * effort_filtered_velocity_[i];
    effort_last_position_[i] = current_pos;

    const double current_vel = effort_filtered_velocity_[i];
    const double desired_pos = i < static_cast<int>(target_position.size()) ? target_position[i] : current_pos;
    const double desired_vel = i < static_cast<int>(target_velocity.size()) ? target_velocity[i] : 0.0;
    const double pos_error = desired_pos - current_pos;
    const double vel_error = desired_vel - current_vel;

    effort_error_integral_[i] = std::clamp(
        effort_error_integral_[i] + pos_error * safe_dt,
        -kIntegralClamp[i],
        kIntegralClamp[i]);

    double static_friction = 0.0;
    if (std::fabs(desired_vel) > 1e-3) {
      static_friction = std::copysign(kStaticFrictionComp[i], desired_vel);
    } else if (std::fabs(pos_error) > 1e-3) {
      static_friction = std::copysign(kStaticFrictionComp[i] * 0.5, pos_error);
    }

    double axis_effort =
        kp[i] * pos_error +
        kd[i] * vel_error +
        ki[i] * effort_error_integral_[i] +
        kVelocityFeedforward[i] * desired_vel +
        kGravityCompensation[i] * std::sin(current_pos) +
        static_friction;
    axis_effort = std::clamp(axis_effort, -kEffortLimit[i], kEffortLimit[i]);
    effort[i] = axis_effort;

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
  const auto tracking = applyEffortTracking(snapshot, dt, hold_position_, zero_velocity,
                                            kHoldTrackKp, kHoldTrackKd, kHoldTrackKi,
                                            kFinalPositionToleranceRad, kFinalVelocityToleranceRad);
  ExecutionStep step;
  step.command = tracking.command;
  step.state = ExecutionState::idle;
  return step;
}

ExecutionStep MotionExecutor::tick(const RobotSnapshot &snapshot, double dt) {
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
    segment.trajectory_elapsed += std::clamp(dt, 1e-4, 0.05);
    const auto sample = sampleJointTrajectory(segment.segment.joint_trajectory,
                                              segment.trajectory_elapsed,
                                              segment.segment.trajectory_dt);
    const auto tracking = applyEffortTracking(snapshot, dt, sample.position, sample.velocity,
                                              kActiveTrackKp, kActiveTrackKd, kActiveTrackKi,
                                              kTrackingPositionToleranceRad,
                                              kTrackingVelocityToleranceRad);
    step.command = tracking.command;
    step.state = ExecutionState::executing;
    if (sample.finished) {
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

  const auto &hold_kp = use_strong_settle ? kStrongHoldTrackKp : kHoldTrackKp;
  const auto &hold_kd = use_strong_settle ? kStrongHoldTrackKd : kHoldTrackKd;
  const auto &hold_ki = use_strong_settle ? kStrongHoldTrackKi : kHoldTrackKi;
  const auto assisted_velocity = buildTerminalVelocityAssist(
      snapshot,
      segment.segment.target_joints,
      use_strong_settle ? kStrongHoldVelocityAssistGain : kHoldVelocityAssistGain,
      use_strong_settle ? kStrongHoldVelocityAssistLimit : kHoldVelocityAssistLimit);
  const auto tracking = applyEffortTracking(snapshot, dt, segment.segment.target_joints,
                                            assisted_velocity, hold_kp, hold_kd, hold_ki,
                                            kFinalPositionToleranceRad,
                                            kFinalVelocityToleranceRad);
  step.command = tracking.command;
  step.state = ExecutionState::settling;

  const bool soft_settled =
      segment.settle_attempts >= kSoftSettleWarmupCycles &&
      tracking.max_position_error <= kSoftFinalPositionToleranceRad &&
      tracking.max_velocity_error <= kSoftFinalVelocityToleranceRad;
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
      step.state = ExecutionState::completed;
      step.plan_completed = true;
      step.message = segment.settle_attempts >= kSettleCyclesMax
                         ? "plan completed after settle timeout fallback"
                         : "plan completed";
    } else {
      step.state = ExecutionState::queued;
    }
  }

  return step;
}

}  // namespace rokae_xmate3_ros2::runtime
