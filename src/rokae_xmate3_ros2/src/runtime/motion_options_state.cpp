#include "runtime/runtime_state.hpp"

#include <algorithm>

namespace rokae_xmate3_ros2::runtime {

void MotionOptionsState::setDefaultSpeed(double speed) {
  std::lock_guard<std::mutex> lock(mutex_);
  default_speed_ = speed;
}

double MotionOptionsState::defaultSpeed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return default_speed_;
}

void MotionOptionsState::setDefaultZone(int zone) {
  std::lock_guard<std::mutex> lock(mutex_);
  default_zone_ = zone;
}

int MotionOptionsState::defaultZone() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return default_zone_;
}

void MotionOptionsState::setZoneValidRange(int min_zone, int max_zone) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (max_zone < min_zone) {
    std::swap(min_zone, max_zone);
  }
  zone_valid_min_ = min_zone;
  zone_valid_max_ = max_zone;
  default_zone_ = std::clamp(default_zone_, zone_valid_min_, zone_valid_max_);
}

std::array<int, 2> MotionOptionsState::zoneValidRange() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {zone_valid_min_, zone_valid_max_};
}

void MotionOptionsState::setSpeedScale(double scale) {
  std::lock_guard<std::mutex> lock(mutex_);
  speed_scale_ = scale;
}

double MotionOptionsState::speedScale() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return speed_scale_;
}

void MotionOptionsState::setDefaultConfOpt(bool forced) {
  std::lock_guard<std::mutex> lock(mutex_);
  default_conf_opt_forced_ = forced;
}

bool MotionOptionsState::defaultConfOptForced() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return default_conf_opt_forced_;
}

void MotionOptionsState::setAvoidSingularity(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  avoid_singularity_enabled_ = enabled;
}

bool MotionOptionsState::avoidSingularityEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return avoid_singularity_enabled_;
}

void MotionOptionsState::setSoftLimit(bool enabled,
                                      const std::array<std::array<double, 2>, 6> &limits) {
  std::lock_guard<std::mutex> lock(mutex_);
  soft_limit_enabled_ = enabled;
  soft_limits_ = limits;
}

SoftLimitSnapshot MotionOptionsState::softLimit() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return SoftLimitSnapshot{soft_limit_enabled_, soft_limits_};
}

MotionRequestContext MotionOptionsState::makeMotionRequestContext(const std::string &request_id,
                                                                  const std::vector<double> &start_joints,
                                                                  double trajectory_dt) const {
  std::lock_guard<std::mutex> lock(mutex_);
  MotionRequestContext context;
  context.request_id = request_id;
  context.start_joints = start_joints;
  context.default_speed = default_speed_;
  context.default_zone = default_zone_;
  context.strict_conf = default_conf_opt_forced_;
  context.avoid_singularity = avoid_singularity_enabled_;
  context.soft_limit_enabled = soft_limit_enabled_;
  context.soft_limits = soft_limits_;
  context.speed_scale = speed_scale_;
  context.trajectory_dt = trajectory_dt;
  return context;
}


}  // namespace rokae_xmate3_ros2::runtime
