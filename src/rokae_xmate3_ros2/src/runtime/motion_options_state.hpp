#ifndef ROKAE_XMATE3_ROS2_RUNTIME_MOTION_OPTIONS_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_MOTION_OPTIONS_STATE_HPP

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "runtime/runtime_snapshots.hpp"
#include "runtime/request_adapter.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime {

class MotionOptionsState {
 public:
  void setDefaultSpeed(double speed);
  [[nodiscard]] double defaultSpeed() const;

  void setDefaultZone(int zone);
  [[nodiscard]] int defaultZone() const;
  void setZoneValidRange(int min_zone, int max_zone);
  [[nodiscard]] std::array<int, 2> zoneValidRange() const;

  void setSpeedScale(double scale);
  [[nodiscard]] double speedScale() const;

  void setDefaultConfOpt(bool forced);
  [[nodiscard]] bool defaultConfOptForced() const;

  void setAvoidSingularity(bool enabled);
  [[nodiscard]] bool avoidSingularityEnabled() const;

  void setSoftLimit(bool enabled, const std::array<std::array<double, 2>, 6> &limits);
  [[nodiscard]] SoftLimitSnapshot softLimit() const;

  [[nodiscard]] MotionRequestContext makeMotionRequestContext(const std::string &request_id,
                                                              const std::vector<double> &start_joints,
                                                              double trajectory_dt) const;

 private:
  mutable std::mutex mutex_;
  double default_speed_ = 50.0;
  int default_zone_ = 0;
  int zone_valid_min_ = 0;
  int zone_valid_max_ = 200;
  double speed_scale_ = 1.0;
  bool default_conf_opt_forced_ = false;
  bool avoid_singularity_enabled_ = false;
  bool soft_limit_enabled_ = false;
  std::array<std::array<double, 2>, 6> soft_limits_ = rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
