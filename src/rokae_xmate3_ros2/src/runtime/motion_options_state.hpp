#ifndef ROKAE_XMATE3_ROS2_RUNTIME_MOTION_OPTIONS_STATE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_MOTION_OPTIONS_STATE_HPP

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include "runtime/runtime_snapshots.hpp"
#include "runtime/request_adapter.hpp"
#include "rokae_xmate3_ros2/spec/xmate_er3_truth.hpp"

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

  /**
   * @brief Configure whether experimental motion command families may enter MoveAppend/replay.
   * @param enabled True only for experimental/internal exposure profiles.
   * @param service_exposure_profile Stable profile token recorded in request diagnostics.
   * @throws None. Empty profile names are normalized to public_xmate_er3_only.
   * @details Boundary behavior: default public xMateER3 requests reject experimental families such as
   * MoveSP and recorded-path replay before planning. The ROS action/service types remain source-compatible,
   * but their experimental payload lanes are not executable unless this policy is explicitly enabled.
   */
  void setExperimentalMotionExtensionsEnabled(bool enabled, std::string service_exposure_profile);
  [[nodiscard]] bool experimentalMotionExtensionsEnabled() const;
  [[nodiscard]] std::string serviceExposureProfile() const;

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
  bool experimental_motion_extensions_enabled_ = false;
  std::string service_exposure_profile_ = "public_xmate_er3_only";
  std::array<std::array<double, 2>, 6> soft_limits_ = rokae_xmate3_ros2::spec::xmate_er3_truth::kDefaultSoftLimits;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
