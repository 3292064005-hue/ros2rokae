#include "runtime/runtime_state_utils.hpp"

#include <cmath>

#include "runtime/pose_utils.hpp"

namespace rokae_xmate3_ros2::runtime::detail {

std::vector<double> normalize_runtime_pose(const std::vector<double> &pose) {
  return pose_utils::sanitizePose(pose);
}

double clamp_record_timestamp(double timestamp_sec, double fallback) {
  if (!std::isfinite(timestamp_sec)) {
    return fallback;
  }
  return timestamp_sec;
}

}  // namespace rokae_xmate3_ros2::runtime::detail
