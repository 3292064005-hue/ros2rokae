#ifndef ROKAE_XMATE3_ROS2_RUNTIME_STATE_UTILS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_STATE_UTILS_HPP

#include <vector>

namespace rokae_xmate3_ros2::runtime::detail {

std::vector<double> normalize_runtime_pose(const std::vector<double> &pose);
double clamp_record_timestamp(double timestamp_sec, double fallback);

}  // namespace rokae_xmate3_ros2::runtime::detail

#endif
