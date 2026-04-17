#ifndef ROKAE_XMATE3_ROS2_RUNTIME_MOTION_EXTENSION_CONTRACT_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_MOTION_EXTENSION_CONTRACT_HPP

#include <string>
#include <vector>

#include "runtime/runtime_types.hpp"

namespace rokae_xmate3_ros2::runtime {

struct MotionExtensionContract {
  MotionKind kind = MotionKind::none;
  std::string request_surface;
  std::string planner_output;
  std::string backend_capability;
  bool public_xmate6 = true;
  bool experimental = false;
};

[[nodiscard]] std::vector<MotionExtensionContract> buildMotionExtensionContracts();
[[nodiscard]] std::string summarizeMotionExtensionContracts(const std::vector<MotionExtensionContract> &contracts);
[[nodiscard]] bool validateMotionExtensionContracts(const std::vector<MotionExtensionContract> &contracts, std::string &error);

}  // namespace rokae_xmate3_ros2::runtime

#endif
