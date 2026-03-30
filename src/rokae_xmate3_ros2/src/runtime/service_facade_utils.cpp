#include "runtime/service_facade_utils.hpp"

#include <algorithm>
#include <cmath>

#include "runtime/pose_utils.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"

namespace rokae_xmate3_ros2::runtime::detail {

builtin_interfaces::msg::Time ToBuiltinTime(const rclcpp::Time &time) {
  builtin_interfaces::msg::Time stamp;
  const auto ns = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

std::string basename_without_extension(const std::string &path) {
  std::string name = path;
  const auto slash = name.find_last_of("/\\");
  if (slash != std::string::npos) {
    name = name.substr(slash + 1);
  }
  const auto dot = name.find_last_of('.');
  if (dot != std::string::npos) {
    name = name.substr(0, dot);
  }
  return name.empty() ? std::string("default_rl_project") : name;
}

std::vector<double> snapshot_joints(const std::array<double, 6> &position) {
  return std::vector<double>(position.begin(), position.end());
}

std::vector<double> resolve_pose_for_coordinate(const std::vector<double> &flange_pose,
                                                const ToolsetSnapshot &toolset,
                                                int coordinate_type) {
  if (coordinate_type == 1) {
    return pose_utils::convertFlangeInBaseToEndInRef(flange_pose, toolset.tool_pose, toolset.wobj_pose);
  }
  return flange_pose;
}

bool validate_coordinate_type(int coordinate_type, std::string &message) {
  if (coordinate_type == 0 || coordinate_type == 1) {
    return true;
  }
  message = "unsupported coordinate_type";
  return false;
}

void append_retimer_diagnostic(DataStoreState &data_store_state,
                               const rclcpp::Time &stamp,
                               const std::string &context,
                               const RetimerMetadata &metadata,
                               const std::string &detail,
                               RuntimeDiagnosticsState *diagnostics_state) {
  rokae_xmate3_ros2::msg::LogInfo log;
  log.timestamp = std::to_string(stamp.nanoseconds());
  log.content = describeRetimerMetadata(metadata, context, detail);
  log.repair.clear();
  log.level = 1;
  data_store_state.appendLog(log);
  if (diagnostics_state != nullptr) {
    diagnostics_state->noteRetimerNote(log.content);
  }
}

rokae_xmate3_ros2::gazebo_model::LoadContext resolve_load_context(const ToolsetSnapshot &toolset) {
  return rokae_xmate3_ros2::gazebo_model::LoadContext{toolset.tool_mass, toolset.tool_com};
}

rokae_xmate3_ros2::gazebo_model::ModelFacade make_runtime_model_facade(
    gazebo::xMate3Kinematics &kinematics,
    const ToolsetSnapshot &toolset) {
  return rokae_xmate3_ros2::gazebo_model::makeModelFacade(
      kinematics,
      {toolset.tool_pose[0], toolset.tool_pose[1], toolset.tool_pose[2],
       toolset.tool_pose[3], toolset.tool_pose[4], toolset.tool_pose[5]},
      resolve_load_context(toolset));
}

std::vector<double> pose_from_array(const std::array<double, 6> &pose) {
  return std::vector<double>(pose.begin(), pose.end());
}

std::array<std::array<double, 2>, 6> soft_limits_from_request(const std::array<double, 12> &values) {
  auto soft_limits = rokae_xmate3_ros2::spec::xmate3::kDefaultSoftLimits;
  for (std::size_t axis = 0; axis < 6; ++axis) {
    soft_limits[axis][0] = values[axis * 2];
    soft_limits[axis][1] = values[axis * 2 + 1];
  }
  return soft_limits;
}

bool is_finite_vector(const std::vector<double> &values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

}  // namespace rokae_xmate3_ros2::runtime::detail
