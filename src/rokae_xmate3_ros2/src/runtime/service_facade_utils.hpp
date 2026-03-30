#ifndef ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_FACADE_UTILS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_SERVICE_FACADE_UTILS_HPP

#include <array>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>

#include "runtime/service_facade.hpp"
#include "runtime/unified_retimer.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace rokae_xmate3_ros2::runtime::detail {

builtin_interfaces::msg::Time ToBuiltinTime(const rclcpp::Time &time);
std::string basename_without_extension(const std::string &path);
std::vector<double> snapshot_joints(const std::array<double, 6> &position);
std::vector<double> resolve_pose_for_coordinate(const std::vector<double> &flange_pose,
                                                const ToolsetSnapshot &toolset,
                                                int coordinate_type);
bool validate_coordinate_type(int coordinate_type, std::string &message);
void append_retimer_diagnostic(DataStoreState &data_store_state,
                               const rclcpp::Time &stamp,
                               const std::string &context,
                               const RetimerMetadata &metadata,
                               const std::string &detail = {},
                               RuntimeDiagnosticsState *diagnostics_state = nullptr);
rokae_xmate3_ros2::gazebo_model::LoadContext resolve_load_context(const ToolsetSnapshot &toolset);
rokae_xmate3_ros2::gazebo_model::ModelFacade make_runtime_model_facade(
    gazebo::xMate3Kinematics &kinematics,
    const ToolsetSnapshot &toolset);
std::vector<double> pose_from_array(const std::array<double, 6> &pose);
std::array<std::array<double, 2>, 6> soft_limits_from_request(const std::array<double, 12> &values);
bool is_finite_vector(const std::vector<double> &values);

}  // namespace rokae_xmate3_ros2::runtime::detail

#endif
