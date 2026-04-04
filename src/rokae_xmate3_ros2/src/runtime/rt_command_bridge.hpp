#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RT_COMMAND_BRIDGE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RT_COMMAND_BRIDGE_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <system_error>

namespace rokae::ros2 {
class xMateRobot;
}

namespace rokae_xmate3_ros2::runtime::rt_command_bridge {

enum class CommandKind {
  JointPosition,
  CartesianPosition,
  Torque,
};

bool publishMetadata(rokae::ros2::xMateRobot &robot,
                     const std::string &dispatch_mode,
                     std::error_code &ec) noexcept;

bool publishCommand(rokae::ros2::xMateRobot &robot,
                    std::atomic<std::uint64_t> &sequence,
                    CommandKind kind,
                    const std::array<double, 6> &values,
                    bool finished,
                    std::error_code &ec,
                    const std::string &dispatch_mode = "independent_rt") noexcept;

bool publishStop(rokae::ros2::xMateRobot &robot, std::error_code &ec) noexcept;

}  // namespace rokae_xmate3_ros2::runtime::rt_command_bridge

#endif
