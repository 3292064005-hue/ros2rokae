#ifndef ROKAE_XMATE3_ROS2_SPEC_XMATE3_SPEC_HPP
#define ROKAE_XMATE3_ROS2_SPEC_XMATE3_SPEC_HPP

#include "rokae_xmate3_ros2/spec/xmate_six_axis_common.hpp"

namespace rokae_xmate3_ros2::spec::xmate3 {

using namespace rokae_xmate3_ros2::spec::xmate_six_axis_common;

inline constexpr const char *kRobotModelName = "xMate3";
inline constexpr const char *kSourceXacroEntry = "urdf/xMate3.xacro";
inline constexpr const char *kBuildGeneratedUrdf = "<build>/generated/urdf/xMate3.urdf";
inline constexpr const char *kBuildGeneratedUrdfMetadata = "<build>/generated/urdf/xMate3.description.json";

[[nodiscard]] constexpr const char *robotModelName() noexcept { return kRobotModelName; }
[[nodiscard]] constexpr const char *sourceXacroEntry() noexcept { return kSourceXacroEntry; }
[[nodiscard]] constexpr const char *buildGeneratedUrdf() noexcept { return kBuildGeneratedUrdf; }
[[nodiscard]] constexpr const char *buildGeneratedUrdfMetadata() noexcept { return kBuildGeneratedUrdfMetadata; }

namespace official_dh {
using namespace rokae_xmate3_ros2::spec::xmate_six_axis_common::official_dh;
}
namespace improved_dh {
using namespace rokae_xmate3_ros2::spec::xmate_six_axis_common::improved_dh;
}
namespace legacy_dh = improved_dh;

}  // namespace rokae_xmate3_ros2::spec::xmate3

#endif
