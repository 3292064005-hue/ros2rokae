#ifndef ROKAE_XMATE3_ROS2_SPEC_XMATE_ER3_TRUTH_HPP
#define ROKAE_XMATE3_ROS2_SPEC_XMATE_ER3_TRUTH_HPP

#include "rokae_xmate3_ros2/spec/xmate_six_axis_common.hpp"

namespace rokae_xmate3_ros2::spec::xmate_er3_truth {

using namespace rokae_xmate3_ros2::spec::xmate_six_axis_common;

inline constexpr const char *kRobotFamilyName = "xMateER3";
inline constexpr const char *kRobotModelName = "xMateER3";
inline constexpr const char *kCanonicalIdentity = "xCoreSDK:xmate_er3";
inline constexpr const char *kModelRevision = "xmate_er3_public_v2026_04";
inline constexpr const char *kCanonicalSourceXacroEntry = "urdf/xMateER3.xacro";
inline constexpr const char *kCanonicalBuildGeneratedUrdf = "<build>/generated/urdf/xMateER3.urdf";
inline constexpr const char *kCanonicalBuildGeneratedUrdfMetadata = "<build>/generated/urdf/xMateER3.description.json";
inline constexpr const char *kLegacySourceXacroEntry = "urdf/xMate3.xacro";
inline constexpr const char *kLegacyBuildGeneratedUrdf = "<build>/generated/urdf/xMate3.urdf";
inline constexpr const char *kLegacyBuildGeneratedUrdfMetadata = "<build>/generated/urdf/xMate3.description.json";

[[nodiscard]] constexpr const char *robotFamilyName() noexcept { return kRobotFamilyName; }
[[nodiscard]] constexpr const char *robotModelName() noexcept { return kRobotModelName; }
[[nodiscard]] constexpr const char *canonicalIdentity() noexcept { return kCanonicalIdentity; }
[[nodiscard]] constexpr const char *modelRevision() noexcept { return kModelRevision; }
[[nodiscard]] constexpr const char *canonicalSourceXacroEntry() noexcept { return kCanonicalSourceXacroEntry; }
[[nodiscard]] constexpr const char *canonicalBuildGeneratedUrdf() noexcept { return kCanonicalBuildGeneratedUrdf; }
[[nodiscard]] constexpr const char *canonicalBuildGeneratedUrdfMetadata() noexcept { return kCanonicalBuildGeneratedUrdfMetadata; }
[[nodiscard]] constexpr const char *legacySourceXacroEntry() noexcept { return kLegacySourceXacroEntry; }
[[nodiscard]] constexpr const char *legacyBuildGeneratedUrdf() noexcept { return kLegacyBuildGeneratedUrdf; }
[[nodiscard]] constexpr const char *legacyBuildGeneratedUrdfMetadata() noexcept { return kLegacyBuildGeneratedUrdfMetadata; }

}  // namespace rokae_xmate3_ros2::spec::xmate_er3_truth

#endif
