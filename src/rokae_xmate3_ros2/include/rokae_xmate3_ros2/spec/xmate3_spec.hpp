#ifndef ROKAE_XMATE3_ROS2_SPEC_XMATE3_SPEC_HPP
#define ROKAE_XMATE3_ROS2_SPEC_XMATE3_SPEC_HPP

#include <array>
#include <cstddef>

namespace rokae_xmate3_ros2::spec::xmate3 {

inline constexpr std::size_t kDoF = 6;
inline constexpr double kServoTickSec = 0.001;
inline constexpr int kRos2ControlNrtUpdateRateHz = 250;
inline constexpr int kRos2ControlRtUpdateRateHz = 1000;

inline constexpr std::array<const char *, kDoF> kJointNames = {
    "xmate_joint_1", "xmate_joint_2", "xmate_joint_3",
    "xmate_joint_4", "xmate_joint_5", "xmate_joint_6"};

inline constexpr std::array<double, kDoF> kJointLimitMin = {
    -3.0527, -2.0933, -2.0933, -3.0527, -2.0933, -6.1082};
inline constexpr std::array<double, kDoF> kJointLimitMax = {
     3.0527,  2.0933,  2.0933,  3.0527,  2.0933,  6.1082};
inline constexpr std::array<double, kDoF> kJointVelocityLimit = {
    2.175, 2.175, 2.175, 2.610, 2.610, 2.610};
inline constexpr std::array<double, kDoF> kJointAccelerationLimit = {
    15.0, 7.5, 10.0, 15.0, 15.0, 20.0};
inline constexpr std::array<double, kDoF> kJointJerkHint = {
    5000.0, 3500.0, 5000.0, 7500.0, 7500.0, 7500.0};
inline constexpr std::array<double, kDoF> kDirectTorqueLimit = {
    85.0, 85.0, 85.0, 36.0, 36.0, 36.0};
inline constexpr std::array<double, kDoF> kDirectTorqueRateLimit = {
    1500.0, 1500.0, 1500.0, 1000.0, 1000.0, 1000.0};

inline constexpr std::array<std::array<double, 2>, kDoF> kDefaultSoftLimits{{
    {{kJointLimitMin[0], kJointLimitMax[0]}},
    {{kJointLimitMin[1], kJointLimitMax[1]}},
    {{kJointLimitMin[2], kJointLimitMax[2]}},
    {{kJointLimitMin[3], kJointLimitMax[3]}},
    {{kJointLimitMin[4], kJointLimitMax[4]}},
    {{kJointLimitMin[5], kJointLimitMax[5]}}
}};

[[nodiscard]] constexpr const auto &jointNames() noexcept { return kJointNames; }
[[nodiscard]] constexpr const auto &jointLimitMin() noexcept { return kJointLimitMin; }
[[nodiscard]] constexpr const auto &jointLimitMax() noexcept { return kJointLimitMax; }
[[nodiscard]] constexpr const auto &jointVelocityLimit() noexcept { return kJointVelocityLimit; }
[[nodiscard]] constexpr const auto &jointAccelerationLimit() noexcept { return kJointAccelerationLimit; }
[[nodiscard]] constexpr const auto &jointJerkHint() noexcept { return kJointJerkHint; }
[[nodiscard]] constexpr const auto &directTorqueLimit() noexcept { return kDirectTorqueLimit; }
[[nodiscard]] constexpr const auto &directTorqueRateLimit() noexcept { return kDirectTorqueRateLimit; }
[[nodiscard]] constexpr const auto &defaultSoftLimits() noexcept { return kDefaultSoftLimits; }

namespace official_dh {
inline constexpr std::array<double, kDoF> kA = {0.0, 0.394, 0.0, 0.0, 0.0, 0.0};
inline constexpr std::array<double, kDoF> kAlpha = {
    -1.5707963267948966, 0.0, 1.5707963267948966,
    -1.5707963267948966, 1.5707963267948966, 0.0};
inline constexpr std::array<double, kDoF> kD = {0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};
inline constexpr std::array<double, kDoF> kThetaOffset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
[[nodiscard]] constexpr const auto &officialDhA() noexcept { return kA; }
[[nodiscard]] constexpr const auto &officialDhAlpha() noexcept { return kAlpha; }
[[nodiscard]] constexpr const auto &officialDhD() noexcept { return kD; }
[[nodiscard]] constexpr const auto &officialDhThetaOffset() noexcept { return kThetaOffset; }
}  // namespace official_dh

namespace improved_dh {
inline constexpr std::array<double, kDoF> kA = {0.0, 0.0, 0.394, 0.0, 0.0, 0.0};
inline constexpr std::array<double, kDoF> kAlpha = {
    0.0, -1.5707963267948966, 0.0,
    1.5707963267948966, -1.5707963267948966, 1.5707963267948966};
inline constexpr std::array<double, kDoF> kD = {0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};
inline constexpr std::array<double, kDoF> kJointOffset = {0.0, -1.5707963267948966, 1.5707963267948966, 0.0, 0.0, 0.0};
[[nodiscard]] constexpr const auto &improvedDhA() noexcept { return kA; }
[[nodiscard]] constexpr const auto &improvedDhAlpha() noexcept { return kAlpha; }
[[nodiscard]] constexpr const auto &improvedDhD() noexcept { return kD; }
[[nodiscard]] constexpr const auto &improvedDhJointOffset() noexcept { return kJointOffset; }
}  // namespace improved_dh

namespace legacy_dh = improved_dh;

}  // namespace rokae_xmate3_ros2::spec::xmate3

#endif
