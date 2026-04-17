#include "runtime/rt_prearm_checks.hpp"
#include "runtime/session_state.hpp"

#include <algorithm>
#include <sstream>

#include "rokae_xmate3_ros2/types.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

bool hasFlag(const std::vector<std::string> &flags, const std::string &needle) {
  return std::find(flags.begin(), flags.end(), needle) != flags.end();
}

bool hasField(const RtSubscriptionPlan &plan, const std::string &field) {
  return std::find(plan.accepted_fields.begin(), plan.accepted_fields.end(), field) != plan.accepted_fields.end();
}

std::vector<std::string> requiredFieldsForMode(int rt_mode) {
  using rokae::RtControllerMode;
  using namespace rokae::RtSupportedFields;
  switch (static_cast<RtControllerMode>(rt_mode)) {
    case RtControllerMode::jointPosition:
    case RtControllerMode::jointImpedance:
      return {jointPos_m, jointVel_m};
    case RtControllerMode::cartesianPosition:
    case RtControllerMode::cartesianImpedance:
      return {jointPos_m, jointVel_m, tcpPose_m};
    case RtControllerMode::torque:
      return {jointPos_m, jointVel_m, tau_m};
    default:
      return {};
  }
}

bool profileSupportsRtMode(const RtPrearmCheckInput &input) {
  using rokae::RtControllerMode;
  if (input.active_profile.find("rt") == std::string::npos &&
      input.active_profile.find("hybrid") == std::string::npos &&
      input.active_profile.find("effort") == std::string::npos) {
    return false;
  }
  switch (static_cast<RtControllerMode>(input.rt_mode)) {
    case RtControllerMode::jointPosition:
    case RtControllerMode::cartesianPosition:
      return hasFlag(input.capability_flags, "rt.experimental") || hasFlag(input.capability_flags, "trajectory_executor");
    case RtControllerMode::jointImpedance:
    case RtControllerMode::cartesianImpedance:
    case RtControllerMode::torque:
      return hasFlag(input.capability_flags, "rt.experimental") || hasFlag(input.capability_flags, "effort_owner");
    default:
      return false;
  }
}

std::string modeName(int rt_mode) {
  using rokae::RtControllerMode;
  switch (static_cast<RtControllerMode>(rt_mode)) {
    case RtControllerMode::jointPosition:
      return "joint_position";
    case RtControllerMode::cartesianPosition:
      return "cartesian_position";
    case RtControllerMode::jointImpedance:
      return "joint_impedance";
    case RtControllerMode::cartesianImpedance:
      return "cartesian_impedance";
    case RtControllerMode::torque:
      return "torque";
    default:
      return "unknown";
  }
}

}  // namespace

std::string RtPrearmCheckReport::summary() const {
  std::ostringstream stream;
  stream << status;
  if (!notes.empty()) {
    stream << " notes=";
    for (std::size_t i = 0; i < notes.size(); ++i) {
      if (i != 0) {
        stream << '|';
      }
      stream << notes[i];
    }
  }
  return stream.str();
}

RtPrearmCheckReport evaluateRtPrearm(const RtPrearmCheckInput &input) {
  RtPrearmCheckReport report;
  if (input.motion_mode != kSessionMotionModeRt) {
    report.status = "motion_mode_not_rt";
    report.notes.push_back("rt.prearm.motion_mode!=RtCommand");
    return report;
  }
  if (!input.power_on) {
    report.status = "power_off";
    report.notes.push_back("rt.prearm.power_off");
    return report;
  }
  if (input.rt_mode < 0 || input.rt_mode > static_cast<int>(rokae::RtControllerMode::torque)) {
    report.status = "rt_mode_unset";
    report.notes.push_back("rt.prearm.invalid_rt_mode");
    return report;
  }
  if (!profileSupportsRtMode(input)) {
    report.status = "profile_not_rt";
    report.notes.push_back("rt.prearm.profile_not_rt_capable");
    report.notes.push_back("rt.prearm.mode=" + modeName(input.rt_mode));
    return report;
  }
  if (!input.network_tolerance_configured) {
    report.status = "network_tolerance_missing";
    report.notes.push_back("rt.prearm.network_tolerance_missing");
    return report;
  }
  if (!input.subscription_plan.ok) {
    report.status = "subscription_plan_invalid";
    report.notes.push_back("rt.prearm.subscription_plan_invalid");
    report.notes.push_back(input.subscription_plan.summary());
    return report;
  }

  const auto required_fields = requiredFieldsForMode(input.rt_mode);
  for (const auto &field : required_fields) {
    if (!hasField(input.subscription_plan, field)) {
      report.status = "subscription_plan_incomplete";
      report.notes.push_back("rt.prearm.missing_field=" + field);
      report.notes.push_back("rt.prearm.mode=" + modeName(input.rt_mode));
      report.notes.push_back(input.subscription_plan.summary());
      return report;
    }
  }

  report.ok = true;
  report.status = "ready";
  report.notes.push_back("rt.prearm.ready");
  report.notes.push_back("rt.prearm.mode=" + modeName(input.rt_mode));
  report.notes.push_back(input.subscription_plan.summary());
  return report;
}

}  // namespace rokae_xmate3_ros2::runtime
