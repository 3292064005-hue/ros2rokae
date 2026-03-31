#include "runtime/rt_prearm_checks.hpp"

#include <sstream>

#include "rokae_xmate3_ros2/types.hpp"

namespace rokae_xmate3_ros2::runtime {

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
  if (input.motion_mode != static_cast<int>(rokae::MotionControlMode::RtCommand)) {
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
  if (input.active_profile.find("rt") == std::string::npos &&
      input.active_profile.find("hybrid") == std::string::npos) {
    report.status = "profile_not_rt";
    report.notes.push_back("rt.prearm.profile_not_rt_capable");
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

  report.ok = true;
  report.status = "ready";
  report.notes.push_back("rt.prearm.ready");
  report.notes.push_back(input.subscription_plan.summary());
  return report;
}

}  // namespace rokae_xmate3_ros2::runtime
