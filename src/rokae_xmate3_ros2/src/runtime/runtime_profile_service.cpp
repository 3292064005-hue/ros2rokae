#include "runtime/runtime_profile_service.hpp"

#include <algorithm>
#include <sstream>

namespace rokae_xmate3_ros2::runtime {
namespace {

bool has_flag(const std::vector<std::string> &flags, const std::string &needle) {
  return std::find(flags.begin(), flags.end(), needle) != flags.end();
}

RuntimeProfileDescriptor make_profile(std::string name,
                                      std::string owner_rule,
                                      std::string required_controller,
                                      std::string preferred_contract,
                                      std::string diagnostics_expectation,
                                      std::vector<std::string> motions,
                                      bool rt_capable,
                                      bool sim_approx,
                                      bool experimental) {
  RuntimeProfileDescriptor profile;
  profile.name = std::move(name);
  profile.owner_rule = std::move(owner_rule);
  profile.required_controller = std::move(required_controller);
  profile.preferred_contract = std::move(preferred_contract);
  profile.diagnostics_expectation = std::move(diagnostics_expectation);
  profile.allowed_motion_families = std::move(motions);
  profile.rt_capable = rt_capable;
  profile.sim_approx = sim_approx;
  profile.experimental = experimental;
  return profile;
}

}  // namespace

std::vector<RuntimeProfileDescriptor> buildRuntimeProfileCatalog(const std::string &backend_mode,
                                                                 const std::string &active_profile,
                                                                 const std::vector<std::string> &capability_flags) {
  std::vector<RuntimeProfileDescriptor> profiles;
  profiles.reserve(5);

  profiles.push_back(make_profile("nrt_queued",
                                  "runtime",
                                  "none",
                                  "queued_nrt",
                                  "planner+runtime diagnostics",
                                  {"MoveAbsJ", "MoveJ", "MoveL", "MoveC", "MoveCF", "MoveSP", "ReplayPath"},
                                  false,
                                  true,
                                  false));
  profiles.push_back(make_profile("rt_simulated",
                                  "runtime",
                                  "simulated_rt_bridge",
                                  "rt_simulated",
                                  "rt prearm + watchdog + loop stats",
                                  {"JointPosition", "CartesianPosition", "Torque"},
                                  true,
                                  true,
                                  true));
  profiles.push_back(make_profile("hybrid_bridge",
                                  "runtime",
                                  "trajectory+effort",
                                  "hybrid_bridge",
                                  "trajectory arbitration + runtime diagnostics",
                                  {"MoveAppend", "QueuedNrt", "RtFollow"},
                                  true,
                                  true,
                                  true));
  profiles.push_back(make_profile("effort_direct",
                                  "effort_owner",
                                  "effort_controller",
                                  "effort_direct",
                                  "effort ownership + watchdog",
                                  {"Torque", "Hold", "Retreat"},
                                  true,
                                  true,
                                  true));
  profiles.push_back(make_profile("jtc_profile",
                                  "trajectory_owner",
                                  "joint_trajectory_controller",
                                  "trajectory_jtc",
                                  "goal execution + queue diagnostics",
                                  {"QueuedNrt", "MoveAppend", "ReplayPath"},
                                  false,
                                  true,
                                  false));

  for (auto &profile : profiles) {
    profile.active = profile.name == active_profile;
    if (profile.name == "hybrid_bridge" && backend_mode == "hybrid") {
      profile.active = true;
    } else if (profile.name == "effort_direct" && backend_mode == "effort") {
      profile.active = true;
    } else if (profile.name == "jtc_profile" && backend_mode == "jtc") {
      profile.active = true;
    }

    if (profile.name == "rt_simulated" && !has_flag(capability_flags, "rt.experimental")) {
      profile.diagnostics_expectation += " (disabled)";
    }
    if (profile.name == "effort_direct" && !has_flag(capability_flags, "effort_owner")) {
      profile.diagnostics_expectation += " (limited)";
    }
    if (profile.name == "jtc_profile" && !has_flag(capability_flags, "trajectory_executor")) {
      profile.diagnostics_expectation += " (limited)";
    }
  }

  std::sort(profiles.begin(), profiles.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.active != rhs.active) {
      return lhs.active > rhs.active;
    }
    return lhs.name < rhs.name;
  });
  return profiles;
}

std::string summarizeRuntimeProfileCatalog(const std::vector<RuntimeProfileDescriptor> &profiles) {
  std::ostringstream stream;
  bool first = true;
  for (const auto &profile : profiles) {
    if (!first) {
      stream << "; ";
    }
    first = false;
    stream << profile.name << '=' << (profile.active ? "active" : "available");
    if (profile.experimental) {
      stream << ",experimental";
    }
    if (profile.rt_capable) {
      stream << ",rt";
    }
  }
  return stream.str();
}

}  // namespace rokae_xmate3_ros2::runtime
