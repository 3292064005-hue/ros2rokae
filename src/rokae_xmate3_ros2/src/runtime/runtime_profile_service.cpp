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
                                      std::string authority_scope,
                                      std::string fidelity_class,
                                      std::string model_revision,
                                      std::string provider_class,
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
  profile.authority_scope = std::move(authority_scope);
  profile.fidelity_class = std::move(fidelity_class);
  profile.model_revision = std::move(model_revision);
  profile.provider_class = std::move(provider_class);
  profile.rt_capable = rt_capable;
  profile.sim_approx = sim_approx;
  profile.experimental = experimental;
  return profile;
}

bool should_activate(const BackendContractDescriptor &backend_contract,
                     const RuntimeProfileDescriptor &profile,
                     const std::string &active_profile) {
  if (profile.name == active_profile) {
    return true;
  }
  if (profile.name == "nrt_strict_parity") {
    return backend_contract.supports_trajectory_execution;
  }
  if (profile.name == "jtc_profile") {
    return backend_contract.backend_mode == "jtc";
  }
  if (profile.name == "hybrid_bridge") {
    return backend_contract.backend_mode == "hybrid";
  }
  if (profile.name == "effort_direct") {
    return backend_contract.backend_mode == "effort";
  }
  if ((profile.name == "rt_sim_experimental_best_effort" || profile.name == "rt_hardened" || profile.name == "hard_1khz") &&
      backend_contract.supports_effort_owner) {
    return profile.name == active_profile;
  }
  return false;
}

}  // namespace

std::vector<RuntimeProfileDescriptor> buildRuntimeProfileCatalog(const BackendContractDescriptor &backend_contract,
                                                                 const std::string &active_profile,
                                                                 const std::vector<std::string> &capability_flags) {
  std::vector<RuntimeProfileDescriptor> profiles;
  profiles.reserve(7);

  profiles.push_back(make_profile("nrt_strict_parity",
                                  "runtime",
                                  "none",
                                  "queued_nrt",
                                  "planner+runtime diagnostics; strict queued non-realtime contract",
                                  {"MoveAbsJ", "MoveJ", "MoveL", "MoveC", "MoveCF", "MoveSP", "ReplayPath"},
                                  "runtime_request_coordinator",
                                  "controller_semantic_parity",
                                  "xmate6_public_v2026_04",
                                  "trajectory_executor",
                                  false,
                                  false,
                                  false));
  profiles.push_back(make_profile("rt_sim_experimental_best_effort",
                                  "runtime",
                                  "simulated_rt_bridge",
                                  "rt_sim_experimental_best_effort",
                                  "best-effort simulation-grade RT; authoritative servo + decoupled observability",
                                  {"JointPosition", "CartesianPosition", "Torque"},
                                  "motion_runtime_view",
                                  "simulation_grade",
                                  "xmate6_public_v2026_04",
                                  "effort_owner",
                                  true,
                                  true,
                                  true));
  profiles.push_back(make_profile("rt_hardened",
                                  "runtime",
                                  "simulated_rt_bridge",
                                  "rt_hardened",
                                  "authoritative servo tick + decoupled observability + no legacy RT custom-data fallback",
                                  {"JointPosition", "CartesianPosition", "Torque"},
                                  "motion_runtime_view",
                                  "simulation_grade",
                                  "xmate6_public_v2026_04",
                                  "effort_owner",
                                  true,
                                  true,
                                  false));
  profiles.push_back(make_profile("hard_1khz",
                                  "runtime",
                                  "daemonized_shm_rt_bridge",
                                  "hard_1khz",
                                  "strict 1kHz fail-fast; single authoritative 1ms servo + shm-only ingress + fail-fast scheduler contract",
                                  {"JointPosition", "CartesianPosition", "Torque"},
                                  "motion_runtime_view",
                                  "simulation_grade",
                                  "xmate6_public_v2026_04",
                                  "effort_owner",
                                  true,
                                  true,
                                  false));
  profiles.push_back(make_profile("hybrid_bridge",
                                  "runtime",
                                  "trajectory+effort",
                                  "hybrid_bridge",
                                  "trajectory arbitration + runtime diagnostics",
                                  {"MoveAppend", "QueuedNrt", "RtFollow"},
                                  "motion_runtime_view",
                                  "simulation_grade",
                                  "xmate6_public_v2026_04",
                                  "hybrid_executor",
                                  true,
                                  true,
                                  true));
  profiles.push_back(make_profile("effort_direct",
                                  "effort_owner",
                                  "effort_controller",
                                  "effort_direct",
                                  "effort ownership + watchdog",
                                  {"Torque", "Hold", "Retreat"},
                                  backend_contract.authority_scope,
                                  backend_contract.fidelity_class,
                                  "xmate6_public_v2026_04",
                                  backend_contract.provider_class,
                                  true,
                                  true,
                                  true));
  profiles.push_back(make_profile("jtc_profile",
                                  backend_contract.owner_rule,
                                  backend_contract.required_controller,
                                  "trajectory_jtc",
                                  "goal execution + queue diagnostics",
                                  {"QueuedNrt", "MoveAppend", "ReplayPath"},
                                  backend_contract.authority_scope,
                                  backend_contract.fidelity_class,
                                  "xmate6_public_v2026_04",
                                  backend_contract.provider_class,
                                  false,
                                  backend_contract.fidelity_class != "controller_semantic_parity",
                                  false));

  for (auto &profile : profiles) {
    profile.active = should_activate(backend_contract, profile, active_profile);
    if (profile.name == "rt_sim_experimental_best_effort" && !has_flag(capability_flags, "rt.experimental")) {
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
    stream << ",provider=" << profile.provider_class;
    stream << ",authority=" << profile.authority_scope;
    stream << ",fidelity=" << profile.fidelity_class;
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
