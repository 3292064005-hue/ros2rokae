#include "runtime/service_facade.hpp"

#include "runtime/runtime_catalog_service.hpp"
#include "runtime/runtime_profile_service.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleGetProfileCapabilities(
    const rokae_xmate3_ros2::srv::GetProfileCapabilities::Request &req,
    rokae_xmate3_ros2::srv::GetProfileCapabilities::Response &res) const {
  (void)req;
  const auto snapshot = diagnostics_state_.snapshot();
  const auto profiles = buildRuntimeProfileCatalog(snapshot.backend_mode, snapshot.active_profile, snapshot.capability_flags);
  const auto options = buildRuntimeOptionCatalog(motion_options_state_, session_state_);

  res.active_profile = snapshot.active_profile;
  res.profile_names.reserve(profiles.size());
  res.owner_rules.reserve(profiles.size());
  res.required_controllers.reserve(profiles.size());
  res.preferred_contracts.reserve(profiles.size());
  res.diagnostics_expectations.reserve(profiles.size());
  res.motion_family_counts.reserve(profiles.size());
  res.rt_capable.reserve(profiles.size());
  res.sim_approx.reserve(profiles.size());
  res.experimental.reserve(profiles.size());
  res.active.reserve(profiles.size());

  for (const auto &profile : profiles) {
    res.profile_names.push_back(profile.name);
    res.owner_rules.push_back(profile.owner_rule);
    res.required_controllers.push_back(profile.required_controller);
    res.preferred_contracts.push_back(profile.preferred_contract);
    res.diagnostics_expectations.push_back(profile.diagnostics_expectation);
    res.motion_family_counts.push_back(static_cast<std::uint32_t>(profile.allowed_motion_families.size()));
    res.rt_capable.push_back(profile.rt_capable);
    res.sim_approx.push_back(profile.sim_approx);
    res.experimental.push_back(profile.experimental);
    res.active.push_back(profile.active);
    res.motion_families_flattened.insert(
        res.motion_families_flattened.end(), profile.allowed_motion_families.begin(), profile.allowed_motion_families.end());
  }

  for (const auto &option : options) {
    res.option_names.push_back(option.name);
    res.option_values.push_back(option.value);
    res.option_mutability.push_back(option.mutability);
    res.option_sources.push_back(option.source);
  }

  res.success = true;
  res.message = summarizeRuntimeProfileCatalog(profiles);
}

}  // namespace rokae_xmate3_ros2::runtime
