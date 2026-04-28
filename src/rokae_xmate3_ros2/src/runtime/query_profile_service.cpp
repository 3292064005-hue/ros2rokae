#include "runtime/service_facade.hpp"
#include "rokae_xmate3_ros2/spec/xmate_er3_truth.hpp"

#include "runtime/runtime_catalog_service.hpp"
#include "runtime/backend_contract_catalog.hpp"
#include "runtime/runtime_profile_service.hpp"
#include "runtime/planning_capability_service.hpp"
#include "runtime/rt_field_registry.hpp"
#include "runtime/motion_extension_contract.hpp"

namespace rokae_xmate3_ros2::runtime {

void QueryFacade::handleGetProfileCapabilities(
    const rokae_xmate3_ros2::srv::GetProfileCapabilities::Request &req,
    rokae_xmate3_ros2::srv::GetProfileCapabilities::Response &res) const {
  (void)req;
  const auto snapshot = diagnostics_state_.snapshot();
  const auto runtime_view = motion_runtime_.view();
  const auto backend_contract = describeBackendMode(snapshot.backend_mode);
  const auto profiles = buildRuntimeProfileCatalog(backend_contract, snapshot.active_profile, snapshot.capability_flags);
  const auto options = buildRuntimeOptionCatalog(motion_options_state_, session_state_, data_store_state_);

  res.active_profile = snapshot.active_profile;
  res.profile_names.reserve(profiles.size());
  res.owner_rules.reserve(profiles.size());
  res.required_controllers.reserve(profiles.size());
  res.preferred_contracts.reserve(profiles.size());
  res.diagnostics_expectations.reserve(profiles.size());
  res.motion_family_counts.reserve(profiles.size());
  res.authority_scopes.reserve(profiles.size());
  res.fidelity_classes.reserve(profiles.size());
  res.model_revisions.reserve(profiles.size());
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
    res.authority_scopes.push_back(profile.authority_scope);
    res.fidelity_classes.push_back(profile.fidelity_class);
    res.model_revisions.push_back(profile.model_revision);
    res.rt_capable.push_back(profile.rt_capable);
    res.sim_approx.push_back(profile.sim_approx);
    res.experimental.push_back(profile.experimental);
    res.active.push_back(profile.active);
    res.motion_families_flattened.insert(
        res.motion_families_flattened.end(), profile.allowed_motion_families.begin(), profile.allowed_motion_families.end());
  }

  const auto kinematics_backends = buildKinematicsBackendCatalog();
  const auto retimer_policies = buildRetimerPolicyCatalog();
  const auto planner_policies = buildPlannerSelectionCatalog();
  const auto motion_contracts = buildMotionExtensionContracts();
  std::string motion_contract_error;
  if (!validateMotionExtensionContracts(motion_contracts, motion_contract_error)) {
    res.success = false;
    res.message = std::string{"motion extension contract invalid: "} + motion_contract_error;
    return;
  }
  for (const auto &entry : kinematics_backends) {
    res.kinematics_backend_names.push_back(entry.name);
    res.kinematics_backend_summaries.push_back(entry.summary);
    res.kinematics_backend_active.push_back(entry.active);
    res.kinematics_backend_experimental.push_back(entry.experimental);
  }
  for (const auto &entry : retimer_policies) {
    res.retimer_policy_names.push_back(entry.name);
    res.retimer_policy_summaries.push_back(entry.summary);
    res.retimer_policy_active.push_back(entry.active);
    res.retimer_policy_experimental.push_back(entry.experimental);
  }
  for (const auto &entry : planner_policies) {
    res.planner_policy_names.push_back(entry.name);
    res.planner_policy_summaries.push_back(entry.summary);
    res.planner_policy_active.push_back(entry.active);
    res.planner_policy_experimental.push_back(entry.experimental);
  }

  for (const auto &option : options) {
    res.option_names.push_back(option.name);
    res.option_values.push_back(option.value);
    res.option_mutability.push_back(option.mutability);
    res.option_sources.push_back(option.source);
  }

  const auto active_it = std::find_if(profiles.begin(), profiles.end(), [](const auto &profile) { return profile.active; });
  const std::string active_authority = active_it != profiles.end() ? active_it->authority_scope : backend_contract.authority_scope;
  const std::string active_fidelity = active_it != profiles.end() ? active_it->fidelity_class : backend_contract.fidelity_class;
  const std::string active_revision = active_it != profiles.end() ? active_it->model_revision : std::string{rokae_xmate3_ros2::spec::xmate_er3_truth::modelRevision()};

  res.success = true;
  res.message = summarizeRuntimeProfileCatalog(profiles) + "; " +
                summarizePlanningCapabilityCatalog(kinematics_backends, retimer_policies, planner_policies) + "; " +
                summarizeMotionExtensionContracts(motion_contracts) + "; " +
                summarizeRtFieldPolicies(defaultRtFieldSet()) +
                "; query_authority=" + active_authority +
                "; fidelity_class=" + active_fidelity +
                "; model_revision=" + active_revision +
                "; runtime_phase=" + to_string(runtime_view.status.runtime_phase) +
                "; backend_provider=" + backend_contract.provider_class +
                "; rt_policy=" + backend_contract.public_rt_policy;
}

}  // namespace rokae_xmate3_ros2::runtime

