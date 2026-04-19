#include "robot_internal.hpp"
#include "runtime/backend_contract_catalog.hpp"
#include "runtime/runtime_profile_service.hpp"

namespace rokae::ros2 {

bool xMateRobot::getProfileCapabilities(std::string& active_profile,
                                        std::vector<rokae::RuntimeProfileCapability>& profiles,
                                        std::vector<rokae::RuntimeOptionDescriptor>& options,
                                        std::error_code& ec) {
#if !ROKAE_ENABLE_INTERNAL_SURFACE
    auto _last_error_scope = track_last_error(impl_, ec);
    profiles.clear();
    options.clear();
    active_profile.clear();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    std::string inferred_profile{"nrt_strict_parity"};
    if (impl_->xmate3_internal_get_runtime_state_snapshot_client_ &&
        impl_->wait_for_service(impl_->xmate3_internal_get_runtime_state_snapshot_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetRuntimeStateSnapshot::Request>();
        auto future = impl_->xmate3_internal_get_runtime_state_snapshot_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            const auto result = future.get();
            if (result->success && !result->active_profile.empty()) {
                inferred_profile = result->active_profile;
            }
        }
    }
    std::string backend_mode = "jtc";
    if (inferred_profile == "hybrid_bridge") {
        backend_mode = "hybrid";
    } else if (inferred_profile == "rt_hardened" ||
               inferred_profile == "hard_1khz" ||
               inferred_profile == "rt_sim_experimental_best_effort" ||
               inferred_profile == "effort_direct") {
        backend_mode = "effort";
    }
    const auto backend_contract = rokae_xmate3_ros2::runtime::describeBackendMode(backend_mode);
    const auto descriptors = rokae_xmate3_ros2::runtime::buildRuntimeProfileCatalog(backend_contract,
                                                                                     inferred_profile,
                                                                                     backend_contract.baseline_capability_flags);
    active_profile = inferred_profile;
    for (const auto &descriptor : descriptors) {
        rokae::RuntimeProfileCapability profile;
        profile.name = descriptor.name;
        profile.owner_rule = descriptor.owner_rule;
        profile.required_controller = descriptor.required_controller;
        profile.preferred_contract = descriptor.preferred_contract;
        profile.diagnostics_expectation = descriptor.diagnostics_expectation;
        profile.allowed_motion_families = descriptor.allowed_motion_families;
        profile.authority_scope = descriptor.authority_scope;
        profile.fidelity_class = descriptor.fidelity_class;
        profile.model_revision = descriptor.model_revision;
        profile.rt_capable = descriptor.rt_capable;
        profile.sim_approx = descriptor.sim_approx;
        profile.experimental = descriptor.experimental;
        profile.active = descriptor.active;
        profiles.push_back(profile);
    }
    rokae::RuntimeOptionDescriptor transport_option;
    transport_option.name = "transport_mode";
    transport_option.value = "shm_topic";
    transport_option.mutability = "runtime_default";
    transport_option.source = "sdk_static_fallback";
    options.push_back(transport_option);

    rokae::RuntimeOptionDescriptor profile_source_option;
    profile_source_option.name = "profile_source";
    profile_source_option.value = "sdk_static_fallback";
    profile_source_option.mutability = "compile_time";
    profile_source_option.source = "sdk_static_fallback";
    options.push_back(profile_source_option);
    ec.clear();
    return true;
#else
    auto _last_error_scope = track_last_error(impl_, ec);
    profiles.clear();
    options.clear();
    active_profile.clear();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->xmate3_internal_get_profile_capabilities_client_ ||
        !impl_->wait_for_service(impl_->xmate3_internal_get_profile_capabilities_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetProfileCapabilities::Request>();
    auto future = impl_->xmate3_internal_get_profile_capabilities_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }

    active_profile = result->active_profile;
    profiles.reserve(result->profile_names.size());
    std::size_t offset = 0;
    for (std::size_t i = 0; i < result->profile_names.size(); ++i) {
        rokae::RuntimeProfileCapability profile;
        profile.name = result->profile_names[i];
        if (i < result->owner_rules.size()) profile.owner_rule = result->owner_rules[i];
        if (i < result->required_controllers.size()) profile.required_controller = result->required_controllers[i];
        if (i < result->preferred_contracts.size()) profile.preferred_contract = result->preferred_contracts[i];
        if (i < result->diagnostics_expectations.size()) profile.diagnostics_expectation = result->diagnostics_expectations[i];
        const std::size_t family_count = i < result->motion_family_counts.size() ? result->motion_family_counts[i] : 0U;
        for (std::size_t j = 0; j < family_count && offset + j < result->motion_families_flattened.size(); ++j) {
            profile.allowed_motion_families.push_back(result->motion_families_flattened[offset + j]);
        }
        offset += family_count;
        if (i < result->authority_scopes.size()) profile.authority_scope = result->authority_scopes[i];
        if (i < result->fidelity_classes.size()) profile.fidelity_class = result->fidelity_classes[i];
        if (i < result->model_revisions.size()) profile.model_revision = result->model_revisions[i];
        if (i < result->rt_capable.size()) profile.rt_capable = result->rt_capable[i];
        if (i < result->sim_approx.size()) profile.sim_approx = result->sim_approx[i];
        if (i < result->experimental.size()) profile.experimental = result->experimental[i];
        if (i < result->active.size()) profile.active = result->active[i];
        profiles.push_back(profile);
    }

    options.reserve(result->option_names.size());
    for (std::size_t i = 0; i < result->option_names.size(); ++i) {
        rokae::RuntimeOptionDescriptor option;
        option.name = result->option_names[i];
        if (i < result->option_values.size()) option.value = result->option_values[i];
        if (i < result->option_mutability.size()) option.mutability = result->option_mutability[i];
        if (i < result->option_sources.size()) option.source = result->option_sources[i];
        options.push_back(option);
    }
    ec.clear();
    return true;
#endif
}

} // namespace rokae::ros2
