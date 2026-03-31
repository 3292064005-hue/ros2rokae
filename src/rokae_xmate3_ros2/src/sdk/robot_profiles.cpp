#include "robot_internal.hpp"

namespace rokae::ros2 {

bool xMateRobot::getProfileCapabilities(std::string& active_profile,
                                        std::vector<rokae::RuntimeProfileCapability>& profiles,
                                        std::vector<rokae::RuntimeOptionDescriptor>& options,
                                        std::error_code& ec) {
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
}

} // namespace rokae::ros2
