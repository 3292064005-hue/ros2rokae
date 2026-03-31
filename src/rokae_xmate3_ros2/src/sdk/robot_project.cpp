#include "robot_internal.hpp"

namespace rokae::ros2 {

std::vector<rokae::RLProjectInfo> xMateRobot::projectInfo(std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return {};
    }
    if (impl_->xmate3_rl_get_project_info_client_ && impl_->wait_for_service(impl_->xmate3_rl_get_project_info_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetRlProjectInfo::Request>();
        auto future = impl_->xmate3_rl_get_project_info_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                std::vector<rokae::RLProjectInfo> projects;
                for (size_t i = 0; i < result->project_names.size(); ++i) {
                    rokae::RLProjectInfo info;
                    info.name = result->project_names[i];
                    info.is_running = i < result->is_running.size() ? result->is_running[i] : false;
                    info.run_rate = i < result->run_rates.size() ? result->run_rates[i] : 1.0;
                    info.loop_mode = i < result->loop_modes.size() ? result->loop_modes[i] : false;
                    projects.push_back(info);
                }
                std::lock_guard<std::mutex> lock(impl_->state_mutex_);
                impl_->projects_ = projects;
                ec.clear();
                return projects;
            }
        }
    }
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    if (impl_->projects_.empty() && !impl_->current_project_.name.empty()) {
        impl_->projects_.push_back(impl_->current_project_);
    }
    ec.clear();
    return impl_->projects_;
}

void xMateRobot::ppToMain(std::error_code& ec) {
    ec.clear();
}

void xMateRobot::runProject(std::error_code& ec) {
    int current_episode = 0;
    std::string project_name;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        project_name = impl_->current_project_.name;
    }
    if (project_name.empty()) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    (void)startRLProject(project_name, current_episode, ec);
}

void xMateRobot::pauseProject(std::error_code& ec) {
    std::string project_name;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        project_name = impl_->current_project_.name;
    }
    if (project_name.empty()) {
        ec = std::make_error_code(std::errc::invalid_argument);
        return;
    }
    if (impl_->xmate3_rl_pause_project_client_ && impl_->wait_for_service(impl_->xmate3_rl_pause_project_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::PauseRLProject::Request>();
        request->project_id = project_name;
        auto future = impl_->xmate3_rl_pause_project_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                std::lock_guard<std::mutex> lock(impl_->state_mutex_);
                impl_->current_project_.is_running = false;
                ec.clear();
                return;
            }
        }
    }
    int finished_episode = 0;
    (void)stopRLProject(project_name, finished_episode, ec);
}

void xMateRobot::setProjectRunningOpt(double rate, bool loop, std::error_code& ec) {
    if (impl_->xmate3_rl_set_running_opt_client_ && impl_->wait_for_service(impl_->xmate3_rl_set_running_opt_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::SetProjectRunningOpt::Request>();
        request->rate = rate;
        request->loop = loop;
        auto future = impl_->xmate3_rl_set_running_opt_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                std::lock_guard<std::mutex> lock(impl_->state_mutex_);
                impl_->current_project_.run_rate = result->applied_rate;
                impl_->current_project_.loop_mode = result->applied_loop;
                ec.clear();
                return;
            }
        }
    }
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    impl_->current_project_.run_rate = std::max(0.0, rate);
    impl_->current_project_.loop_mode = loop;
    ec.clear();
}

std::vector<rokae::WorkToolInfo> xMateRobot::toolsInfo(std::error_code& ec) {
    if (impl_->connected_ && impl_->xmate3_rl_get_tools_info_client_ && impl_->wait_for_service(impl_->xmate3_rl_get_tools_info_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetToolCatalog::Request>();
        auto future = impl_->xmate3_rl_get_tools_info_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                std::vector<rokae::WorkToolInfo> tools;
                const size_t count = result->names.size();
                tools.reserve(count);
                for (size_t i = 0; i < count; ++i) {
                    rokae::WorkToolInfo info;
                    info.name = result->names[i];
                    info.alias = i < result->aliases.size() ? result->aliases[i] : result->names[i];
                    info.robotHeld = i < result->robot_held.size() ? result->robot_held[i] : true;
                    if (i * 6 + 5 < result->pose_flattened.size()) {
                        std::array<double, 6> pose{};
                        for (size_t j = 0; j < 6; ++j) {
                            pose[j] = result->pose_flattened[i * 6 + j];
                        }
                        info.pos = rokae::Frame(pose);
                    }
                    if (i < result->masses.size()) {
                        info.load.mass = result->masses[i];
                    }
                    tools.push_back(info);
                }
                {
                    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
                    impl_->tools_ = tools;
                }
                ec.clear();
                return tools;
            }
        }
    }
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    if (impl_->tools_.empty()) {
        rokae::WorkToolInfo info;
        info.name = impl_->toolset_cache_.tool_name.empty() ? "tool0" : impl_->toolset_cache_.tool_name;
        info.alias = info.name;
        info.robotHeld = true;
        info.pos = impl_->toolset_cache_.end;
        info.load = impl_->toolset_cache_.load;
        impl_->tools_.push_back(info);
    }
    ec.clear();
    return impl_->tools_;
}

std::vector<rokae::WorkToolInfo> xMateRobot::wobjsInfo(std::error_code& ec) {
    if (impl_->connected_ && impl_->xmate3_rl_get_wobjs_info_client_ && impl_->wait_for_service(impl_->xmate3_rl_get_wobjs_info_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::GetWobjCatalog::Request>();
        auto future = impl_->xmate3_rl_get_wobjs_info_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                std::vector<rokae::WorkToolInfo> wobjs;
                const size_t count = result->names.size();
                wobjs.reserve(count);
                for (size_t i = 0; i < count; ++i) {
                    rokae::WorkToolInfo info;
                    info.name = result->names[i];
                    info.alias = i < result->aliases.size() ? result->aliases[i] : result->names[i];
                    info.robotHeld = i < result->robot_held.size() ? result->robot_held[i] : false;
                    if (i * 6 + 5 < result->pose_flattened.size()) {
                        std::array<double, 6> pose{};
                        for (size_t j = 0; j < 6; ++j) {
                            pose[j] = result->pose_flattened[i * 6 + j];
                        }
                        info.pos = rokae::Frame(pose);
                    }
                    wobjs.push_back(info);
                }
                {
                    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
                    impl_->wobjs_ = wobjs;
                }
                ec.clear();
                return wobjs;
            }
        }
    }
    std::lock_guard<std::mutex> lock(impl_->state_mutex_);
    if (impl_->wobjs_.empty()) {
        rokae::WorkToolInfo info;
        info.name = impl_->toolset_cache_.wobj_name.empty() ? "wobj0" : impl_->toolset_cache_.wobj_name;
        info.alias = info.name;
        info.robotHeld = false;
        info.pos = impl_->toolset_cache_.ref;
        impl_->wobjs_.push_back(info);
    }
    ec.clear();
    return impl_->wobjs_;
}

bool xMateRobot::loadRLProject(const std::string& project_path, std::string& project_name, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rl_load_project_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::LoadRLProject::Request>();
    request->project_path = project_path;
    auto future = impl_->xmate3_rl_load_project_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "加载RL工程失败: %s", result->error_msg.c_str());
        return false;
    }
    project_name = result->project_name;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        impl_->current_project_.name = project_name;
        impl_->current_project_.is_running = false;
        auto it = std::find_if(impl_->projects_.begin(), impl_->projects_.end(), [&](const rokae::RLProjectInfo& info) {
            return info.name == project_name;
        });
        if (it == impl_->projects_.end()) {
            impl_->projects_.push_back(impl_->current_project_);
        } else {
            *it = impl_->current_project_;
        }
    }
    ec.clear();
    return true;
}

bool xMateRobot::startRLProject(const std::string& project_id, int& current_episode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rl_start_project_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::StartRLProject::Request>();
    request->project_id = project_id;
    auto future = impl_->xmate3_rl_start_project_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    current_episode = result->current_episode;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        impl_->current_project_.name = project_id;
        impl_->current_project_.is_running = true;
    }
    ec.clear();
    return true;
}

bool xMateRobot::stopRLProject(const std::string& project_id, int& finished_episode, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_rl_stop_project_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::StopRLProject::Request>();
    request->project_id = project_id;
    auto future = impl_->xmate3_rl_stop_project_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }
    finished_episode = result->finished_episode;
    {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        if (impl_->current_project_.name == project_id) {
            impl_->current_project_.is_running = false;
        }
    }
    ec.clear();
    return true;
}


} // namespace rokae::ros2
