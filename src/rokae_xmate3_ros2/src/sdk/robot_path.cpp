#include "robot_internal.hpp"

namespace rokae::ros2 {
namespace {

bool validate_drag_request(rokae::DragParameter::Space space,
                           rokae::DragParameter::Type type,
                           std::error_code &ec) {
    switch (space) {
        case rokae::DragParameter::Space::jointSpace:
        case rokae::DragParameter::Space::cartesianSpace:
            break;
        default:
            ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
            return false;
    }
    switch (type) {
        case rokae::DragParameter::Type::translationOnly:
        case rokae::DragParameter::Type::rotationOnly:
        case rokae::DragParameter::Type::freely:
            break;
        default:
            ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
            return false;
    }
    return true;
}

} // namespace

// ==================== 拖动与路径录制接口实现 ====================
void xMateRobot::enableDrag(rokae::DragParameter::Space space, rokae::DragParameter::Type type, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!validate_drag_request(space, type, ec)) {
        return;
    }
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_enable_drag_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::EnableDrag::Request>();
    request->space = static_cast<uint8_t>(space);
    request->type = static_cast<uint8_t>(type);

    auto future = impl_->xmate3_cobot_enable_drag_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "开启拖动失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人拖动模式已开启");
}

void xMateRobot::disableDrag(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_disable_drag_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::DisableDrag::Request>();
    auto future = impl_->xmate3_cobot_disable_drag_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "关闭拖动失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "机器人拖动模式已关闭");
}

void xMateRobot::startRecordPath(std::chrono::seconds duration, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (duration <= std::chrono::seconds::zero()) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_start_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::StartRecordPath::Request>();
    request->duration = duration.count();

    auto future = impl_->xmate3_cobot_start_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "开始路径录制失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径录制已开始, 时长: %ld秒", duration.count());
}

void xMateRobot::stopRecordPath(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_stop_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::StopRecordPath::Request>();
    auto future = impl_->xmate3_cobot_stop_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "停止路径录制失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径录制已停止");
}

void xMateRobot::cancelRecordPath(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_cancel_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::CancelRecordPath::Request>();
    auto future = impl_->xmate3_cobot_cancel_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "取消路径录制失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径录制已取消");
}

void xMateRobot::saveRecordPath(const std::string& name, std::error_code& ec, const std::string& saveAs) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (name.empty()) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_save_record_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SaveRecordPath::Request>();
    request->name = name;
    request->save_as = saveAs;

    auto future = impl_->xmate3_cobot_save_record_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "保存路径失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    const auto saved_name = saveAs.empty() ? name : saveAs;
    RCLCPP_INFO(impl_->node_->get_logger(), "路径保存成功, 名称: %s", saved_name.c_str());
}

void xMateRobot::replayPath(const std::string& name, double rate, std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (name.empty() || rate <= 0.0 || rate >= 3.0) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_replay_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::ReplayPath::Request>();
    request->name = name;
    request->rate = rate;

    auto future = impl_->xmate3_cobot_replay_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "路径回放失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径回放已开始, 名称: %s, 速率: %.2f", name.c_str(), rate);
}

void xMateRobot::removePath(const std::string& name, std::error_code& ec, bool removeAll) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!removeAll && name.empty()) {
        ec = rokae::make_error_code(rokae::SdkError::invalid_argument);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_remove_path_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::RemovePath::Request>();
    request->name = name;
    request->remove_all = removeAll;

    auto future = impl_->xmate3_cobot_remove_path_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "删除路径失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "路径删除成功");
}

std::vector<std::string> xMateRobot::queryPathLists(std::error_code& ec) {
    auto _last_error_scope = track_last_error(impl_, ec);
    impl_->ensurePathClients();
    std::vector<std::string> path_list;
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return path_list;
    }

    if (!impl_->wait_for_service(impl_->xmate3_cobot_query_path_lists_client_, ec)) {
        return path_list;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::QueryPathLists::Request>();
    auto future = impl_->xmate3_cobot_query_path_lists_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return path_list;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return path_list;
    }

    path_list = result->path_names;
    ec.clear();
    return path_list;
}


} // namespace rokae::ros2
