#include "robot_internal.hpp"

namespace rokae::ros2 {

// ==================== IO接口实现 ====================
bool xMateRobot::getDI(unsigned int board, unsigned int port, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_get_di_client_, ec)) {
        return false;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetDI::Request>();
    request->board = board;
    request->port = port;

    auto future = impl_->xmate3_io_get_di_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }

    ec.clear();
    return result->state;
}

bool xMateRobot::getDO(unsigned int board, unsigned int port, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_get_do_client_, ec)) {
        return false;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetDO::Request>();
    request->board = board;
    request->port = port;

    auto future = impl_->xmate3_io_get_do_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
    }

    ec.clear();
    return result->state;
}

void xMateRobot::setDI(unsigned int board, unsigned int port, bool state, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_di_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDI::Request>();
    request->board = board;
    request->port = port;
    request->state = state;

    auto future = impl_->xmate3_io_set_di_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置DI信号失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
}

void xMateRobot::setDO(unsigned int board, unsigned int port, bool state, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_do_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetDO::Request>();
    request->board = board;
    request->port = port;
    request->state = state;

    auto future = impl_->xmate3_io_set_do_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置DO信号失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "DO信号设置成功, board:%d, port:%d, state:%d", board, port, state);
}

double xMateRobot::getAI(unsigned int board, unsigned int port, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return 0.0;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_get_ai_client_, ec)) {
        return 0.0;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::GetAI::Request>();
    request->board = board;
    request->port = port;

    auto future = impl_->xmate3_io_get_ai_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return 0.0;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return 0.0;
    }

    ec.clear();
    return result->value;
}

void xMateRobot::setAO(unsigned int board, unsigned int port, double value, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_ao_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetAO::Request>();
    request->board = board;
    request->port = port;
    request->value = value;

    auto future = impl_->xmate3_io_set_ao_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置AO信号失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
}

void xMateRobot::setSimulationMode(bool state, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }

    if (!impl_->wait_for_service(impl_->xmate3_io_set_simulation_mode_client_, ec)) {
        return;
    }

    auto request = std::make_shared<rokae_xmate3_ros2::srv::SetSimulationMode::Request>();
    request->state = state;

    auto future = impl_->xmate3_io_set_simulation_mode_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }

    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "设置仿真模式失败: %s", result->message.c_str());
        return;
    }

    ec.clear();
    RCLCPP_INFO(impl_->node_->get_logger(), "输入仿真模式设置为: %s", state ? "开启" : "关闭");
}


} // namespace rokae::ros2
