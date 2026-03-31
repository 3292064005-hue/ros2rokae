#include "robot_internal.hpp"

namespace rokae::ros2 {

std::string xMateRobot::sendCustomData(const std::string& topic,
                                       const std::string& payload,
                                       std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return {};
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_send_custom_data_client_, ec)) {
        return {};
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::SendCustomData::Request>();
    request->data_topic = topic;
    request->custom_data = payload;
    auto future = impl_->xmate3_comm_send_custom_data_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return {};
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "发送自定义数据失败: %s", result->error_msg.c_str());
        return {};
    }
    ec.clear();
    return result->response_data;
}

bool xMateRobot::registerDataCallback(const std::string& data_topic,
                                      const std::string& callback_id,
                                      std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return false;
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_register_data_callback_client_, ec)) {
        return false;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::RegisterDataCallback::Request>();
    request->data_topic = data_topic;
    request->callback_id = callback_id;
    auto future = impl_->xmate3_comm_register_data_callback_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return false;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        RCLCPP_ERROR(impl_->node_->get_logger(), "注册数据回调失败: %s", result->error_msg.c_str());
        return false;
    }
    ec.clear();
    return true;
}

std::string xMateRobot::readRegister(const std::string& name, int index, std::error_code& ec) {
    if (impl_->connected_ && impl_->xmate3_comm_read_register_ex_client_ && impl_->wait_for_service(impl_->xmate3_comm_read_register_ex_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::ReadRegisterEx::Request>();
        request->name = name;
        request->index = index;
        auto future = impl_->xmate3_comm_read_register_ex_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                ec.clear();
                return result->value;
            }
        }
    }
    return readRegister(name + "[" + std::to_string(index) + "]", ec);
}

std::string xMateRobot::readRegister(const std::string& key, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return {};
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_read_register_client_, ec)) {
        return {};
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::ReadRegister::Request>();
    request->key = key;
    auto future = impl_->xmate3_comm_read_register_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return {};
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return {};
    }
    ec.clear();
    return result->value;
}

void xMateRobot::writeRegister(const std::string& name, int index, const std::string& value, std::error_code& ec) {
    if (impl_->connected_ && impl_->xmate3_comm_write_register_ex_client_ && impl_->wait_for_service(impl_->xmate3_comm_write_register_ex_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::WriteRegisterEx::Request>();
        request->name = name;
        request->index = index;
        request->value = value;
        auto future = impl_->xmate3_comm_write_register_ex_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                ec.clear();
                return;
            }
        }
    }
    writeRegister(name + "[" + std::to_string(index) + "]", value, ec);
}

void xMateRobot::setxPanelVout(rokae::xPanelOpt::Vout opt, std::error_code& ec) {
    if (impl_->connected_ && impl_->xmate3_comm_set_xpanel_vout_client_ && impl_->wait_for_service(impl_->xmate3_comm_set_xpanel_vout_client_, ec)) {
        auto request = std::make_shared<rokae_xmate3_ros2::srv::SetXPanelVout::Request>();
        request->mode = static_cast<int>(opt);
        auto future = impl_->xmate3_comm_set_xpanel_vout_client_->async_send_request(request);
        if (impl_->wait_for_future(future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = future.get();
            if (result->success) {
                std::lock_guard<std::mutex> lock(impl_->state_mutex_);
                impl_->xpanel_vout_ = opt;
                ec.clear();
                return;
            }
        }
    }
    writeRegister("xpanel_vout", std::to_string(static_cast<int>(opt)), ec);
    if (!ec) {
        std::lock_guard<std::mutex> lock(impl_->state_mutex_);
        impl_->xpanel_vout_ = opt;
    }
}

void xMateRobot::writeRegister(const std::string& key, const std::string& value, std::error_code& ec) {
    if (!impl_->connected_) {
        ec = std::make_error_code(std::errc::not_connected);
        return;
    }
    if (!impl_->wait_for_service(impl_->xmate3_comm_write_register_client_, ec)) {
        return;
    }
    auto request = std::make_shared<rokae_xmate3_ros2::srv::WriteRegister::Request>();
    request->key = key;
    request->value = value;
    auto future = impl_->xmate3_comm_write_register_client_->async_send_request(request);
    if (impl_->wait_for_future(future) != rclcpp::FutureReturnCode::SUCCESS) {
        ec = std::make_error_code(std::errc::io_error);
        return;
    }
    auto result = future.get();
    if (!result->success) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return;
    }
    ec.clear();
}

} // namespace rokae::ros2
