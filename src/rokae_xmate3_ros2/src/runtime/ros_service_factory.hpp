#ifndef ROKAE_XMATE3_ROS2_RUNTIME_ROS_SERVICE_FACTORY_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_ROS_SERVICE_FACTORY_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace rokae_xmate3_ros2::runtime::detail {

template <typename ServiceT, typename FacadeT>
rclcpp::ServiceBase::SharedPtr CreateFacadeService(
    const rclcpp::Node::SharedPtr &node,
    const std::string &name,
    FacadeT *facade,
    void (FacadeT::*method)(const typename ServiceT::Request &, typename ServiceT::Response &) const) {
  return node->create_service<ServiceT>(
      name,
      [facade, method](const std::shared_ptr<typename ServiceT::Request> req,
                       std::shared_ptr<typename ServiceT::Response> res) {
        (facade->*method)(*req, *res);
      });
}

inline bool is_retryable_submission_failure(const std::string &message) {
  return message == "runtime is busy with another motion request" ||
         message == "runtime planner queue is busy";
}

}  // namespace rokae_xmate3_ros2::runtime::detail

#endif
