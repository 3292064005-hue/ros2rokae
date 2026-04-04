#ifndef ROKAE_XMATE3_ROS2_RUNTIME_ROS_BINDINGS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_ROS_BINDINGS_HPP

#include <array>
#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/msg/rt_fast_command.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/service_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

class RuntimePublishBridge;

struct RosBindingsRtIngressOptions {
  bool enable_topic_rt_ingress = true;
};

class RosBindings {
 public:
  RosBindings(rclcpp::Node::SharedPtr node,
              RuntimeContext &runtime_context,
              RuntimePublishBridge *publish_bridge,
              gazebo::xMate3Kinematics &kinematics,
              JointStateFetcher joint_state_fetcher,
              TimeProvider time_provider,
              TrajectoryDtProvider trajectory_dt_provider,
              RequestIdGenerator request_id_generator,
              RosBindingsRtIngressOptions rt_ingress_options = {});
  ~RosBindings();

 public:
  [[nodiscard]] const rclcpp::Node::SharedPtr &node() const noexcept { return node_; }
  [[nodiscard]] ControlFacade *controlFacade() const noexcept { return control_facade_.get(); }
  [[nodiscard]] QueryFacade *queryFacade() const noexcept { return query_facade_.get(); }
  [[nodiscard]] IoProgramFacade *ioProgramFacade() const noexcept { return io_program_facade_.get(); }
  [[nodiscard]] PathFacade *pathFacade() const noexcept { return path_facade_.get(); }

 private:
  void initServices();
  void registerCompatibilityAliases();
  void initActionServers();
  void initRtIngress();
  void handleRtFastCommand(const rokae_xmate3_ros2::msg::RtFastCommand::SharedPtr &msg);
  void executeMoveAppend(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> &goal_handle);

  rclcpp::Node::SharedPtr node_;
  RuntimeContext &runtime_context_;
  RuntimePublishBridge *publish_bridge_;
  JointStateFetcher joint_state_fetcher_;
  TrajectoryDtProvider trajectory_dt_provider_;
  RequestIdGenerator request_id_generator_;
  RosBindingsRtIngressOptions rt_ingress_options_{};

  std::unique_ptr<ControlFacade> control_facade_;
  std::unique_ptr<QueryFacade> query_facade_;
  std::unique_ptr<IoProgramFacade> io_program_facade_;
  std::unique_ptr<PathFacade> path_facade_;

  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::ServiceBase::SharedPtr> compatibility_services_;
  rclcpp::CallbackGroup::SharedPtr rt_ingress_group_;
  rclcpp::Subscription<rokae_xmate3_ros2::msg::RtFastCommand>::SharedPtr rt_fast_command_sub_;
  rclcpp_action::Server<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr move_append_action_server_;
  std::atomic<bool> move_append_shutdown_requested_{false};
  std::mutex move_append_workers_mutex_;
  std::vector<std::future<void>> move_append_workers_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
