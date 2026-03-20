#ifndef ROKAE_XMATE3_ROS2_RUNTIME_ROS_BINDINGS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_ROS_BINDINGS_HPP

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/service_facade.hpp"

namespace rokae_xmate3_ros2::runtime {

class RosBindings {
 public:
  RosBindings(rclcpp::Node::SharedPtr node,
              RuntimeContext &runtime_context,
              gazebo::xMate3Kinematics &kinematics,
              JointStateFetcher joint_state_fetcher,
              TimeProvider time_provider,
              TrajectoryDtProvider trajectory_dt_provider,
              RequestIdGenerator request_id_generator);

  [[nodiscard]] bool isRecordingPath() const;
  void recordPathSample(const std::array<double, 6> &joint_position) const;

 private:
  void initServices();
  void initActionServers();
  void executeMoveAppend(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> &goal_handle);

  rclcpp::Node::SharedPtr node_;
  RuntimeContext &runtime_context_;
  JointStateFetcher joint_state_fetcher_;
  TrajectoryDtProvider trajectory_dt_provider_;
  RequestIdGenerator request_id_generator_;

  std::unique_ptr<ControlFacade> control_facade_;
  std::unique_ptr<QueryFacade> query_facade_;
  std::unique_ptr<IoProgramFacade> io_program_facade_;
  std::unique_ptr<PathFacade> path_facade_;

  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  std::vector<rclcpp::ServiceBase::SharedPtr> compatibility_services_;
  rclcpp_action::Server<rokae_xmate3_ros2::action::MoveAppend>::SharedPtr move_append_action_server_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
