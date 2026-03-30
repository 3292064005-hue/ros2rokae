#include "runtime/ros_bindings.hpp"

#include <array>
#include <chrono>
#include <thread>

#include "runtime/ros_service_factory.hpp"
#include "runtime/runtime_publish_bridge.hpp"

namespace rokae_xmate3_ros2::runtime {

void RosBindings::initActionServers() {
  move_append_action_server_ = rclcpp_action::create_server<rokae_xmate3_ros2::action::MoveAppend>(
      node_,
      "/xmate3/cobot/move_append",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const rokae_xmate3_ros2::action::MoveAppend::Goal> goal) {
        (void)uuid;
        (void)goal;
        if (!runtime_context_.sessionState().powerOn()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {
        (void)goal_handle;
        runtime_context_.requestCoordinator().stop("move append canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> goal_handle) {
        std::thread([this, goal_handle]() { executeMoveAppend(goal_handle); }).detach();
      });
}

void RosBindings::executeMoveAppend(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> &goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();

  std::array<double, 6> cached_pos{};
  std::array<double, 6> cached_vel{};
  std::array<double, 6> cached_torque{};
  joint_state_fetcher_(cached_pos, cached_vel, cached_torque);

  const auto request_id = request_id_generator_("move_");
  SubmissionResult submission;
  constexpr auto kSubmissionRetryTimeout = std::chrono::milliseconds(1200);
  constexpr auto kSubmissionRetrySleep = std::chrono::milliseconds(10);
  const auto retry_deadline = std::chrono::steady_clock::now() + kSubmissionRetryTimeout;

  while (true) {
    joint_state_fetcher_(cached_pos, cached_vel, cached_torque);
    submission = runtime_context_.requestCoordinator().submitMoveAppend(
        *goal,
        cached_pos,
        trajectory_dt_provider_(),
        request_id);
    if (submission.success) {
      break;
    }
    if (!detail::is_retryable_submission_failure(submission.message) ||
        std::chrono::steady_clock::now() >= retry_deadline) {
      break;
    }
    std::this_thread::sleep_for(kSubmissionRetrySleep);
  }

  if (!submission.success) {
    result->success = false;
    result->cmd_id = request_id;
    result->message = submission.message;
    goal_handle->abort(result);
    return;
  }

  if (publish_bridge_ != nullptr) {
    publish_bridge_->driveMoveAppendGoal(goal_handle, request_id);
    return;
  }

  result->success = false;
  result->cmd_id = request_id;
  result->message = "MoveAppend bridge unavailable";
  goal_handle->abort(result);
}

}  // namespace rokae_xmate3_ros2::runtime
