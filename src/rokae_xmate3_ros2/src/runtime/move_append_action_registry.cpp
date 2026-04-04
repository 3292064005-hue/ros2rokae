#include "runtime/ros_bindings.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <future>
#include <limits>
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
        if (!runtime_context_.sessionState().powerOn() || move_append_shutdown_requested_.load()) {
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
        std::lock_guard<std::mutex> lock(move_append_workers_mutex_);
        move_append_workers_.erase(
            std::remove_if(
                move_append_workers_.begin(),
                move_append_workers_.end(),
                [](std::future<void> &worker) {
                  return worker.valid() && worker.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
                }),
            move_append_workers_.end());
        move_append_workers_.emplace_back(
            std::async(std::launch::async, [this, goal_handle]() { executeMoveAppend(goal_handle); }));
      });
}

void RosBindings::executeMoveAppend(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> &goal_handle) {
  try {
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
    constexpr auto kGoalPollTimeout = std::chrono::milliseconds(100);
    const auto retry_deadline = std::chrono::steady_clock::now() + kSubmissionRetryTimeout;

    while (!move_append_shutdown_requested_.load()) {
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

    if (move_append_shutdown_requested_.load()) {
      result->success = false;
      result->cmd_id = request_id;
      result->message = "MoveAppend shutdown requested";
      goal_handle->abort(result);
      return;
    }

    if (!submission.success) {
      result->success = false;
      result->cmd_id = request_id;
      result->message = submission.message;
      goal_handle->abort(result);
      return;
    }

    if (publish_bridge_ == nullptr) {
      result->success = false;
      result->cmd_id = request_id;
      result->message = "MoveAppend bridge unavailable";
      goal_handle->abort(result);
      return;
    }

    std::string last_state;
    std::size_t last_completed = std::numeric_limits<std::size_t>::max();
    std::uint64_t last_revision = 0;

    while (!move_append_shutdown_requested_.load() && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        runtime_context_.requestCoordinator().stop("move append canceled");
        auto canceled = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
        canceled->success = false;
        canceled->cmd_id = request_id;
        canceled->message = "Canceled";
        goal_handle->canceled(canceled);
        return;
      }

      const auto status = publish_bridge_->waitForRequestUpdate(request_id, last_revision, kGoalPollTimeout);
      if (status.revision > last_revision) {
        last_revision = status.revision;
      }

      const auto feedback_snapshot = buildMoveAppendFeedback(status, last_completed, last_state);
      if (feedback_snapshot.should_publish) {
        goal_handle->publish_feedback(publish_bridge_->buildMoveAppendFeedbackMessage(feedback_snapshot));
        last_completed = status.completed_segments;
        last_state = status.message;
      }

      if (status.terminal()) {
        auto terminal = publish_bridge_->buildMoveAppendResult(request_id, status);
        if (terminal->success) {
          goal_handle->succeed(terminal);
        } else if (goal_handle->is_canceling()) {
          goal_handle->canceled(terminal);
        } else {
          goal_handle->abort(terminal);
        }
        return;
      }
    }

    runtime_context_.requestCoordinator().stop("move append shutdown requested");
    result->success = false;
    result->cmd_id = request_id;
    result->message = move_append_shutdown_requested_.load() ? "MoveAppend shutdown requested" : "MoveAppend interrupted";
    goal_handle->abort(result);
  } catch (const std::exception &ex) {
    runtime_context_.requestCoordinator().stop("move append exception");
    auto failed = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
    failed->success = false;
    failed->message = std::string{"MoveAppend internal exception: "} + ex.what();
    goal_handle->abort(failed);
  } catch (...) {
    runtime_context_.requestCoordinator().stop("move append unknown exception");
    auto failed = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
    failed->success = false;
    failed->message = "MoveAppend internal exception: unknown";
    goal_handle->abort(failed);
  }
}

}  // namespace rokae_xmate3_ros2::runtime
