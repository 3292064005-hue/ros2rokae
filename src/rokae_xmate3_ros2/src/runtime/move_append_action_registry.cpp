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
        // Defer cancellation side effects to executeMoveAppend() so queue-only goals do not
        // abort unrelated runtime work before the request_id is known.
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

/**
 * @brief Queue a public-lane NRT MoveAppend request and return as soon as the runtime accepts it.
 * @param goal_handle Accepted ROS action goal handle.
 * @throws None. All runtime and transport failures are converted into deterministic action aborts.
 * @note Boundary behavior: in the public xMate6 lane MoveAppend is queue-only. This action must not
 *       wait for execution terminal states; moveStart() remains the sole start/resume authority and
 *       terminal execution outcomes are observed through runtime state / event surfaces afterwards.
 */
void RosBindings::executeMoveAppend(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<rokae_xmate3_ros2::action::MoveAppend>> &goal_handle) {
  const auto goal = goal_handle->get_goal();
  const auto request_id = request_id_generator_("move_");
  bool queue_accepted = false;
  try {
    auto result = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();

    std::array<double, 6> cached_pos{};
    std::array<double, 6> cached_vel{};
    std::array<double, 6> cached_torque{};
    joint_state_fetcher_(cached_pos, cached_vel, cached_torque);

    SubmissionResult submission;
    constexpr auto kSubmissionRetryTimeout = std::chrono::milliseconds(1200);
    constexpr auto kSubmissionRetrySleep = std::chrono::milliseconds(10);
    const auto retry_deadline = std::chrono::steady_clock::now() + kSubmissionRetryTimeout;

    while (!move_append_shutdown_requested_.load()) {
      if (goal_handle->is_canceling()) {
        auto canceled = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
        canceled->success = false;
        canceled->cmd_id = request_id;
        canceled->message = "Canceled before queue acceptance";
        goal_handle->canceled(canceled);
        return;
      }
      joint_state_fetcher_(cached_pos, cached_vel, cached_torque);
      submission = runtime_context_.requestCoordinator().queueMoveAppend(
          *goal,
          cached_pos,
          trajectory_dt_provider_(),
          request_id);
      if (submission.success) {
        queue_accepted = true;
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

    const auto queued_status = runtime_context_.requestCoordinator().waitForUpdate(
        request_id,
        0,
        std::chrono::milliseconds(0));
    const auto feedback_snapshot = buildMoveAppendFeedback(
        queued_status,
        std::numeric_limits<std::size_t>::max(),
        std::string{});
    if (feedback_snapshot.should_publish) {
      goal_handle->publish_feedback(publish_bridge_->buildMoveAppendFeedbackMessage(feedback_snapshot));
    }

    // Once the runtime has accepted the request into the queue, MoveAppend is complete in the
    // public xMate6 lane. Late cancellations do not mutate runtime state or retract the queued work.
    goal_handle->succeed(publish_bridge_->buildMoveAppendQueuedResult(request_id, submission.message));
  } catch (const std::exception &ex) {
    if (queue_accepted) {
      try {
        if (publish_bridge_ != nullptr && !goal_handle->is_canceling()) {
          goal_handle->succeed(publish_bridge_->buildMoveAppendQueuedResult(request_id, "queued awaiting moveStart"));
          return;
        }
      } catch (...) {
      }
    }
    auto failed = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
    failed->success = false;
    failed->cmd_id = request_id;
    failed->message = std::string{"MoveAppend internal exception: "} + ex.what();
    goal_handle->abort(failed);
  } catch (...) {
    if (queue_accepted) {
      try {
        if (publish_bridge_ != nullptr && !goal_handle->is_canceling()) {
          goal_handle->succeed(publish_bridge_->buildMoveAppendQueuedResult(request_id, "queued awaiting moveStart"));
          return;
        }
      } catch (...) {
      }
    }
    auto failed = std::make_shared<rokae_xmate3_ros2::action::MoveAppend::Result>();
    failed->success = false;
    failed->cmd_id = request_id;
    failed->message = "MoveAppend internal exception: unknown";
    goal_handle->abort(failed);
  }
}

}  // namespace rokae_xmate3_ros2::runtime
