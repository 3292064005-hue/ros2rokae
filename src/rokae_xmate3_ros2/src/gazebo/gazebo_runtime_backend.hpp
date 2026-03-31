#ifndef ROKAE_XMATE3_ROS2_GAZEBO_RUNTIME_BACKEND_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_RUNTIME_BACKEND_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
#include "runtime/runtime_types.hpp"

namespace gazebo {
namespace runtime = rokae_xmate3_ros2::runtime;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

enum class BackendMode {
  effort,
  jtc,
  hybrid,
};

namespace {

constexpr double kDefaultTrajectorySampleDt = 0.01;
constexpr double kMinTrajectorySampleDt = 0.001;
constexpr double kMaxTrajectorySampleDt = 0.05;
constexpr double kTrajectoryCompletionInferenceGraceSec = 0.35;
constexpr double kTrajectoryCompletionWallClockGraceSec = 0.5;
constexpr double kTrajectoryCompletionInferenceVelocityRadPerSec = 0.02;

std::array<double, 6> vectorToArray(const std::vector<double> &values,
                                    const std::array<double, 6> &fallback,
                                    double min_value) {
  std::array<double, 6> resolved = fallback;
  if (values.size() < resolved.size()) {
    return resolved;
  }
  for (std::size_t i = 0; i < resolved.size(); ++i) {
    if (!std::isfinite(values[i]) || values[i] <= min_value) {
      return fallback;
    }
    resolved[i] = values[i];
  }
  return resolved;
}

BackendMode parseBackendMode(const std::string &value) {
  if (value == "effort") {
    return BackendMode::effort;
  }
  if (value == "jtc") {
    return BackendMode::jtc;
  }
  return BackendMode::hybrid;
}

const char *toString(BackendMode mode) {
  switch (mode) {
    case BackendMode::effort:
      return "effort";
    case BackendMode::jtc:
      return "jtc";
    case BackendMode::hybrid:
    default:
      return "hybrid";
  }
}

std::vector<std::string> diagnosticCapabilityFlags(BackendMode mode) {
  std::vector<std::string> flags{
      "simulation.gazebo11",
      "ros2.humble",
      "rt.experimental",
      "compat.alias.get_joint_torque",
      "compat.alias.get_end_torque",
      "diagnostics.runtime_status",
      "diagnostics.get_runtime_diagnostics",
      "planning.validate_motion",
  };
  flags.push_back(std::string("backend.") + toString(mode));
  return flags;
}

double maxAbsVector(const std::vector<double> &values) {
  double max_value = 0.0;
  for (double value : values) {
    max_value = std::max(max_value, std::abs(value));
  }
  return max_value;
}

}  // namespace

class GazeboRuntimeBackend final : public runtime::BackendInterface {
 public:
  GazeboRuntimeBackend(std::vector<physics::JointPtr> *joints,
                       const std::array<std::pair<double, double>, 6> *original_joint_limits)
      : joints_(joints), original_joint_limits_(original_joint_limits) {}

  void beginShutdown() {
    shutting_down_.store(true);
    control_owner_.store(runtime::ControlOwner::none);
    joints_ = nullptr;
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    ++trajectory_generation_;
    trajectory_state_.active = false;
    trajectory_state_.completed = true;
    trajectory_state_.canceled = true;
    trajectory_state_.message = "backend shutdown";
    goal_handle_.reset();
    trajectory_client_.reset();
  }

  void configureTrajectoryClient(const rclcpp::Node::SharedPtr &node,
                                 const std::vector<std::string> &joint_names) {
    node_ = node;
    trajectory_joint_names_ = joint_names;
    if (node_ != nullptr) {
      trajectory_client_ =
          rclcpp_action::create_client<FollowJointTrajectory>(node_, "/joint_trajectory_controller/follow_joint_trajectory");
    }
  }

  runtime::RobotSnapshot readSnapshot() const override {
    runtime::RobotSnapshot snapshot;
    if (shutting_down_.load() || !joints_) {
      return snapshot;
    }
    for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
      snapshot.joint_position[i] = (*joints_)[i]->Position(0);
      snapshot.joint_velocity[i] = (*joints_)[i]->GetVelocity(0);
      snapshot.joint_torque[i] = (*joints_)[i]->GetForce(0);
    }
    return snapshot;
  }

  void setControlOwner(runtime::ControlOwner owner) override { control_owner_.store(owner); }

  [[nodiscard]] runtime::ControlOwner controlOwner() const override {
    return control_owner_.load();
  }

  void applyControl(const runtime::ControlCommand &command) override {
    if (shutting_down_.load() || !joints_ || !command.has_effort ||
        control_owner_.load() != runtime::ControlOwner::effort) {
      clearControl();
      return;
    }
    for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
      (*joints_)[i]->SetForce(0, command.effort[i]);
    }
  }

  void clearControl() override {
    if (shutting_down_.load() || !joints_) {
      return;
    }
    for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
      (*joints_)[i]->SetForce(0, 0.0);
    }
  }

  void setBrakeLock(const runtime::RobotSnapshot &snapshot, bool locked) override {
    if (shutting_down_.load() || !joints_ || locked == brakes_locked_) {
      return;
    }

    if (locked) {
      clearControl();
      for (size_t i = 0; i < 6 && i < joints_->size(); ++i) {
        // Avoid mutating Gazebo joint limits while gazebo_ros2_control owns the model.
        // Locking the simulated brake is represented by clearing effort commands and
        // zeroing the commanded joint velocity so the SDK-facing state machine remains
        // deterministic without destabilizing controller_manager startup.
        (*joints_)[i]->SetVelocity(0, 0.0);
      }
    }

    (void)snapshot;
    brakes_locked_ = locked;
  }

  [[nodiscard]] bool brakesLocked() const override { return brakes_locked_; }

  [[nodiscard]] bool supportsTrajectoryExecution() const override {
    return !shutting_down_.load() && trajectory_client_ != nullptr && trajectory_client_->action_server_is_ready();
  }

  bool startTrajectoryExecution(const runtime::TrajectoryExecutionGoal &goal,
                                std::string &message) override {
    if (shutting_down_.load()) {
      message = "backend is shutting down";
      return false;
    }
    if (trajectory_client_ == nullptr) {
      message = "joint_trajectory_controller client is unavailable";
      return false;
    }
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(3))) {
      message = "joint_trajectory_controller action server is unavailable";
      return false;
    }
    if (goal.points.empty()) {
      message = "trajectory goal is empty";
      return false;
    }

    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory.joint_names = trajectory_joint_names_;
    goal_msg.trajectory.points.reserve(goal.points.size());
    for (const auto &point : goal.points) {
      trajectory_msgs::msg::JointTrajectoryPoint ros_point;
      ros_point.positions = point.position;
      ros_point.velocities = point.velocity;
      ros_point.accelerations = point.acceleration;
      const auto sec = static_cast<int32_t>(std::floor(point.time_from_start));
      const auto fractional = point.time_from_start - static_cast<double>(sec);
      ros_point.time_from_start.sec = sec;
      ros_point.time_from_start.nanosec =
          static_cast<uint32_t>(std::clamp(fractional * 1e9, 0.0, 999999999.0));
      goal_msg.trajectory.points.push_back(std::move(ros_point));
    }

    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      generation = ++trajectory_generation_;
      trajectory_state_ = runtime::TrajectoryExecutionState{};
      trajectory_state_.request_id = goal.request_id;
      trajectory_state_.message = "sending trajectory";
      trajectory_segment_end_times_ = goal.segment_end_times;
      trajectory_goal_duration_sec_ =
          goal.points.empty() ? 0.0 : std::max(goal.points.back().time_from_start, 0.0);
      trajectory_goal_dispatch_time_ = std::chrono::steady_clock::now();
      trajectory_last_feedback_time_ = std::chrono::steady_clock::now();
      goal_handle_.reset();
    }

    typename rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback =
        [this, request_id = goal.request_id, generation](std::shared_ptr<FollowJointTrajectoryGoalHandle> goal_handle) {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          if (generation != trajectory_generation_) {
            return;
          }
          trajectory_state_.request_id = request_id;
          trajectory_state_.accepted = (goal_handle != nullptr);
          trajectory_state_.active = (goal_handle != nullptr);
          trajectory_state_.completed = (goal_handle == nullptr);
          trajectory_state_.failed = (goal_handle == nullptr);
          trajectory_state_.message = goal_handle != nullptr ? "trajectory accepted" : "trajectory goal rejected";
          trajectory_goal_dispatch_time_ = std::chrono::steady_clock::now();
          goal_handle_ = goal_handle;
        };
    options.feedback_callback =
        [this, request_id = goal.request_id, generation](FollowJointTrajectoryGoalHandle::SharedPtr,
                                                         const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          if (generation != trajectory_generation_) {
            return;
          }
          trajectory_state_.request_id = request_id;
          trajectory_state_.accepted = true;
          trajectory_state_.active = true;
          trajectory_state_.desired_time_from_start = durationToSec(feedback->desired.time_from_start);
          trajectory_state_.actual_position = feedback->actual.positions;
          trajectory_state_.actual_velocity = feedback->actual.velocities;
          trajectory_state_.actual_acceleration = feedback->actual.accelerations;
          trajectory_state_.message = "executing";
          trajectory_last_feedback_time_ = std::chrono::steady_clock::now();
        };
    options.result_callback =
        [this, request_id = goal.request_id, generation](const FollowJointTrajectoryGoalHandle::WrappedResult &result) {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          if (generation != trajectory_generation_) {
            return;
          }
          trajectory_state_.request_id = request_id;
          trajectory_state_.active = false;
          trajectory_state_.completed = true;
          trajectory_state_.succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED;
          trajectory_state_.canceled = result.code == rclcpp_action::ResultCode::CANCELED;
          trajectory_state_.failed = result.code == rclcpp_action::ResultCode::ABORTED ||
                                     result.code == rclcpp_action::ResultCode::UNKNOWN;
          if (result.result != nullptr && !result.result->error_string.empty()) {
            trajectory_state_.message = result.result->error_string;
          } else if (trajectory_state_.succeeded) {
            trajectory_state_.message = "trajectory completed";
          } else if (trajectory_state_.canceled) {
            trajectory_state_.message = "trajectory canceled";
          } else {
            trajectory_state_.message = "trajectory aborted";
          }
          trajectory_last_feedback_time_ = std::chrono::steady_clock::now();
        };

    const auto goal_future = trajectory_client_->async_send_goal(goal_msg, options);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (generation == trajectory_generation_) {
        trajectory_state_.active = false;
        trajectory_state_.completed = true;
        trajectory_state_.failed = true;
        trajectory_state_.message = "timed out waiting for trajectory goal response";
      }
      message = "timed out waiting for trajectory goal response";
      return false;
    }

    const auto goal_handle = goal_future.get();
    if (goal_handle == nullptr) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (generation == trajectory_generation_) {
        trajectory_state_.active = false;
        trajectory_state_.completed = true;
        trajectory_state_.failed = true;
        trajectory_state_.message = "trajectory goal rejected";
      }
      message = "trajectory goal rejected";
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (generation == trajectory_generation_) {
        goal_handle_ = goal_handle;
        trajectory_state_.request_id = goal.request_id;
        trajectory_state_.accepted = true;
        trajectory_state_.active = true;
        trajectory_state_.completed = false;
        trajectory_state_.failed = false;
        trajectory_state_.canceled = false;
        trajectory_state_.succeeded = false;
        trajectory_state_.message = "trajectory accepted";
      }
    }
    message.clear();
    return true;
  }

  void cancelTrajectoryExecution(const std::string &reason) override {
    std::shared_ptr<FollowJointTrajectoryGoalHandle> goal_handle;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      ++trajectory_generation_;
      trajectory_state_.active = false;
      trajectory_state_.completed = true;
      trajectory_state_.canceled = true;
      trajectory_state_.message = reason.empty() ? "trajectory canceled" : reason;
      goal_handle = goal_handle_;
      trajectory_client = trajectory_client_;
      goal_handle_.reset();
    }
    if (!shutting_down_.load() && trajectory_client != nullptr && goal_handle != nullptr) {
      try {
        (void)trajectory_client->async_cancel_goal(goal_handle);
      } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &exc) {
        if (node_ != nullptr) {
          RCLCPP_DEBUG(node_->get_logger(),
                       "Ignoring stale trajectory goal during cancel: %s",
                       exc.what());
        }
      } catch (const std::exception &exc) {
        if (node_ != nullptr) {
          RCLCPP_WARN(node_->get_logger(),
                      "Failed to cancel trajectory goal cleanly: %s",
                      exc.what());
        }
      }
    }
  }

  [[nodiscard]] runtime::TrajectoryExecutionState readTrajectoryExecutionState() const override {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (trajectory_state_.active &&
        !trajectory_state_.completed &&
        trajectory_goal_duration_sec_ > 0.0) {
      const bool reached_goal_horizon =
          trajectory_state_.desired_time_from_start + 1e-3 >= trajectory_goal_duration_sec_;
      const auto wall_elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - trajectory_goal_dispatch_time_).count();
      const bool reached_wall_clock_horizon =
          wall_elapsed >= trajectory_goal_duration_sec_ + kTrajectoryCompletionWallClockGraceSec;
      const bool nearly_still =
          !trajectory_state_.actual_velocity.empty() &&
          maxAbsVector(trajectory_state_.actual_velocity) <=
              kTrajectoryCompletionInferenceVelocityRadPerSec;
      const auto feedback_age =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - trajectory_last_feedback_time_).count();
      const bool stale_feedback = feedback_age >= kTrajectoryCompletionInferenceGraceSec;
      if ((reached_goal_horizon && (nearly_still || stale_feedback)) || reached_wall_clock_horizon) {
        trajectory_state_.active = false;
        trajectory_state_.completed = true;
        trajectory_state_.succeeded = true;
        trajectory_state_.canceled = false;
        trajectory_state_.failed = false;
        if (trajectory_state_.message.empty() || trajectory_state_.message == "executing") {
          trajectory_state_.message = "trajectory completed (inferred)";
        }
      }
    }
    return trajectory_state_;
  }

 private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  static double durationToSec(const builtin_interfaces::msg::Duration &duration) {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  }

  std::vector<physics::JointPtr> *joints_;
  const std::array<std::pair<double, double>, 6> *original_joint_limits_;
  bool brakes_locked_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  std::vector<std::string> trajectory_joint_names_;
  mutable std::mutex trajectory_mutex_;
  std::shared_ptr<FollowJointTrajectoryGoalHandle> goal_handle_;
  mutable runtime::TrajectoryExecutionState trajectory_state_;
  mutable std::vector<double> trajectory_segment_end_times_;
  mutable std::chrono::steady_clock::time_point trajectory_goal_dispatch_time_{
      std::chrono::steady_clock::now()};
  mutable std::chrono::steady_clock::time_point trajectory_last_feedback_time_{
      std::chrono::steady_clock::now()};
  mutable double trajectory_goal_duration_sec_ = 0.0;
  std::uint64_t trajectory_generation_ = 0;
  std::atomic<runtime::ControlOwner> control_owner_{runtime::ControlOwner::none};
  std::atomic<bool> shutting_down_{false};
};

}  // namespace gazebo

#endif
