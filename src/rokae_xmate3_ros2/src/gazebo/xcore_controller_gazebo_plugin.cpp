/**
 * @file xcore_controller_gazebo_plugin.cpp
 * @brief xCore SDK Gazebo 仿真控制器插件
 *
 * 这个插件直接控制Gazebo关节来实现xCore SDK功能，
 * 同时配合ros2_control的joint_state_broadcaster提供标准ROS2接口。
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <array>
#include <atomic>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <chrono>

#include <builtin_interfaces/msg/time.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"
#include "rokae_xmate3_ros2/srv/prepare_shutdown.hpp"
#include "runtime/runtime_control_bridge.hpp"
#include "runtime/runtime_publish_bridge.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/ros_bindings.hpp"
#include "rokae_xmate3_ros2/runtime/shutdown_coordinator.hpp"

// ROS2 接口头文件
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/runtime_diagnostics.hpp"



namespace gazebo {

namespace {

constexpr double kDefaultTrajectorySampleDt = 0.01;
constexpr double kMinTrajectorySampleDt = 0.001;
constexpr double kMaxTrajectorySampleDt = 0.05;
constexpr double kTrajectoryCompletionInferenceGraceSec = 0.35;
constexpr double kTrajectoryCompletionWallClockGraceSec = 0.5;
constexpr double kTrajectoryCompletionInferenceVelocityRadPerSec = 0.02;

enum class BackendMode {
  effort,
  jtc,
  hybrid,
};

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

namespace runtime = rokae_xmate3_ros2::runtime;

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

class XCoreControllerPlugin : public ModelPlugin {
public:
    XCoreControllerPlugin() : ModelPlugin() {}
    ~XCoreControllerPlugin() override;

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
    void InitROS2();
    void InitPublishers();
    void OnUpdate();
    void ExecuteMotion();
    void prepareForShutdown(const std::string &reason);
    [[nodiscard]] runtime::ShutdownContractView collectShutdownContractState(bool request_prepare);
    [[nodiscard]] runtime::MotionRuntime *motionRuntime();
    [[nodiscard]] const runtime::MotionRuntime *motionRuntime() const;
    void RefreshJointStateCache();
    void GetCachedJointState(std::array<double, 6> &position,
                             std::array<double, 6> &velocity,
                             std::array<double, 6> &torque);
    physics::ModelPtr model_;
    sdf::ElementPtr sdf_;
    std::vector<physics::JointPtr> joints_;
    std::vector<std::string> joint_names_;
    int joint_num_ = 0;
    event::ConnectionPtr update_conn_;

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::unique_ptr<xMate3Kinematics> kinematics_;

    std::unique_ptr<runtime::RuntimeContext> runtime_context_;
    std::unique_ptr<runtime::RosBindings> ros_bindings_;
    std::unique_ptr<runtime::RuntimeControlBridge> control_bridge_;
    std::unique_ptr<runtime::RuntimePublishBridge> publish_bridge_;
    bool joint_positions_initialized_ = false;

    bool initial_pose_set_ = false;
    std::array<std::pair<double, double>, 6> original_joint_limits_;
    const std::vector<double> default_initial_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    double current_update_dt_ = kDefaultTrajectorySampleDt;
    std::mutex joint_state_cache_mutex_;
    std::mutex update_cycle_mutex_;
    std::array<double, 6> cached_joint_position_{};
    std::array<double, 6> cached_joint_velocity_{};
    std::array<double, 6> cached_joint_torque_{};
    std::unique_ptr<GazeboRuntimeBackend> motion_backend_;
    std::atomic<uint64_t> next_request_id_{1};
    double trajectory_sample_dt_ = kDefaultTrajectorySampleDt;
    BackendMode backend_mode_ = BackendMode::hybrid;
    std::atomic<bool> shutting_down_{false};
    std::mutex shutdown_prepare_mutex_;
    bool shutdown_prepared_ = false;
    runtime::ShutdownCoordinator shutdown_coordinator_;
    rclcpp::Service<rokae_xmate3_ros2::srv::PrepareShutdown>::SharedPtr prepare_shutdown_service_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_pub_;
    rclcpp::Publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>::SharedPtr runtime_diagnostics_pub_;
};

void XCoreControllerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    gzmsg << "[xCore Controller] Loading plugin for model: " << _model->GetName() << std::endl;

    model_ = _model;
    sdf_ = _sdf;
    std::string backend_mode_value = "hybrid";
    if (sdf_ && sdf_->HasElement("backend_mode")) {
        backend_mode_value = sdf_->Get<std::string>("backend_mode");
    }
    backend_mode_ = parseBackendMode(backend_mode_value);
    gzmsg << "[xCore Controller] backend_mode=" << toString(backend_mode_) << std::endl;

    // 获取关节 - 直接用于控制
    joint_names_ = {"xmate_joint_1", "xmate_joint_2", "xmate_joint_3",
                    "xmate_joint_4", "xmate_joint_5", "xmate_joint_6"};
    for (const auto& name : joint_names_) {
        auto joint = _model->GetJoint(name);
        if (joint) {
            joints_.push_back(joint);
        } else {
            gzerr << "[xCore] Joint not found: " << name << std::endl;
        }
    }
    joint_num_ = joints_.size();

    // 保存URDF原始关节限位
    const std::array<std::pair<double, double>, 6> default_limits = {
        std::make_pair(-3.0527, 3.0527), // joint1
        std::make_pair(-2.0933, 2.0933), // joint2
        std::make_pair(-2.0933, 2.0933), // joint3
        std::make_pair(-3.0527, 3.0527), // joint4
        std::make_pair(-2.0933, 2.0933), // joint5
        std::make_pair(-6.1082, 6.1082)  // joint6
    };
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
        original_joint_limits_[i] = default_limits[i];
    }

    runtime_context_ = std::make_unique<runtime::RuntimeContext>();

    // ========== 删除原来这里的初始位置设置代码 ==========
    joint_positions_initialized_ = false;

    InitROS2();

    if (backend_mode_ == BackendMode::jtc) {
        gzerr << "[xCore Controller] plugin loaded while backend_mode=jtc; "
              << "effort commands will remain disabled and JTC ownership is expected to stay external."
              << std::endl;
    }

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&XCoreControllerPlugin::OnUpdate, this));

    gzmsg << "[xCore Controller] Plugin loaded successfully, joints: " << joint_num_ << std::endl;
}

XCoreControllerPlugin::~XCoreControllerPlugin() {
    prepareForShutdown("plugin shutdown");

    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    if (executor_ && node_) {
        try {
            auto node_base = node_->get_node_base_interface();
            if (node_base &&
                node_base->get_associated_with_executor_atomic().load()) {
                executor_->remove_node(node_);
            }
        } catch (const std::exception &e) {
            gzdbg << "[xCore Controller] executor remove_node skipped during shutdown: "
                  << e.what() << std::endl;
        }
    }
    ros_bindings_.reset();
    publish_bridge_.reset();
    control_bridge_.reset();
    motion_backend_.reset();
    runtime_context_.reset();
    kinematics_.reset();
    executor_.reset();
    node_.reset();
}

void XCoreControllerPlugin::prepareForShutdown(const std::string &reason) {
    std::lock_guard<std::mutex> shutdown_lock(shutdown_prepare_mutex_);
    if (shutdown_prepared_) {
        return;
    }

    shutting_down_.store(true);
    update_conn_.reset();

    {
        std::unique_lock<std::mutex> update_lock(update_cycle_mutex_);
        if (motion_backend_) {
            motion_backend_->clearControl();
            motion_backend_->cancelTrajectoryExecution(reason);
            motion_backend_->beginShutdown();
        }
    }

    if (runtime_context_) {
        runtime_context_->motionRuntime().stop(reason);
        runtime_context_->motionRuntime().reset();
    }

    shutdown_prepared_ = true;
}

runtime::ShutdownContractView
XCoreControllerPlugin::collectShutdownContractState(bool request_prepare) {
    if (request_prepare) {
        shutdown_coordinator_.beginShutdown("internal prepare shutdown");
        prepareForShutdown("internal prepare shutdown");
    }

    runtime::ShutdownContractView state;
    if (!runtime_context_) {
        state.contract_version = runtime::kRuntimeContractVersion;
        state.code = runtime::RuntimeContractCode::faulted;
        state.owner = runtime::ControlOwner::none;
        state.runtime_phase = runtime::RuntimePhase::faulted;
        state.shutdown_phase = runtime::ShutdownPhase::faulted;
        state.backend_quiescent = false;
        state.message = "runtime context unavailable";
        state.safe_to_delete = false;
        state.safe_to_stop_world = false;
        return state;
    }

    auto &motion_runtime = runtime_context_->motionRuntime();
    auto facts = motion_runtime.contractFacts();

    const auto backend_owner =
        motion_backend_ ? motion_backend_->controlOwner() : runtime::ControlOwner::none;

    const auto trajectory_state =
        motion_backend_ ? motion_backend_->readTrajectoryExecutionState() : runtime::TrajectoryExecutionState{};
    facts.backend_quiescent =
        facts.backend_quiescent &&
        backend_owner == runtime::ControlOwner::none &&
        (!trajectory_state.active || trajectory_state.completed || trajectory_state.failed ||
         trajectory_state.canceled || trajectory_state.succeeded);
    facts.plugin_detached = shutting_down_.load() && !static_cast<bool>(update_conn_);
    facts.faulted = facts.faulted || facts.runtime_phase == runtime::RuntimePhase::faulted;
    if (facts.message.empty()) {
        facts.message = shutdown_prepared_ ? "shutdown prepared" : "shutdown requested";
    }

    shutdown_coordinator_.updateFacts(facts);
    state = shutdown_coordinator_.currentView();
    runtime_context_->diagnosticsState().updateShutdownContract(state);
    return state;
}

void XCoreControllerPlugin::InitROS2() {
    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("xcore_gazebo_controller");
    trajectory_sample_dt_ = std::clamp(
        node_->declare_parameter("trajectory_sample_dt", kDefaultTrajectorySampleDt),
        kMinTrajectorySampleDt,
        kMaxTrajectorySampleDt);

    auto planner_config = gazebo::TrajectoryPlanner::config();
    planner_config.max_joint_step_rad = std::clamp(
        node_->declare_parameter("planner.max_joint_step_rad", planner_config.max_joint_step_rad),
        1e-3,
        0.2);
    planner_config.max_cartesian_step_m = std::clamp(
        node_->declare_parameter("planner.max_cartesian_step_m", planner_config.max_cartesian_step_m),
        1e-4,
        0.05);
    planner_config.max_orientation_step_rad = std::clamp(
        node_->declare_parameter("planner.max_orientation_step_rad", planner_config.max_orientation_step_rad),
        1e-3,
        0.5);
    planner_config.joint_speed_limits_rad_per_sec = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "planner.joint_speed_limits",
            std::vector<double>(planner_config.joint_speed_limits_rad_per_sec.begin(),
                                planner_config.joint_speed_limits_rad_per_sec.end())),
        planner_config.joint_speed_limits_rad_per_sec,
        1e-6);
    planner_config.joint_acc_limits_rad_per_sec2 = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "planner.joint_acc_limits",
            std::vector<double>(planner_config.joint_acc_limits_rad_per_sec2.begin(),
                                planner_config.joint_acc_limits_rad_per_sec2.end())),
        planner_config.joint_acc_limits_rad_per_sec2,
        1e-6);
    gazebo::TrajectoryPlanner::setConfig(planner_config);

    runtime::MotionExecutorConfig executor_config;
    executor_config.active_track_kp = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.active_track_kp",
            std::vector<double>(executor_config.active_track_kp.begin(), executor_config.active_track_kp.end())),
        executor_config.active_track_kp,
        0.0);
    executor_config.active_track_kd = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.active_track_kd",
            std::vector<double>(executor_config.active_track_kd.begin(), executor_config.active_track_kd.end())),
        executor_config.active_track_kd,
        0.0);
    executor_config.active_track_ki = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.active_track_ki",
            std::vector<double>(executor_config.active_track_ki.begin(), executor_config.active_track_ki.end())),
        executor_config.active_track_ki,
        0.0);
    executor_config.hold_track_kp = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.hold_track_kp",
            std::vector<double>(executor_config.hold_track_kp.begin(), executor_config.hold_track_kp.end())),
        executor_config.hold_track_kp,
        0.0);
    executor_config.hold_track_kd = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.hold_track_kd",
            std::vector<double>(executor_config.hold_track_kd.begin(), executor_config.hold_track_kd.end())),
        executor_config.hold_track_kd,
        0.0);
    executor_config.hold_track_ki = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.hold_track_ki",
            std::vector<double>(executor_config.hold_track_ki.begin(), executor_config.hold_track_ki.end())),
        executor_config.hold_track_ki,
        0.0);
    executor_config.strong_hold_track_kp = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.strong_hold_track_kp",
            std::vector<double>(executor_config.strong_hold_track_kp.begin(),
                                executor_config.strong_hold_track_kp.end())),
        executor_config.strong_hold_track_kp,
        0.0);
    executor_config.strong_hold_track_kd = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.strong_hold_track_kd",
            std::vector<double>(executor_config.strong_hold_track_kd.begin(),
                                executor_config.strong_hold_track_kd.end())),
        executor_config.strong_hold_track_kd,
        0.0);
    executor_config.strong_hold_track_ki = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.strong_hold_track_ki",
            std::vector<double>(executor_config.strong_hold_track_ki.begin(),
                                executor_config.strong_hold_track_ki.end())),
        executor_config.strong_hold_track_ki,
        0.0);
    executor_config.static_friction = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.static_friction",
            std::vector<double>(executor_config.static_friction.begin(), executor_config.static_friction.end())),
        executor_config.static_friction,
        0.0);
    executor_config.gravity_compensation = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.gravity_compensation",
            std::vector<double>(executor_config.gravity_compensation.begin(),
                                executor_config.gravity_compensation.end())),
        executor_config.gravity_compensation,
        0.0);
    executor_config.effort_limit = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.effort_limit",
            std::vector<double>(executor_config.effort_limit.begin(), executor_config.effort_limit.end())),
        executor_config.effort_limit,
        1e-6);
    executor_config.torque_rate_limit = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "executor.torque_rate_limit",
            std::vector<double>(executor_config.torque_rate_limit.begin(),
                                executor_config.torque_rate_limit.end())),
        executor_config.torque_rate_limit,
        1e-6);
    executor_config.tracking_position_tolerance_rad = std::max(
        node_->declare_parameter("executor.tracking_position_tolerance_rad",
                                 executor_config.tracking_position_tolerance_rad),
        1e-6);
    executor_config.tracking_velocity_tolerance_rad = std::max(
        node_->declare_parameter("executor.tracking_velocity_tolerance_rad",
                                 executor_config.tracking_velocity_tolerance_rad),
        1e-6);
    executor_config.final_position_tolerance_rad = std::max(
        node_->declare_parameter("executor.final_position_tolerance_rad",
                                 executor_config.final_position_tolerance_rad),
        1e-6);
    executor_config.final_velocity_tolerance_rad = std::max(
        node_->declare_parameter("executor.final_velocity_tolerance_rad",
                                 executor_config.final_velocity_tolerance_rad),
        1e-6);
    executor_config.soft_final_position_tolerance_rad = std::max(
        node_->declare_parameter("executor.soft_final_position_tolerance_rad",
                                 executor_config.soft_final_position_tolerance_rad),
        1e-6);
    executor_config.soft_final_velocity_tolerance_rad = std::max(
        node_->declare_parameter("executor.soft_final_velocity_tolerance_rad",
                                 executor_config.soft_final_velocity_tolerance_rad),
        1e-6);

    const int zone_valid_min_mm = node_->declare_parameter("zone_valid_min_mm", 0);
    const int zone_valid_max_mm = node_->declare_parameter("zone_valid_max_mm", 200);
    runtime_context_->motionOptionsState().setZoneValidRange(zone_valid_min_mm, zone_valid_max_mm);

    runtime::RuntimeControlBridgeConfig control_bridge_config;
    control_bridge_config.collision_nominal_thresholds = vectorToArray(
        node_->declare_parameter<std::vector<double>>(
            "collision_nominal_thresholds",
            std::vector<double>(control_bridge_config.collision_nominal_thresholds.begin(),
                                control_bridge_config.collision_nominal_thresholds.end())),
        control_bridge_config.collision_nominal_thresholds,
        1e-6);
    control_bridge_config.collision_slow_scale = std::clamp(
        node_->declare_parameter("collision_slow_scale", control_bridge_config.collision_slow_scale),
        0.05,
        1.0);
    control_bridge_config.collision_retreat_distance =
        std::max(node_->declare_parameter("collision_retreat_distance",
                                          control_bridge_config.collision_retreat_distance),
                 0.0);

    kinematics_ = std::make_unique<xMate3Kinematics>();
    motion_backend_ = std::make_unique<GazeboRuntimeBackend>(&joints_, &original_joint_limits_);
    if (backend_mode_ != BackendMode::effort) {
        motion_backend_->configureTrajectoryClient(node_, joint_names_);
    }
    runtime_context_->attachBackend(motion_backend_.get());
    runtime_context_->motionRuntime().setExecutorConfig(executor_config);
    runtime_context_->diagnosticsState().configure(
        toString(backend_mode_),
        diagnosticCapabilityFlags(backend_mode_));
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    auto joint_state_fetcher = [this](std::array<double, 6> &position,
                                      std::array<double, 6> &velocity,
                                      std::array<double, 6> &torque) {
        GetCachedJointState(position, velocity, torque);
    };
    auto time_provider = [this]() { return node_->get_clock()->now(); };
    auto trajectory_dt_provider = [this]() { return trajectory_sample_dt_; };
    auto request_id_generator = [this](const std::string &prefix) {
        return prefix + std::to_string(next_request_id_.fetch_add(1));
    };

    InitPublishers();
    publish_bridge_ = std::make_unique<runtime::RuntimePublishBridge>(*runtime_context_);
    ros_bindings_ = std::make_unique<runtime::RosBindings>(
        node_,
        *runtime_context_,
        publish_bridge_.get(),
        *kinematics_,
        joint_state_fetcher,
        time_provider,
        trajectory_dt_provider,
        request_id_generator);
    control_bridge_ = std::make_unique<runtime::RuntimeControlBridge>(*runtime_context_, control_bridge_config);

    prepare_shutdown_service_ = node_->create_service<rokae_xmate3_ros2::srv::PrepareShutdown>(
        "/xmate3/internal/prepare_shutdown",
        [this](const std::shared_ptr<rokae_xmate3_ros2::srv::PrepareShutdown::Request> request,
               std::shared_ptr<rokae_xmate3_ros2::srv::PrepareShutdown::Response> response) {
          const auto state = collectShutdownContractState(request->request_shutdown);
          response->accepted = runtime_context_ != nullptr &&
                               (request->request_shutdown || shutdown_coordinator_.requested());
          response->contract_version = state.contract_version;
          response->code = runtime::to_u16(state.code);
          response->owner = runtime::to_u8(state.owner);
          response->runtime_phase = runtime::to_u8(state.runtime_phase);
          response->shutdown_phase = runtime::to_u8(state.shutdown_phase);
          response->active_request_count =
              static_cast<decltype(response->active_request_count)>(state.active_request_count);
          response->active_goal_count =
              static_cast<decltype(response->active_goal_count)>(state.active_goal_count);
          response->backend_quiescent = state.backend_quiescent;
          response->safe_to_delete = state.safe_to_delete;
          response->safe_to_stop_world = state.safe_to_stop_world;
          response->message = state.message;
          RCLCPP_INFO(
              node_->get_logger(),
              "prepare_shutdown contract accepted=%s version=%u code=%s owner=%s runtime_phase=%s shutdown_phase=%s "
              "active_request_count=%zu active_goal_count=%zu backend_quiescent=%s "
              "safe_to_delete=%s safe_to_stop_world=%s message=%s",
              response->accepted ? "true" : "false",
              response->contract_version,
              runtime::to_string(state.code),
              runtime::to_string(state.owner),
              runtime::to_string(state.runtime_phase),
              runtime::to_string(state.shutdown_phase),
              state.active_request_count,
              state.active_goal_count,
              state.backend_quiescent ? "true" : "false",
              state.safe_to_delete ? "true" : "false",
              state.safe_to_stop_world ? "true" : "false",
              response->message.c_str());
        });

    executor_thread_ = std::thread([this]() {
        try {
            executor_->spin();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "ROS executor stopped unexpectedly: %s", e.what());
        }
    });

    RCLCPP_INFO(
        node_->get_logger(),
        "runtime diagnostics ready: backend=%s rt_level=experimental aliases=[get_joint_torque,get_end_torque] "
        "services=[/xmate3/internal/validate_motion,/xmate3/internal/get_runtime_diagnostics] "
        "topic=[/xmate3/internal/runtime_status]",
        toString(backend_mode_));
}

void XCoreControllerPlugin::InitPublishers() {
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/xmate3/joint_states", 10);
    operation_state_pub_ = node_->create_publisher<rokae_xmate3_ros2::msg::OperationState>("/xmate3/cobot/operation_state", 10);
    runtime_diagnostics_pub_ =
        node_->create_publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>("/xmate3/internal/runtime_status", 10);
}

void XCoreControllerPlugin::RefreshJointStateCache() {
    std::array<double, 6> position{};
    std::array<double, 6> velocity{};
    std::array<double, 6> torque{};
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
        position[i] = joints_[i]->Position(0);
        velocity[i] = joints_[i]->GetVelocity(0);
        torque[i] = joints_[i]->GetForce(0);
    }
    std::lock_guard<std::mutex> lock(joint_state_cache_mutex_);
    cached_joint_position_ = position;
    cached_joint_velocity_ = velocity;
    cached_joint_torque_ = torque;
}

void XCoreControllerPlugin::GetCachedJointState(std::array<double, 6> &position,
                                                std::array<double, 6> &velocity,
                                                std::array<double, 6> &torque) {
    std::lock_guard<std::mutex> lock(joint_state_cache_mutex_);
    position = cached_joint_position_;
    velocity = cached_joint_velocity_;
    torque = cached_joint_torque_;
}



runtime::MotionRuntime *XCoreControllerPlugin::motionRuntime() {
    return runtime_context_ ? &runtime_context_->motionRuntime() : nullptr;
}

const runtime::MotionRuntime *XCoreControllerPlugin::motionRuntime() const {
    return runtime_context_ ? &runtime_context_->motionRuntime() : nullptr;
}

void XCoreControllerPlugin::OnUpdate() {
    std::unique_lock<std::mutex> update_lock(update_cycle_mutex_);
    if (shutting_down_.load() || !model_ || !node_ || !motion_backend_ || !control_bridge_) {
        return;
    }

    // Keep the runtime tick on a deterministic sample period instead of
    // querying Gazebo's world during teardown. Model::GetWorld() is not safe
    // once the model/world are being destroyed, and the runtime already
    // exposes a configured sampling period for planning/execution.
    current_update_dt_ = trajectory_sample_dt_;

    // ========== 【最高优先级】强制设置初始姿态（仅执行一次） ==========
    if (!initial_pose_set_ && joint_num_ > 0) {
        gzmsg << "[xCore Controller] Setting initial joint pose..." << std::endl;
        for (int i = 0; i < joint_num_ && i < static_cast<int>(default_initial_pose_.size()); ++i) {
            // 第三个参数true：跳过物理约束，强制设置关节位置
            joints_[i]->SetPosition(0, default_initial_pose_[i], true);
            // 清空关节速度和力，避免残留值导致姿态跳变
            joints_[i]->SetVelocity(0, 0.0);
            joints_[i]->SetForce(0, 0.0);
        }
        // 标记初始化完成
        initial_pose_set_ = true;
        joint_positions_initialized_ = true;
        gzmsg << "[xCore Controller] Initial pose set successfully!" << std::endl;
    }

    // 运动控制/抱闸逻辑（必须在初始姿态设置完成后执行）
    if (initial_pose_set_) {
        ExecuteMotion();
    }

    RefreshJointStateCache();

    auto now = node_->now();
    if (publish_bridge_ != nullptr && joint_state_pub_ != nullptr && operation_state_pub_ != nullptr) {
        collectShutdownContractState(false);
        std::array<double, 6> cached_pos{};
        std::array<double, 6> cached_vel{};
        std::array<double, 6> cached_torque{};
        GetCachedJointState(cached_pos, cached_vel, cached_torque);

        runtime::PublisherTickInput tick_input;
        tick_input.stamp = now;
        tick_input.frame_id = "base_link";
        tick_input.joint_names = &joint_names_;
        tick_input.position = cached_pos;
        tick_input.velocity = cached_vel;
        tick_input.torque = cached_torque;
        tick_input.min_publish_period_sec = 0.001;

        const auto publish_tick = publish_bridge_->buildPublisherTick(tick_input);
        if (publish_tick.publish_joint_state) {
            joint_state_pub_->publish(publish_tick.joint_state);
        }
        if (publish_tick.publish_operation_state) {
            operation_state_pub_->publish(publish_tick.operation_state);
        }
        if (publish_tick.publish_operation_state && runtime_diagnostics_pub_ != nullptr) {
            runtime_diagnostics_pub_->publish(publish_bridge_->buildRuntimeDiagnosticsMessage());
        }
    }

}

void XCoreControllerPlugin::ExecuteMotion() {
    if (!motion_backend_ || !control_bridge_) {
        return;
    }

    const auto snapshot = motion_backend_->readSnapshot();
    const auto tick_result = control_bridge_->tick(*motion_backend_, snapshot, current_update_dt_);
    if (publish_bridge_ != nullptr && node_ != nullptr) {
        publish_bridge_->emitRuntimeStatus(tick_result.status, node_->now(), node_->get_logger());
    }
}

GZ_REGISTER_MODEL_PLUGIN(XCoreControllerPlugin)

} // namespace gazebo
