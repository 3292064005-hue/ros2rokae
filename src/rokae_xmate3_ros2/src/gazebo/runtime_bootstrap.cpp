#include "gazebo/runtime_bootstrap.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "rokae_xmate3_ros2/gazebo/trajectory_planner.hpp"
#include "rokae_xmate3_ros2/types.hpp"
#include "runtime/rt_scheduler.hpp"
#include "runtime/runtime_host_builder.hpp"

namespace gazebo {

namespace runtime = rokae_xmate3_ros2::runtime;

namespace {

rclcpp::Node::SharedPtr makeBootstrapNode(const RuntimeBootstrap::RosIntegrationOptions& options) {
  if (options.node) {
    return options.node;
  }
  rclcpp::NodeOptions node_options;
  if (options.context) {
    node_options.context(options.context);
  }
  for (const auto &parameter : options.parameter_overrides) {
    node_options.append_parameter_override(parameter.get_name(), parameter.get_parameter_value());
  }
  return std::make_shared<rclcpp::Node>("xcore_gazebo_controller", node_options);
}

std::string getenvOrDefault(const char *name, const std::string &fallback) {
  if (name == nullptr) {
    return fallback;
  }
  const char *value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return std::string(value);
}

}  // namespace

RuntimeBootstrap::RuntimeBootstrap(BackendMode backend_mode,
                                   std::vector<physics::JointPtr> *joints,
                                   const std::array<std::pair<double, double>, 6> *original_joint_limits,
                                   const std::vector<std::string> *joint_names,
                                   JointStateFetcher joint_state_fetcher)
    : RuntimeBootstrap(backend_mode,
                       joints,
                       original_joint_limits,
                       joint_names,
                       std::move(joint_state_fetcher),
                       RosIntegrationOptions{}) {}

RuntimeBootstrap::RuntimeBootstrap(BackendMode backend_mode,
                                   std::vector<physics::JointPtr> *joints,
                                   const std::array<std::pair<double, double>, 6> *original_joint_limits,
                                   const std::vector<std::string> *joint_names,
                                   JointStateFetcher joint_state_fetcher,
                                   RosIntegrationOptions ros_integration)
    : backend_mode_(backend_mode),
      joints_(joints),
      original_joint_limits_(original_joint_limits),
      joint_names_(joint_names),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      ros_integration_(std::move(ros_integration)),
      runtime_context_(std::make_unique<runtime::RuntimeContext>()) {}

RuntimeBootstrap::~RuntimeBootstrap() {
  shutdown("bootstrap shutdown");
  releaseExecutorNode();
}

void RuntimeBootstrap::start() {
  if (!ros_integration_.node && !ros_integration_.context) {
    ros_context_lease_ = rokae_xmate3_ros2::runtime::RosContextOwner::acquire("gazebo_runtime_bootstrap");
  }

  node_ = makeBootstrapNode(ros_integration_);
  host_builder_ = std::make_unique<runtime::RuntimeHostBuilder>(node_);
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
          std::vector<double>(executor_config.strong_hold_track_kp.begin(), executor_config.strong_hold_track_kp.end())),
      executor_config.strong_hold_track_kp,
      0.0);
  executor_config.strong_hold_track_kd = vectorToArray(
      node_->declare_parameter<std::vector<double>>(
          "executor.strong_hold_track_kd",
          std::vector<double>(executor_config.strong_hold_track_kd.begin(), executor_config.strong_hold_track_kd.end())),
      executor_config.strong_hold_track_kd,
      0.0);
  executor_config.strong_hold_track_ki = vectorToArray(
      node_->declare_parameter<std::vector<double>>(
          "executor.strong_hold_track_ki",
          std::vector<double>(executor_config.strong_hold_track_ki.begin(), executor_config.strong_hold_track_ki.end())),
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
          std::vector<double>(executor_config.gravity_compensation.begin(), executor_config.gravity_compensation.end())),
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
          std::vector<double>(executor_config.torque_rate_limit.begin(), executor_config.torque_rate_limit.end())),
      executor_config.torque_rate_limit,
      1e-6);
  executor_config.tracking_position_tolerance_rad = std::max(
      node_->declare_parameter("executor.tracking_position_tolerance_rad", executor_config.tracking_position_tolerance_rad),
      1e-6);
  executor_config.tracking_velocity_tolerance_rad = std::max(
      node_->declare_parameter("executor.tracking_velocity_tolerance_rad", executor_config.tracking_velocity_tolerance_rad),
      1e-6);
  executor_config.final_position_tolerance_rad = std::max(
      node_->declare_parameter("executor.final_position_tolerance_rad", executor_config.final_position_tolerance_rad),
      1e-6);
  executor_config.final_velocity_tolerance_rad = std::max(
      node_->declare_parameter("executor.final_velocity_tolerance_rad", executor_config.final_velocity_tolerance_rad),
      1e-6);
  executor_config.soft_final_position_tolerance_rad = std::max(
      node_->declare_parameter("executor.soft_final_position_tolerance_rad", executor_config.soft_final_position_tolerance_rad),
      1e-6);
  executor_config.soft_final_velocity_tolerance_rad = std::max(
      node_->declare_parameter("executor.soft_final_velocity_tolerance_rad", executor_config.soft_final_velocity_tolerance_rad),
      1e-6);

  const int zone_valid_min_mm = node_->declare_parameter("zone_valid_min_mm", 0);
  const int zone_valid_max_mm = node_->declare_parameter("zone_valid_max_mm", 200);
  runtime_context_->motionOptionsState().setZoneValidRange(zone_valid_min_mm, zone_valid_max_mm);

  runtime::RuntimeControlBridgeConfig control_bridge_config;
  control_bridge_config.rt_default_fields = {rokae::RtSupportedFields::jointPos_m,
                                             rokae::RtSupportedFields::jointVel_m,
                                             rokae::RtSupportedFields::tau_m};
  control_bridge_config.rt_use_state_data_in_loop = true;
  control_bridge_config.rt_network_tolerance_configured = true;
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
  control_bridge_config.collision_retreat_distance = std::max(
      node_->declare_parameter("collision_retreat_distance", control_bridge_config.collision_retreat_distance),
      0.0);

  kinematics_ = std::make_unique<xMate3Kinematics>();
  motion_backend_ = std::make_unique<GazeboRuntimeBackend>(joints_, original_joint_limits_);
  if (backend_mode_ != BackendMode::effort && joint_names_ != nullptr) {
    motion_backend_->configureTrajectoryClient(node_, *joint_names_);
  }
  runtime_context_->attachBackend(motion_backend_.get());
  runtime_context_->motionRuntime().setExecutorConfig(executor_config);
  const std::string inferred_runtime_profile =
      backend_mode_ == BackendMode::effort ? std::string{"rt_sim_experimental_best_effort"} :
      std::string{"nrt_strict_parity"};
  const std::string requested_service_exposure_profile =
      node_->declare_parameter("service_exposure_profile", getenvOrDefault("ROKAE_SERVICE_EXPOSURE_PROFILE", to_string(defaultServiceExposureProfile())));
  service_exposure_profile_ = parseServiceExposureProfile(requested_service_exposure_profile);
  const std::string requested_runtime_profile = node_->declare_parameter("runtime_profile", inferred_runtime_profile);
  auto host_bootstrap = host_builder_->resolveBootstrap(
      requested_runtime_profile,
      runtime::RuntimeHostKind::gazebo_plugin,
      toString(backend_mode_),
      diagnosticCapabilityFlags(backend_mode_));
  rt_profile_config_ = host_bootstrap.rt_profile;
  host_builder_->configureContext(*runtime_context_, host_bootstrap, true);
  const bool rt_scheduler_enable = node_->declare_parameter<bool>("rt_scheduler.enable", true);
  const std::string rt_scheduler_policy = node_->declare_parameter<std::string>("rt_scheduler.policy", "fifo");
  const int rt_scheduler_priority = node_->declare_parameter<int>("rt_scheduler.priority", 80);
  const std::string rt_scheduler_cpu_affinity = node_->declare_parameter<std::string>("rt_scheduler.cpu_affinity", "");
  const bool rt_memory_lock_all = node_->declare_parameter<bool>("rt_memory.lock_all", true);
  if (rt_scheduler_enable) {
    runtime_context_->diagnosticsState().setRtSchedulerState("host_managed(gazebo_update_thread)");
    RCLCPP_INFO(
        node_->get_logger(),
        "rt scheduler request is host-managed for Gazebo plugin runtime; policy=%s priority=%d cpu_affinity=\"%s\" lock_all=%s",
        rt_scheduler_policy.c_str(),
        rt_scheduler_priority,
        rt_scheduler_cpu_affinity.c_str(),
        rt_memory_lock_all ? "true" : "false");
  } else {
    runtime_context_->diagnosticsState().setRtSchedulerState("disabled");
  }
  host_bootstrap.control_bridge_config.rt_default_fields = control_bridge_config.rt_default_fields;
  host_bootstrap.control_bridge_config.rt_use_state_data_in_loop = control_bridge_config.rt_use_state_data_in_loop;
  host_bootstrap.control_bridge_config.rt_network_tolerance_configured = control_bridge_config.rt_network_tolerance_configured;
  host_bootstrap.control_bridge_config.collision_nominal_thresholds = control_bridge_config.collision_nominal_thresholds;
  host_bootstrap.control_bridge_config.collision_slow_scale = control_bridge_config.collision_slow_scale;
  host_bootstrap.control_bridge_config.collision_retreat_distance = control_bridge_config.collision_retreat_distance;
  runtime_context_->diagnosticsState().setSessionModes(
      runtime_context_->sessionState().motionMode(),
      runtime_context_->sessionState().rtControlMode());
  attachExecutorNode();
  initRuntimeBindings(host_bootstrap);
  initPrepareShutdownService();
  startExecutorThread();

  RCLCPP_INFO(
      node_->get_logger(),
      "runtime diagnostics ready: backend=%s requested_profile=%s effective_profile=%s service_exposure_profile=%s rt_level=best_effort_non_controller_grade aliases=[get_joint_torque,get_end_torque] "
      "services=[/xmate3/internal/get_runtime_diagnostics] "
      "topic=[/xmate3/internal/runtime_status] profile_summary=%s",
      toString(backend_mode_),
      requested_runtime_profile.c_str(),
      rt_profile_config_.effective_profile.c_str(),
      to_string(service_exposure_profile_),
      runtime::summarizeRuntimeRtProfile(rt_profile_config_).c_str());
}


void RuntimeBootstrap::attachExecutorNode() {
  executor_state_ = host_builder_->attachExecutor(
      ros_integration_.executor,
      ros_integration_.attach_to_executor,
      rt_profile_config_.executor_threads);
}

void RuntimeBootstrap::initRuntimeBindings(const runtime::RuntimeHostBootstrapConfig &host_bootstrap) {
  auto time_provider = [this]() { return node_->get_clock()->now(); };
  auto trajectory_dt_provider = [this]() { return trajectory_sample_dt_; };
  auto request_id_generator = [this](const std::string &prefix) {
    return prefix + std::to_string(next_request_id_.fetch_add(1));
  };

  initPublishers();
  publish_bridge_ = host_builder_->createPublishBridge(*runtime_context_);
  ros_bindings_ = host_builder_->createRosBindings(
      *runtime_context_,
      publish_bridge_.get(),
      *kinematics_,
      joint_state_fetcher_,
      time_provider,
      trajectory_dt_provider,
      request_id_generator,
      service_exposure_profile_,
      rt_profile_config_);
  control_bridge_ = host_builder_->createControlBridge(*runtime_context_, host_bootstrap);
  initPublishTimer();
}


void RuntimeBootstrap::initPublishTimer() {
  if (!node_ || !publish_bridge_) {
    return;
  }
  publish_timer_ = host_builder_->createPublishTimer(
      *publish_bridge_,
      host_publishers_,
      joint_names_,
      joint_state_fetcher_,
      rt_profile_config_,
      [this]() {
        if (shutting_down_.load() || !publish_bridge_ || !host_publishers_.joint_state_pub ||
            !host_publishers_.operation_state_pub) {
          return false;
        }
        static_cast<void>(collectShutdownContractState(false));
        return true;
      });
}

void RuntimeBootstrap::initPrepareShutdownService() {
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
}

void RuntimeBootstrap::startExecutorThread() {
  host_builder_->startExecutor(executor_state_);
}

void RuntimeBootstrap::releaseExecutorNode() {
  if (host_builder_) {
    host_builder_->releaseExecutor(executor_state_);
  }
}

void RuntimeBootstrap::shutdown(const std::string &reason) {
  std::lock_guard<std::mutex> shutdown_lock(shutdown_prepare_mutex_);
  if (shutdown_prepared_) {
    return;
  }

  shutting_down_.store(true);
  if (motion_backend_) {
    motion_backend_->clearControl();
    motion_backend_->cancelTrajectoryExecution(reason);
    motion_backend_->beginShutdown();
  }

  if (runtime_context_) {
    runtime_context_->motionRuntime().stop(reason);
    runtime_context_->motionRuntime().reset();
  }

  shutdown_prepared_ = true;
}

runtime::ShutdownContractView RuntimeBootstrap::collectShutdownContractState(bool request_prepare) {
  if (request_prepare) {
    shutdown_coordinator_.beginShutdown("internal prepare shutdown");
    shutdown("internal prepare shutdown");
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
  facts.plugin_detached = shutting_down_.load();
  facts.faulted = facts.faulted || facts.runtime_phase == runtime::RuntimePhase::faulted;
  if (facts.message.empty()) {
    facts.message = shutdown_prepared_ ? "shutdown prepared" : "shutdown requested";
  }

  shutdown_coordinator_.updateFacts(facts);
  state = shutdown_coordinator_.currentView();
  runtime_context_->diagnosticsState().updateShutdownContract(state);
  return state;
}

void RuntimeBootstrap::initPublishers() {
  host_publishers_ = host_builder_->createPublishers();
}

}  // namespace gazebo
