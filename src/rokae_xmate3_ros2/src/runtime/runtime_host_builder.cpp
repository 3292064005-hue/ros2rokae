#include "runtime/runtime_host_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace rokae_xmate3_ros2::runtime {

RuntimeHostBootstrapConfig buildRuntimeHostBootstrapConfig(
    const std::string &requested_profile,
    RuntimeHostKind host_kind,
    std::string backend_label,
    std::vector<std::string> capability_flags) {
  RuntimeHostBootstrapConfig config;
  config.backend_label = std::move(backend_label);
  config.capability_flags = std::move(capability_flags);
  config.rt_profile = resolveRuntimeRtProfile(requested_profile, host_kind);
  if (!config.rt_profile.supported) {
    throw std::runtime_error(config.rt_profile.startup_error.empty()
                                 ? std::string{"unsupported runtime host profile"}
                                 : config.rt_profile.startup_error);
  }

  config.control_bridge_config.authoritative_servo_clock = config.rt_profile.authoritative_servo_clock;
  config.control_bridge_config.authoritative_servo_period_sec = config.rt_profile.servo_period_sec;
  config.control_bridge_config.allow_topic_rt_transport = config.rt_profile.allow_topic_rt_ingress;
  config.control_bridge_config.allow_legacy_rt_custom_data = config.rt_profile.allow_legacy_rt_custom_data;
  config.control_bridge_config.require_shm_rt_transport = config.rt_profile.require_shm_transport;
  config.control_bridge_config.fail_on_rt_deadline_miss = config.rt_profile.fail_on_rt_deadline_miss;
  return config;
}

RuntimeHostBuilder::RuntimeHostBuilder(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

RuntimeHostBootstrapConfig RuntimeHostBuilder::resolveBootstrap(
    const std::string &requested_profile,
    RuntimeHostKind host_kind,
    std::string backend_label,
    std::vector<std::string> capability_flags) const {
  return buildRuntimeHostBootstrapConfig(
      requested_profile,
      host_kind,
      std::move(backend_label),
      std::move(capability_flags));
}

void RuntimeHostBuilder::configureContext(RuntimeContext &runtime_context,
                                          const RuntimeHostBootstrapConfig &host_bootstrap,
                                          bool simulation_mode) const {
  runtime_context.diagnosticsState().configure(
      host_bootstrap.backend_label,
      host_bootstrap.capability_flags,
      host_bootstrap.rt_profile.effective_profile);
  runtime_context.diagnosticsState().setRuntimeOptionSummary(
      summarizeRuntimeRtProfile(host_bootstrap.rt_profile));
  runtime_context.sessionState().setSimulationMode(simulation_mode);
}

RuntimeHostPublishers RuntimeHostBuilder::createPublishers() const {
  RuntimeHostPublishers publishers;
  publishers.joint_state_pub =
      node_->create_publisher<sensor_msgs::msg::JointState>("/xmate3/joint_states", 10);
  publishers.operation_state_pub =
      node_->create_publisher<rokae_xmate3_ros2::msg::OperationState>("/xmate3/cobot/operation_state", 10);
  publishers.runtime_diagnostics_pub =
      node_->create_publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>("/xmate3/internal/runtime_status", 10);
  return publishers;
}

std::unique_ptr<RuntimePublishBridge> RuntimeHostBuilder::createPublishBridge(
    RuntimeContext &runtime_context) const {
  return std::make_unique<RuntimePublishBridge>(runtime_context);
}

std::unique_ptr<RuntimeControlBridge> RuntimeHostBuilder::createControlBridge(
    RuntimeContext &runtime_context,
    const RuntimeHostBootstrapConfig &host_bootstrap) const {
  return std::make_unique<RuntimeControlBridge>(runtime_context, host_bootstrap.control_bridge_config);
}

std::unique_ptr<RosBindings> RuntimeHostBuilder::createRosBindings(
    RuntimeContext &runtime_context,
    RuntimePublishBridge *publish_bridge,
    gazebo::xMate3Kinematics &kinematics,
    RuntimeHostJointStateFetcher joint_state_fetcher,
    RuntimeHostTimeProvider time_provider,
    RuntimeHostTrajectoryDtProvider trajectory_dt_provider,
    RuntimeHostRequestIdGenerator request_id_generator,
    ServiceExposureProfile service_exposure_profile,
    const RuntimeRtProfileConfig &rt_profile) const {
  return std::make_unique<RosBindings>(
      node_,
      runtime_context,
      publish_bridge,
      kinematics,
      std::move(joint_state_fetcher),
      std::move(time_provider),
      std::move(trajectory_dt_provider),
      std::move(request_id_generator),
      RosBindingsRtIngressOptions{rt_profile.allow_topic_rt_ingress, service_exposure_profile});
}

rclcpp::TimerBase::SharedPtr RuntimeHostBuilder::createPublishTimer(
    RuntimePublishBridge &publish_bridge,
    const RuntimeHostPublishers &publishers,
    const std::vector<std::string> *joint_names,
    RuntimeHostJointStateFetcher joint_state_fetcher,
    const RuntimeRtProfileConfig &rt_profile,
    RuntimeHostShouldPublish should_publish) const {
  const auto publish_period_ms = std::max<int>(
      1,
      static_cast<int>(std::lround(
          std::min({rt_profile.publish_rates.joint_state_period_sec,
                    rt_profile.publish_rates.operation_state_period_sec,
                    rt_profile.publish_rates.diagnostics_period_sec}) *
          1000.0)));
  return node_->create_wall_timer(std::chrono::milliseconds(publish_period_ms),
                                  [this,
                                   &publish_bridge,
                                   publishers,
                                   joint_names,
                                   joint_state_fetcher = std::move(joint_state_fetcher),
                                   rt_profile,
                                   should_publish = std::move(should_publish)]() {
    if (should_publish && !should_publish()) {
      return;
    }

    std::array<double, 6> position{};
    std::array<double, 6> velocity{};
    std::array<double, 6> torque{};
    if (joint_state_fetcher) {
      joint_state_fetcher(position, velocity, torque);
    }

    PublisherTickInput tick_input;
    tick_input.stamp = node_->now();
    tick_input.frame_id = "base_link";
    tick_input.joint_names = joint_names;
    tick_input.position = position;
    tick_input.velocity = velocity;
    tick_input.torque = torque;
    tick_input.min_publish_period_sec = 0.0;
    tick_input.joint_state_publish_period_sec = rt_profile.publish_rates.joint_state_period_sec;
    tick_input.operation_state_publish_period_sec = rt_profile.publish_rates.operation_state_period_sec;
    tick_input.diagnostics_publish_period_sec = rt_profile.publish_rates.diagnostics_period_sec;

    const auto publish_tick = publish_bridge.buildPublisherTick(tick_input);
    if (publish_tick.publish_joint_state && publishers.joint_state_pub) {
      publishers.joint_state_pub->publish(publish_tick.joint_state);
    }
    if (publish_tick.publish_operation_state && publishers.operation_state_pub) {
      publishers.operation_state_pub->publish(publish_tick.operation_state);
    }
    if (publish_tick.publish_runtime_diagnostics && publishers.runtime_diagnostics_pub != nullptr) {
      publishers.runtime_diagnostics_pub->publish(publish_bridge.buildRuntimeDiagnosticsMessage());
    }
  });
}

RuntimeHostExecutorState RuntimeHostBuilder::attachExecutor(
    std::shared_ptr<rclcpp::Executor> external_executor,
    bool attach_to_external_executor,
    int executor_threads) const {
  RuntimeHostExecutorState state;
  if (external_executor && attach_to_external_executor) {
    state.executor = std::move(external_executor);
    auto node_base = node_->get_node_base_interface();
    const bool already_associated =
        node_base && node_base->get_associated_with_executor_atomic().load();
    if (already_associated) {
      state.attached_external_executor = true;
      state.executor_node_added_by_builder = false;
      return state;
    }
    state.executor->add_node(node_);
    state.attached_external_executor = true;
    state.executor_node_added_by_builder = true;
    return state;
  }

  if (executor_threads <= 1) {
    state.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  } else {
    state.executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), executor_threads);
  }
  state.executor->add_node(node_);
  state.executor_node_added_by_builder = true;
  return state;
}

void RuntimeHostBuilder::startExecutor(RuntimeHostExecutorState &state) const {
  if (state.attached_external_executor || !state.executor) {
    return;
  }
  state.executor_thread = std::thread([this, &state]() {
    try {
      state.executor->spin();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "ROS executor stopped unexpectedly: %s", e.what());
    }
  });
}

void RuntimeHostBuilder::releaseExecutor(RuntimeHostExecutorState &state) const {
  if (state.executor && !state.attached_external_executor) {
    state.executor->cancel();
  }
  if (state.executor_thread.joinable()) {
    state.executor_thread.join();
  }
  if (state.executor && node_ && state.executor_node_added_by_builder) {
    try {
      auto node_base = node_->get_node_base_interface();
      if (node_base && node_base->get_associated_with_executor_atomic().load()) {
        state.executor->remove_node(node_);
      }
    } catch (const std::exception &) {
    }
  }
  state.attached_external_executor = false;
  state.executor_node_added_by_builder = false;
}

}  // namespace rokae_xmate3_ros2::runtime
