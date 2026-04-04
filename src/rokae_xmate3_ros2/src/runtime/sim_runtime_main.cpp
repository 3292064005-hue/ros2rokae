#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/runtime_diagnostics.hpp"
#include "rokae_xmate3_ros2/runtime/ros_context_owner.hpp"
#include "rokae_xmate3_ros2/spec/xmate3_spec.hpp"
#include "runtime/mock_runtime_backend.hpp"
#include "runtime/ros_bindings.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/runtime_control_bridge.hpp"
#include "runtime/runtime_publish_bridge.hpp"

namespace rokae_xmate3_ros2::runtime {

int run_sim_runtime_main() {
  using namespace std::chrono_literals;

  auto ros_context_lease = RosContextOwner::acquire("rokae_sim_runtime");
  auto node = std::make_shared<rclcpp::Node>("rokae_sim_runtime");

  RuntimeContext runtime_context;
  HeadlessMockRuntimeBackend backend;
  runtime_context.attachBackend(&backend);

  const auto runtime_profile = node->declare_parameter<std::string>("runtime_profile", "nrt_strict_parity");
  runtime_context.diagnosticsState().configure(
      "daemonized_headless_mock",
      {
          "backend.headless_mock",
          "runtime.daemonized",
          "rt.experimental",
          "rt.best_effort_non_controller_grade",
          "profile.nrt_strict_parity",
          "profile.rt_sim_experimental_best_effort",
      },
      runtime_profile);
  runtime_context.sessionState().setSimulationMode(true);

  RuntimeControlBridgeConfig control_bridge_config;
  RuntimeControlBridge control_bridge(runtime_context, control_bridge_config);
  RuntimePublishBridge publish_bridge(runtime_context);
  gazebo::xMate3Kinematics kinematics;

  auto joint_state_fetcher = [&backend](std::array<double, 6> &position,
                                        std::array<double, 6> &velocity,
                                        std::array<double, 6> &torque) {
    const auto snapshot = backend.readSnapshot();
    position = snapshot.joint_position;
    velocity = snapshot.joint_velocity;
    torque = snapshot.joint_torque;
  };

  auto time_provider = [node]() { return node->get_clock()->now(); };
  auto trajectory_dt_provider = []() { return rokae_xmate3_ros2::spec::xmate3::kServoTickSec; };
  auto request_seed = std::make_shared<std::atomic<std::uint64_t>>(1);
  auto request_id_generator = [request_seed](const std::string &prefix) {
    return prefix + std::to_string(request_seed->fetch_add(1));
  };

  RosBindings bindings(node,
                       runtime_context,
                       &publish_bridge,
                       kinematics,
                       joint_state_fetcher,
                       time_provider,
                       trajectory_dt_provider,
                       request_id_generator);

  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>("/xmate3/joint_states", 10);
  auto operation_state_pub =
      node->create_publisher<rokae_xmate3_ros2::msg::OperationState>("/xmate3/cobot/operation_state", 10);
  auto runtime_diagnostics_pub =
      node->create_publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>("/xmate3/internal/runtime_status", 10);

  std::vector<std::string> joint_names;
  joint_names.reserve(rokae_xmate3_ros2::spec::xmate3::kJointNames.size());
  for (const auto *name : rokae_xmate3_ros2::spec::xmate3::kJointNames) {
    joint_names.emplace_back(name);
  }

  auto last_tick = std::chrono::steady_clock::now();
  auto timer = node->create_wall_timer(1ms, [&]() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_tick).count();
    last_tick = now;

    backend.step(dt, runtime_context.sessionState().powerOn());
    const auto snapshot = backend.readSnapshot();
    const auto tick_result = control_bridge.tick(backend, snapshot, dt);
    publish_bridge.emitRuntimeStatus(tick_result.status, node->now(), node->get_logger());

    PublisherTickInput tick_input;
    tick_input.stamp = node->now();
    tick_input.frame_id = "base_link";
    tick_input.joint_names = &joint_names;
    tick_input.position = snapshot.joint_position;
    tick_input.velocity = snapshot.joint_velocity;
    tick_input.torque = snapshot.joint_torque;
    tick_input.min_publish_period_sec = 0.001;

    const auto publish_tick = publish_bridge.buildPublisherTick(tick_input);
    if (publish_tick.publish_joint_state) {
      joint_state_pub->publish(publish_tick.joint_state);
    }
    if (publish_tick.publish_operation_state) {
      operation_state_pub->publish(publish_tick.operation_state);
      runtime_diagnostics_pub->publish(publish_bridge.buildRuntimeDiagnosticsMessage());
    }
  });
  (void)timer;

  RCLCPP_INFO(
      node->get_logger(),
      "rokae_sim_runtime is running (authoritative runtime daemon). active_profile=%s, rt_policy=best_effort_non_controller_grade",
      runtime_profile.c_str());
  rclcpp::spin(node);
  return 0;
}

}  // namespace rokae_xmate3_ros2::runtime

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const int code = rokae_xmate3_ros2::runtime::run_sim_runtime_main();
  rclcpp::shutdown();
  return code;
}
