#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
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

namespace {

std::string applyRtScheduler(rclcpp::Node &node,
                             const bool enable,
                             const std::string &policy,
                             const int priority,
                             const std::string &cpu_affinity,
                             const bool lock_all_memory) {
  if (!enable) {
    return "disabled";
  }

  std::string state = "active";
  if (lock_all_memory) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      state = "degraded_best_effort(memory_lock_failed)";
    }
  }

  int sched_policy = SCHED_FIFO;
  if (policy == "rr") {
    sched_policy = SCHED_RR;
  } else if (policy != "fifo") {
    sched_policy = SCHED_OTHER;
  }
  sched_param sched{};
  sched.sched_priority = std::clamp(priority, 1, 95);
  if (sched_setscheduler(0, sched_policy, &sched) != 0) {
    state = "degraded_best_effort(scheduler_failed)";
  }

  if (!cpu_affinity.empty()) {
    cpu_set_t set;
    CPU_ZERO(&set);
    bool any = false;
    std::size_t start = 0;
    while (start <= cpu_affinity.size()) {
      const auto end = cpu_affinity.find(',', start);
      const std::string token = cpu_affinity.substr(start, end == std::string::npos ? std::string::npos : end - start);
      if (!token.empty()) {
        try {
          const int cpu = std::stoi(token);
          if (cpu >= 0 && cpu < CPU_SETSIZE) {
            CPU_SET(cpu, &set);
            any = true;
          }
        } catch (...) {
        }
      }
      if (end == std::string::npos) {
        break;
      }
      start = end + 1;
    }
    if (any && pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      state = "degraded_best_effort(affinity_failed)";
    }
  }
  RCLCPP_INFO(
      node.get_logger(),
      "rt scheduler setup: enable=%s policy=%s priority=%d cpu_affinity=\"%s\" lock_all=%s state=%s",
      enable ? "true" : "false",
      policy.c_str(),
      priority,
      cpu_affinity.c_str(),
      lock_all_memory ? "true" : "false",
      state.c_str());
  return state;
}

}  // namespace

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
          "rt.transport.shm_ring",
          "rt.transport.ros_topic",
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

  const bool rt_scheduler_enable = node->declare_parameter<bool>("rt_scheduler.enable", true);
  const std::string rt_scheduler_policy = node->declare_parameter<std::string>("rt_scheduler.policy", "fifo");
  const int rt_scheduler_priority = node->declare_parameter<int>("rt_scheduler.priority", 80);
  const std::string rt_scheduler_cpu_affinity = node->declare_parameter<std::string>("rt_scheduler.cpu_affinity", "");
  const bool rt_memory_lock_all = node->declare_parameter<bool>("rt_memory.lock_all", true);

  std::vector<std::string> joint_names;
  joint_names.reserve(rokae_xmate3_ros2::spec::xmate3::kJointNames.size());
  for (const auto *name : rokae_xmate3_ros2::spec::xmate3::kJointNames) {
    joint_names.emplace_back(name);
  }

  std::atomic<bool> running{true};
  std::thread control_thread([&]() {
    const auto scheduler_state = applyRtScheduler(
        *node,
        rt_scheduler_enable,
        rt_scheduler_policy,
        rt_scheduler_priority,
        rt_scheduler_cpu_affinity,
        rt_memory_lock_all);
    runtime_context.diagnosticsState().setRtSchedulerState(scheduler_state);
    auto last_tick = std::chrono::steady_clock::now();
    auto next_tick = last_tick + 1ms;
    while (rclcpp::ok() && running.load()) {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - last_tick).count();
      last_tick = now;
      backend.step(dt, runtime_context.sessionState().powerOn());
      const auto snapshot = backend.readSnapshot();
      const auto tick_result = control_bridge.tick(backend, snapshot, dt);
      publish_bridge.emitRuntimeStatus(tick_result.status, node->now(), node->get_logger());
      std::this_thread::sleep_until(next_tick);
      next_tick += 1ms;
    }
  });

  auto timer = node->create_wall_timer(1ms, [&]() {
    const auto snapshot = backend.readSnapshot();
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
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  running.store(false);
  if (control_thread.joinable()) {
    control_thread.join();
  }
  return 0;
}

}  // namespace rokae_xmate3_ros2::runtime

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const int code = rokae_xmate3_ros2::runtime::run_sim_runtime_main();
  rclcpp::shutdown();
  return code;
}
