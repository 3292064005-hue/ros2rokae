#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
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
#include "runtime/runtime_host_builder.hpp"
#include "runtime/runtime_publish_bridge.hpp"
#include "runtime/rt_runtime_profile.hpp"
#include "runtime/rt_scheduler.hpp"

namespace rokae_xmate3_ros2::runtime {

namespace {

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

int run_sim_runtime_main() {
  using namespace std::chrono_literals;

  auto ros_context_lease = RosContextOwner::acquire("rokae_sim_runtime");
  auto node = std::make_shared<rclcpp::Node>("rokae_sim_runtime");

  RuntimeContext runtime_context;
  HeadlessMockRuntimeBackend backend;
  runtime_context.attachBackend(&backend);

  const auto requested_service_exposure_profile =
      node->declare_parameter<std::string>("service_exposure_profile", getenvOrDefault("ROKAE_SERVICE_EXPOSURE_PROFILE", to_string(defaultServiceExposureProfile())));
  const auto service_exposure_profile = parseServiceExposureProfile(requested_service_exposure_profile);
  const auto requested_runtime_profile = node->declare_parameter<std::string>("runtime_profile", "nrt_strict_parity");
  RuntimeHostBuilder host_builder(node);
  RuntimeHostBootstrapConfig host_bootstrap;
  try {
    host_bootstrap = host_builder.resolveBootstrap(
        requested_runtime_profile,
        RuntimeHostKind::daemonized_runtime,
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
            "profile.rt_hardened",
            "profile.hard_1khz",
        });
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(node->get_logger(), "requested runtime_profile=%s is not supported: %s", requested_runtime_profile.c_str(), ex.what());
    return 2;
  }
  const auto &runtime_profile = host_bootstrap.rt_profile;

  host_builder.configureContext(runtime_context, host_bootstrap, true);

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

  auto publish_bridge = host_builder.createPublishBridge(runtime_context);
  auto bindings = host_builder.createRosBindings(
      runtime_context,
      publish_bridge.get(),
      kinematics,
      joint_state_fetcher,
      time_provider,
      trajectory_dt_provider,
      request_id_generator,
      service_exposure_profile,
      runtime_profile);
  auto control_bridge = host_builder.createControlBridge(runtime_context, host_bootstrap);
  auto publishers = host_builder.createPublishers();

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
  std::promise<bool> scheduler_ready_promise;
  auto scheduler_ready_future = scheduler_ready_promise.get_future();
  std::atomic<bool> scheduler_ready_announced{false};
  std::thread control_thread([&]() {
    const auto announce_scheduler_ready = [&](bool ok) {
      bool expected = false;
      if (scheduler_ready_announced.compare_exchange_strong(expected, true)) {
        scheduler_ready_promise.set_value(ok);
      }
    };

    try {
      const auto scheduler_result = applyRtScheduler(
          node->get_logger(),
          RtSchedulerRequest{rt_scheduler_enable,
                             rt_scheduler_policy,
                             rt_scheduler_priority,
                             rt_scheduler_cpu_affinity,
                             rt_memory_lock_all,
                             runtime_profile.fail_on_degraded_scheduler});
      runtime_context.diagnosticsState().setRtSchedulerState(scheduler_result.state);
      announce_scheduler_ready(!scheduler_result.hard_failure);
      if (scheduler_result.hard_failure) {
        running.store(false);
        return;
      }
      auto last_tick = std::chrono::steady_clock::now();
      auto next_tick = last_tick + 1ms;
      while (rclcpp::ok() && running.load()) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_tick).count();
        last_tick = now;
        backend.step(dt, runtime_context.sessionState().powerOn());
        const auto snapshot = backend.readSnapshot();
        const auto tick_result = control_bridge->tick(backend, snapshot, dt);
        publish_bridge->emitRuntimeStatus(tick_result.status, node->now(), node->get_logger());
        std::this_thread::sleep_until(next_tick);
        next_tick += 1ms;
      }
    } catch (const std::exception &e) {
      announce_scheduler_ready(false);
      runtime_context.diagnosticsState().setRtSchedulerState("thread_fault");
      RCLCPP_ERROR(node->get_logger(), "rokae_sim_runtime control thread failed: %s", e.what());
      running.store(false);
    } catch (...) {
      announce_scheduler_ready(false);
      runtime_context.diagnosticsState().setRtSchedulerState("thread_fault");
      RCLCPP_ERROR(node->get_logger(), "rokae_sim_runtime control thread failed with unknown exception");
      running.store(false);
    }
  });

  if (!scheduler_ready_future.get()) {
    running.store(false);
    if (control_thread.joinable()) {
      control_thread.join();
    }
    RCLCPP_ERROR(node->get_logger(),
                 "rokae_sim_runtime failed to enter the requested strict RT scheduler contract");
    return 2;
  }

  auto timer = host_builder.createPublishTimer(
      *publish_bridge,
      publishers,
      &joint_names,
      joint_state_fetcher,
      runtime_profile,
      [&running]() { return running.load(); });
  (void)timer;

  RCLCPP_INFO(
      node->get_logger(),
      "rokae_sim_runtime is running (authoritative runtime daemon). requested_profile=%s effective_profile=%s service_exposure_profile=%s profile_summary=%s",
      requested_runtime_profile.c_str(),
      runtime_profile.effective_profile.c_str(),
      to_string(service_exposure_profile),
      summarizeRuntimeRtProfile(runtime_profile).c_str());
  auto executor_state = host_builder.attachExecutor(nullptr, false, runtime_profile.executor_threads);
  executor_state.executor->spin();
  host_builder.releaseExecutor(executor_state);
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
