#ifndef ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_HOST_BUILDER_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_RUNTIME_HOST_BUILDER_HPP

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/runtime_diagnostics.hpp"
#include "runtime/ros_bindings.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/runtime_control_bridge.hpp"
#include "runtime/runtime_publish_bridge.hpp"
#include "runtime/rt_runtime_profile.hpp"

namespace rokae_xmate3_ros2::runtime {

struct RuntimeHostBootstrapConfig {
  std::string backend_label{"unknown"};
  std::vector<std::string> capability_flags;
  RuntimeRtProfileConfig rt_profile{};
  RuntimeControlBridgeConfig control_bridge_config{};
};

struct RuntimeHostPublishers {
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Publisher<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_pub;
  rclcpp::Publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>::SharedPtr runtime_diagnostics_pub;
};

struct RuntimeHostExecutorState {
  std::shared_ptr<rclcpp::Executor> executor;
  bool attached_external_executor = false;
  bool executor_node_added_by_builder = false;
  std::thread executor_thread;
};

using RuntimeHostJointStateFetcher = std::function<void(std::array<double, 6> &,
                                                        std::array<double, 6> &,
                                                        std::array<double, 6> &)>;
using RuntimeHostTimeProvider = std::function<rclcpp::Time()>;
using RuntimeHostTrajectoryDtProvider = std::function<double()>;
using RuntimeHostRequestIdGenerator = std::function<std::string(const std::string &)>;
using RuntimeHostShouldPublish = std::function<bool()>;

/**
 * @brief Build the single runtime-host bootstrap contract used by daemon and plugin hosts.
 * @param requested_profile Requested runtime profile string.
 * @param host_kind Runtime host ownership shape.
 * @param backend_label Stable diagnostics backend label.
 * @param capability_flags Capability flags advertised by the host/backend.
 * @return Fully resolved runtime-profile + control-bridge bootstrap contract.
 * @throws std::runtime_error when the requested profile is unsupported for the selected host.
 */
[[nodiscard]] RuntimeHostBootstrapConfig buildRuntimeHostBootstrapConfig(
    const std::string &requested_profile,
    RuntimeHostKind host_kind,
    std::string backend_label,
    std::vector<std::string> capability_flags);

/**
 * @brief Single builder for runtime-host bootstrap, assembly and executor lifecycle.
 *
 * This builder is shared by the authoritative daemon host and the Gazebo plugin host so they
 * resolve the same runtime profile, publish surface, bindings assembly and executor ownership
 * semantics. Host-specific control loops remain outside, but runtime-node lifecycle and assembly
 * are centralized here to keep the authoritative host contract singular.
 */
class RuntimeHostBuilder {
 public:
  explicit RuntimeHostBuilder(rclcpp::Node::SharedPtr node);

  [[nodiscard]] RuntimeHostBootstrapConfig resolveBootstrap(
      const std::string &requested_profile,
      RuntimeHostKind host_kind,
      std::string backend_label,
      std::vector<std::string> capability_flags) const;

  void configureContext(RuntimeContext &runtime_context,
                        const RuntimeHostBootstrapConfig &host_bootstrap,
                        bool simulation_mode) const;

  [[nodiscard]] RuntimeHostPublishers createPublishers() const;

  [[nodiscard]] std::unique_ptr<RuntimePublishBridge> createPublishBridge(
      RuntimeContext &runtime_context) const;

  [[nodiscard]] std::unique_ptr<RuntimeControlBridge> createControlBridge(
      RuntimeContext &runtime_context,
      const RuntimeHostBootstrapConfig &host_bootstrap) const;

  [[nodiscard]] std::unique_ptr<RosBindings> createRosBindings(
      RuntimeContext &runtime_context,
      RuntimePublishBridge *publish_bridge,
      gazebo::xMate3Kinematics &kinematics,
      RuntimeHostJointStateFetcher joint_state_fetcher,
      RuntimeHostTimeProvider time_provider,
      RuntimeHostTrajectoryDtProvider trajectory_dt_provider,
      RuntimeHostRequestIdGenerator request_id_generator,
      ServiceExposureProfile service_exposure_profile,
      const RuntimeRtProfileConfig &rt_profile) const;

  [[nodiscard]] rclcpp::TimerBase::SharedPtr createPublishTimer(
      RuntimePublishBridge &publish_bridge,
      const RuntimeHostPublishers &publishers,
      const std::vector<std::string> *joint_names,
      RuntimeHostJointStateFetcher joint_state_fetcher,
      const RuntimeRtProfileConfig &rt_profile,
      RuntimeHostShouldPublish should_publish = {}) const;

  [[nodiscard]] RuntimeHostExecutorState attachExecutor(
      std::shared_ptr<rclcpp::Executor> external_executor,
      bool attach_to_external_executor,
      int executor_threads) const;

  void startExecutor(RuntimeHostExecutorState &state) const;
  void releaseExecutor(RuntimeHostExecutorState &state) const;

 private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
