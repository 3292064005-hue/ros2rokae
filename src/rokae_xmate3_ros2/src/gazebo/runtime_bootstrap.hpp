#ifndef ROKAE_XMATE3_ROS2_GAZEBO_RUNTIME_BOOTSTRAP_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_RUNTIME_BOOTSTRAP_HPP

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "gazebo/gazebo_runtime_backend.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/srv/prepare_shutdown.hpp"
#include "runtime/runtime_context.hpp"
#include "runtime/runtime_control_bridge.hpp"
#include "runtime/runtime_publish_bridge.hpp"
#include "runtime/ros_bindings.hpp"
#include "rokae_xmate3_ros2/runtime/shutdown_coordinator.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"
#include "rokae_xmate3_ros2/msg/runtime_diagnostics.hpp"

namespace gazebo {

class RuntimeBootstrap {
 public:
  using JointStateFetcher = std::function<void(std::array<double, 6> &,
                                               std::array<double, 6> &,
                                               std::array<double, 6> &)>;

  RuntimeBootstrap(BackendMode backend_mode,
                   std::vector<physics::JointPtr> *joints,
                   const std::array<std::pair<double, double>, 6> *original_joint_limits,
                   const std::vector<std::string> *joint_names,
                   JointStateFetcher joint_state_fetcher);
  ~RuntimeBootstrap();

  void start();
  void shutdown(const std::string &reason);
  [[nodiscard]] rokae_xmate3_ros2::runtime::ShutdownContractView collectShutdownContractState(
      bool request_prepare);

  [[nodiscard]] rclcpp::Node::SharedPtr node() const { return node_; }
  [[nodiscard]] GazeboRuntimeBackend *backend() const { return motion_backend_.get(); }
  [[nodiscard]] rokae_xmate3_ros2::runtime::RuntimeContext *runtimeContext() const {
    return runtime_context_.get();
  }
  [[nodiscard]] rokae_xmate3_ros2::runtime::RuntimeControlBridge *controlBridge() const {
    return control_bridge_.get();
  }
  [[nodiscard]] rokae_xmate3_ros2::runtime::RuntimePublishBridge *publishBridge() const {
    return publish_bridge_.get();
  }
  [[nodiscard]] rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher() const {
    return joint_state_pub_;
  }
  [[nodiscard]] rclcpp::Publisher<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operationStatePublisher() const {
    return operation_state_pub_;
  }
  [[nodiscard]] rclcpp::Publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>::SharedPtr runtimeDiagnosticsPublisher() const {
    return runtime_diagnostics_pub_;
  }
  [[nodiscard]] double trajectorySampleDt() const { return trajectory_sample_dt_; }
  [[nodiscard]] bool isShuttingDown() const { return shutting_down_.load(); }

 private:
  void initPublishers();

  BackendMode backend_mode_ = BackendMode::hybrid;
  std::vector<physics::JointPtr> *joints_ = nullptr;
  const std::array<std::pair<double, double>, 6> *original_joint_limits_ = nullptr;
  const std::vector<std::string> *joint_names_ = nullptr;
  JointStateFetcher joint_state_fetcher_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::unique_ptr<xMate3Kinematics> kinematics_;

  std::unique_ptr<rokae_xmate3_ros2::runtime::RuntimeContext> runtime_context_;
  std::unique_ptr<rokae_xmate3_ros2::runtime::RosBindings> ros_bindings_;
  std::unique_ptr<rokae_xmate3_ros2::runtime::RuntimeControlBridge> control_bridge_;
  std::unique_ptr<rokae_xmate3_ros2::runtime::RuntimePublishBridge> publish_bridge_;
  std::unique_ptr<GazeboRuntimeBackend> motion_backend_;

  std::atomic<uint64_t> next_request_id_{1};
  double trajectory_sample_dt_ = kDefaultTrajectorySampleDt;
  std::atomic<bool> shutting_down_{false};
  std::mutex shutdown_prepare_mutex_;
  bool shutdown_prepared_ = false;
  rokae_xmate3_ros2::runtime::ShutdownCoordinator shutdown_coordinator_;
  rclcpp::Service<rokae_xmate3_ros2::srv::PrepareShutdown>::SharedPtr prepare_shutdown_service_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<rokae_xmate3_ros2::msg::OperationState>::SharedPtr operation_state_pub_;
  rclcpp::Publisher<rokae_xmate3_ros2::msg::RuntimeDiagnostics>::SharedPtr runtime_diagnostics_pub_;
};

}  // namespace gazebo

#endif
