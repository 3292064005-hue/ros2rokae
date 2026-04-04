#include "runtime/ros_bindings.hpp"

#include "runtime/runtime_publish_bridge.hpp"

namespace rokae_xmate3_ros2::runtime {

RosBindings::RosBindings(rclcpp::Node::SharedPtr node,
                         RuntimeContext &runtime_context,
                         RuntimePublishBridge *publish_bridge,
                         gazebo::xMate3Kinematics &kinematics,
                         JointStateFetcher joint_state_fetcher,
                         TimeProvider time_provider,
                         TrajectoryDtProvider trajectory_dt_provider,
                         RequestIdGenerator request_id_generator)
    : node_(std::move(node)),
      runtime_context_(runtime_context),
      publish_bridge_(publish_bridge),
      joint_state_fetcher_(std::move(joint_state_fetcher)),
      trajectory_dt_provider_(std::move(trajectory_dt_provider)),
      request_id_generator_(std::move(request_id_generator)),
      control_facade_(std::make_unique<ControlFacade>(runtime_context_.sessionState(),
                                                      runtime_context_.motionOptionsState(),
                                                      runtime_context_.toolingState(),
                                                      runtime_context_.backend(),
                                                      &runtime_context_.motionRuntime(),
                                                      &runtime_context_.requestCoordinator())),
      query_facade_(std::make_unique<QueryFacade>(runtime_context_.sessionState(),
                                                  runtime_context_.motionOptionsState(),
                                                  runtime_context_.toolingState(),
                                                  runtime_context_.dataStoreState(),
                                                  runtime_context_.programState(),
                                                  runtime_context_.diagnosticsState(),
                                                  kinematics,
                                                  joint_state_fetcher_,
                                                  std::move(time_provider),
                                                  trajectory_dt_provider_,
                                                  6)),
      io_program_facade_(std::make_unique<IoProgramFacade>(runtime_context_.sessionState(),
                                                           runtime_context_.dataStoreState(),
                                                           runtime_context_.programState(),
                                                           runtime_context_.toolingState(),
                                                           [this]() { return node_->get_clock()->now(); })),
      path_facade_(std::make_unique<PathFacade>(runtime_context_.sessionState(),
                                                runtime_context_.programState(),
                                                runtime_context_.toolingState(),
                                                &runtime_context_.requestCoordinator(),
                                                joint_state_fetcher_,
                                                trajectory_dt_provider_,
                                                request_id_generator_)) {
  initServices();
  registerCompatibilityAliases();
  initActionServers();
}

RosBindings::~RosBindings() {
  move_append_shutdown_requested_.store(true);
  runtime_context_.requestCoordinator().stop("move append worker teardown");
  std::lock_guard<std::mutex> lock(move_append_workers_mutex_);
  for (auto &worker : move_append_workers_) {
    if (worker.valid()) {
      worker.wait();
    }
  }
  move_append_workers_.clear();
}

}  // namespace rokae_xmate3_ros2::runtime
