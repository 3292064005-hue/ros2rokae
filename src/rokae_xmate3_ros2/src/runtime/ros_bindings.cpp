#include "runtime/ros_bindings.hpp"

#include <algorithm>
#include <chrono>

#include "runtime/runtime_publish_bridge.hpp"
#include "rokae_xmate3_ros2/runtime/rt_fast_command.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

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
  initRtIngress();
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

void RosBindings::initRtIngress() {
  rt_ingress_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = rt_ingress_group_;
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();
  rt_fast_command_sub_ = node_->create_subscription<rokae_xmate3_ros2::msg::RtFastCommand>(
      rt_topics::kFastCommandTopic,
      qos,
      [this](const rokae_xmate3_ros2::msg::RtFastCommand::SharedPtr msg) { handleRtFastCommand(msg); },
      options);
}

void RosBindings::handleRtFastCommand(const rokae_xmate3_ros2::msg::RtFastCommand::SharedPtr &msg) {
  RtFastCommandFrame frame;
  frame.sequence = msg->seq;
  frame.rt_mode = msg->rt_mode;
  frame.values = msg->values;
  frame.finished = msg->finished;
  frame.dispatch_mode = msg->dispatch_mode.empty() ? std::string{"independent_rt"} : msg->dispatch_mode;
  frame.transport = RtFastTransport::ros_topic;
  switch (msg->command_kind) {
    case rokae_xmate3_ros2::msg::RtFastCommand::COMMAND_JOINT_POSITION:
      frame.kind = RtFastCommandKind::joint_position;
      break;
    case rokae_xmate3_ros2::msg::RtFastCommand::COMMAND_CARTESIAN_POSITION:
      frame.kind = RtFastCommandKind::cartesian_position;
      break;
    case rokae_xmate3_ros2::msg::RtFastCommand::COMMAND_TORQUE:
      frame.kind = RtFastCommandKind::torque;
      break;
    case rokae_xmate3_ros2::msg::RtFastCommand::COMMAND_STOP:
      frame.kind = RtFastCommandKind::stop;
      break;
    default:
      frame.kind = RtFastCommandKind::joint_position;
      break;
  }

  auto receive_tp = std::chrono::steady_clock::now();
  const auto now_ros = node_->get_clock()->now();
  const auto sent_ros = rclcpp::Time(msg->send_time);
  const auto rx_latency_ns = std::max<std::int64_t>(0, now_ros.nanoseconds() - sent_ros.nanoseconds());
  frame.sent_at = receive_tp - std::chrono::nanoseconds(rx_latency_ns);
  runtime_context_.dataStoreState().ingestRtFastCommand(frame, 1);
}

}  // namespace rokae_xmate3_ros2::runtime
