#include "gazebo/xcore_controller_gazebo_plugin.hpp"

#include <functional>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Console.hh>

namespace gazebo {

namespace runtime = rokae_xmate3_ros2::runtime;

XCoreControllerPlugin::~XCoreControllerPlugin() {
  update_conn_.reset();
  if (bootstrap_) {
    bootstrap_->shutdown("plugin shutdown");
    bootstrap_.reset();
  }
}

void XCoreControllerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  try {
    gzmsg << "[xCore Controller] Loading plugin for model: " << model->GetName() << std::endl;

    model_ = model;
    sdf_ = sdf;
    std::string backend_mode_value = "hybrid";
    if (sdf_ && sdf_->HasElement("backend_mode")) {
      backend_mode_value = sdf_->Get<std::string>("backend_mode");
    }
    const auto backend_mode = parseBackendMode(backend_mode_value);
    gzmsg << "[xCore Controller] backend_mode=" << toString(backend_mode) << std::endl;

    joint_names_ = {"xmate_joint_1", "xmate_joint_2", "xmate_joint_3",
                    "xmate_joint_4", "xmate_joint_5", "xmate_joint_6"};
    for (const auto &name : joint_names_) {
      auto joint = model_->GetJoint(name);
      if (joint) {
        joints_.push_back(joint);
      } else {
        gzerr << "[xCore] Joint not found: " << name << std::endl;
      }
    }
    joint_num_ = static_cast<int>(joints_.size());

    const std::array<std::pair<double, double>, 6> default_limits = {{
        std::make_pair(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[0], rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[0]),
        std::make_pair(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[1], rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[1]),
        std::make_pair(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[2], rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[2]),
        std::make_pair(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[3], rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[3]),
        std::make_pair(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[4], rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[4]),
        std::make_pair(rokae_xmate3_ros2::spec::xmate3::kJointLimitMin[5], rokae_xmate3_ros2::spec::xmate3::kJointLimitMax[5])}};
    for (int i = 0; i < 6 && i < joint_num_; ++i) {
      original_joint_limits_[i] = default_limits[i];
    }

    auto joint_state_fetcher = [this](std::array<double, 6> &position,
                                      std::array<double, 6> &velocity,
                                      std::array<double, 6> &torque) {
      joint_state_cache_.read(position, velocity, torque);
    };

    bootstrap_ = std::make_unique<RuntimeBootstrap>(backend_mode,
                                                    &joints_,
                                                    &original_joint_limits_,
                                                    &joint_names_,
                                                    joint_state_fetcher);
    bootstrap_->start();

    if (backend_mode == BackendMode::jtc) {
      gzerr << "[xCore Controller] plugin loaded while backend_mode=jtc; "
            << "effort commands will remain disabled and JTC ownership is expected to stay external."
            << std::endl;
    }

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&XCoreControllerPlugin::OnUpdate, this, std::placeholders::_1));
    gzmsg << "[xCore Controller] Plugin loaded successfully, joints: " << joint_num_ << std::endl;
  } catch (const std::exception &ex) {
    gzerr << "[xCore Controller] Load failed with std::exception: " << ex.what() << std::endl;
  } catch (...) {
    gzerr << "[xCore Controller] Load failed with unknown exception" << std::endl;
  }
}

void XCoreControllerPlugin::OnUpdate(const common::UpdateInfo &info) {
  std::unique_lock<std::mutex> update_lock(update_cycle_mutex_);
  if (!model_ || !bootstrap_ || bootstrap_->isShuttingDown() || !bootstrap_->node() || !bootstrap_->backend() ||
      !bootstrap_->controlBridge()) {
    return;
  }

  const double inferred_dt =
      has_last_sim_time_ ? std::max(0.0, (info.simTime - last_sim_time_).Double()) : bootstrap_->trajectorySampleDt();
  last_sim_time_ = info.simTime;
  has_last_sim_time_ = true;
  current_update_dt_ = inferred_dt > 0.0 ? inferred_dt : bootstrap_->trajectorySampleDt();
  initial_pose_initializer_.applyOnce(joints_, joint_num_);

  if (initial_pose_initializer_.initialized()) {
    ExecuteMotion();
  }

  joint_state_cache_.refresh(joints_, joint_num_);

  auto now = bootstrap_->node()->now();
  auto publish_bridge = bootstrap_->publishBridge();
  auto joint_state_pub = bootstrap_->jointStatePublisher();
  auto operation_state_pub = bootstrap_->operationStatePublisher();
  auto runtime_diagnostics_pub = bootstrap_->runtimeDiagnosticsPublisher();

  if (publish_bridge != nullptr && joint_state_pub != nullptr && operation_state_pub != nullptr) {
    static_cast<void>(bootstrap_->collectShutdownContractState(false));
    std::array<double, 6> cached_pos{};
    std::array<double, 6> cached_vel{};
    std::array<double, 6> cached_torque{};
    joint_state_cache_.read(cached_pos, cached_vel, cached_torque);

    runtime::PublisherTickInput tick_input;
    tick_input.stamp = now;
    tick_input.frame_id = "base_link";
    tick_input.joint_names = &joint_names_;
    tick_input.position = cached_pos;
    tick_input.velocity = cached_vel;
    tick_input.torque = cached_torque;
    tick_input.min_publish_period_sec = 0.001;

    const auto publish_tick = publish_bridge->buildPublisherTick(tick_input);
    if (publish_tick.publish_joint_state) {
      joint_state_pub->publish(publish_tick.joint_state);
    }
    if (publish_tick.publish_operation_state) {
      operation_state_pub->publish(publish_tick.operation_state);
    }
    if (publish_tick.publish_operation_state && runtime_diagnostics_pub != nullptr) {
      runtime_diagnostics_pub->publish(publish_bridge->buildRuntimeDiagnosticsMessage());
    }
  }
}

void XCoreControllerPlugin::ExecuteMotion() {
  auto *backend = bootstrap_ ? bootstrap_->backend() : nullptr;
  auto *control_bridge = bootstrap_ ? bootstrap_->controlBridge() : nullptr;
  auto *publish_bridge = bootstrap_ ? bootstrap_->publishBridge() : nullptr;
  auto node = bootstrap_ ? bootstrap_->node() : nullptr;
  if (!backend || !control_bridge) {
    return;
  }

  const auto snapshot = backend->readSnapshot();
  const auto tick_result = control_bridge->tick(*backend, snapshot, current_update_dt_);
  if (publish_bridge != nullptr && node != nullptr) {
    publish_bridge->emitRuntimeStatus(tick_result.status, node->now(), node->get_logger());
  }
}

GZ_REGISTER_MODEL_PLUGIN(XCoreControllerPlugin)

}  // namespace gazebo
