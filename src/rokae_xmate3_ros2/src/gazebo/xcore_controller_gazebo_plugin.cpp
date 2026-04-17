#include "gazebo/xcore_controller_gazebo_plugin.hpp"

#include <algorithm>
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
    std::string backend_mode_value = "jtc";
    if (sdf_ && sdf_->HasElement("backend_mode")) {
      backend_mode_value = sdf_->Get<std::string>("backend_mode");
    }
    const auto backend_mode = parseBackendMode(backend_mode_value);
    std::string service_exposure_profile_value = "public_xmate6_only";
    if (sdf_ && sdf_->HasElement("service_exposure_profile")) {
      service_exposure_profile_value = sdf_->Get<std::string>("service_exposure_profile");
    }
    gzmsg << "[xCore Controller] backend_mode=" << toString(backend_mode)
          << " service_exposure_profile=" << service_exposure_profile_value << std::endl;

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

    RuntimeBootstrap::RosIntegrationOptions ros_integration;
    ros_integration.parameter_overrides.emplace_back("service_exposure_profile", service_exposure_profile_value);
    bootstrap_ = std::make_unique<RuntimeBootstrap>(backend_mode,
                                                    &joints_,
                                                    &original_joint_limits_,
                                                    &joint_names_,
                                                    joint_state_fetcher,
                                                    std::move(ros_integration));
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
  const double nominal_dt = bootstrap_->trajectorySampleDt();
  double resolved_dt = inferred_dt;
  if (resolved_dt <= 0.0 || resolved_dt > 0.01) {
    resolved_dt = nominal_dt;
  }
  if (bootstrap_->rtProfileConfig().authoritative_servo_clock) {
    // Keep servo clock deterministic under RT-oriented profiles.
    resolved_dt = nominal_dt;
  } else {
    resolved_dt = std::clamp(resolved_dt, 0.0005, 0.005);
  }
  current_update_dt_ = resolved_dt;
  initial_pose_initializer_.applyOnce(joints_, joint_num_);

  if (initial_pose_initializer_.initialized()) {
    ExecuteMotion();
  }

  joint_state_cache_.refresh(joints_, joint_num_);

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
