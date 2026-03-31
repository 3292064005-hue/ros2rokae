#ifndef ROKAE_XMATE3_ROS2_GAZEBO_XCORE_CONTROLLER_GAZEBO_PLUGIN_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_XCORE_CONTROLLER_GAZEBO_PLUGIN_HPP

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include "gazebo/initial_pose_initializer.hpp"
#include "gazebo/joint_state_cache.hpp"
#include "gazebo/runtime_bootstrap.hpp"

namespace gazebo {

class XCoreControllerPlugin : public ModelPlugin {
 public:
  XCoreControllerPlugin() = default;
  ~XCoreControllerPlugin() override;

 void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  void OnUpdate(const common::UpdateInfo &info);
  void ExecuteMotion();

  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  std::vector<physics::JointPtr> joints_;
  std::vector<std::string> joint_names_;
  int joint_num_ = 0;
  event::ConnectionPtr update_conn_;

  std::array<std::pair<double, double>, 6> original_joint_limits_{};
  double current_update_dt_ = kDefaultTrajectorySampleDt;
  common::Time last_sim_time_{};
  bool has_last_sim_time_ = false;
  std::mutex update_cycle_mutex_;

  JointStateCache joint_state_cache_;
  InitialPoseInitializer initial_pose_initializer_;
  std::unique_ptr<RuntimeBootstrap> bootstrap_;
};

}  // namespace gazebo

#endif
