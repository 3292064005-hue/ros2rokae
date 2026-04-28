#ifndef ROKAE_XMATE3_ROS2_GAZEBO_INITIAL_POSE_INITIALIZER_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_INITIAL_POSE_INITIALIZER_HPP

#include <vector>

#include <gazebo/common/Console.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class InitialPoseInitializer {
 public:
  explicit InitialPoseInitializer(std::vector<double> default_initial_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      : default_initial_pose_(std::move(default_initial_pose)) {}

  bool applyOnce(std::vector<physics::JointPtr> &joints, int joint_num) {
    if (initialized_ || joint_num <= 0) {
      return false;
    }
    gzmsg << "[xCore Controller] Setting initial joint pose..." << std::endl;
    for (int i = 0; i < joint_num && i < static_cast<int>(default_initial_pose_.size()); ++i) {
      joints[i]->SetPosition(0, default_initial_pose_[i], true);
      joints[i]->SetVelocity(0, 0.0);
      joints[i]->SetForce(0, 0.0);
    }
    initialized_ = true;
    gzmsg << "[xCore Controller] Initial pose set successfully!" << std::endl;
    return true;
  }

  [[nodiscard]] bool initialized() const { return initialized_; }

 private:
  bool initialized_ = false;
  std::vector<double> default_initial_pose_;
};

}  // namespace gazebo

#endif
