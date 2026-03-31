#ifndef ROKAE_XMATE3_ROS2_GAZEBO_JOINT_STATE_CACHE_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_JOINT_STATE_CACHE_HPP

#include <array>
#include <mutex>
#include <vector>

#include <gazebo/physics/physics.hh>

namespace gazebo {

class JointStateCache {
 public:
  void refresh(const std::vector<physics::JointPtr> &joints, int joint_num) {
    std::array<double, 6> position{};
    std::array<double, 6> velocity{};
    std::array<double, 6> torque{};
    for (int i = 0; i < 6 && i < joint_num; ++i) {
      position[i] = joints[i]->Position(0);
      velocity[i] = joints[i]->GetVelocity(0);
      torque[i] = joints[i]->GetForce(0);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = position;
    velocity_ = velocity;
    torque_ = torque;
  }

  void read(std::array<double, 6> &position,
            std::array<double, 6> &velocity,
            std::array<double, 6> &torque) const {
    std::lock_guard<std::mutex> lock(mutex_);
    position = position_;
    velocity = velocity_;
    torque = torque_;
  }

 private:
  mutable std::mutex mutex_;
  std::array<double, 6> position_{};
  std::array<double, 6> velocity_{};
  std::array<double, 6> torque_{};
};

}  // namespace gazebo

#endif
