#ifndef ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_CORE_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_PLANNER_CORE_HPP

#include <memory>

#include "runtime/runtime_types.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rokae_xmate3_ros2::runtime {

class MotionPlanner {
 public:
  MotionPlanner();

  [[nodiscard]] MotionPlan plan(const MotionRequest &request) const;

 private:
  std::unique_ptr<::gazebo::xMate3Kinematics> kinematics_;
};

}  // namespace rokae_xmate3_ros2::runtime

#endif
