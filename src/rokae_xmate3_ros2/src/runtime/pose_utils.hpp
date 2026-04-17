#ifndef ROKAE_XMATE3_ROS2_RUNTIME_POSE_UTILS_HPP
#define ROKAE_XMATE3_ROS2_RUNTIME_POSE_UTILS_HPP

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rokae_xmate3_ros2::runtime::pose_utils {

inline Eigen::Quaterniond rpyToQuaternion(double rx, double ry, double rz) {
  return Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());
}

inline std::vector<double> identityPose() {
  return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

inline std::vector<double> sanitizePose(const std::vector<double> &pose) {
  if (pose.size() >= 6) {
    return std::vector<double>(pose.begin(), pose.begin() + 6);
  }
  return identityPose();
}

inline Eigen::Isometry3d poseToIsometry(const std::vector<double> &pose) {
  const auto normalized = sanitizePose(pose);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rpyToQuaternion(normalized[3], normalized[4], normalized[5]).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(normalized[0], normalized[1], normalized[2]);
  return transform;
}

inline std::vector<double> isometryToPose(const Eigen::Isometry3d &transform) {
  std::vector<double> pose(6, 0.0);
  pose[0] = transform.translation().x();
  pose[1] = transform.translation().y();
  pose[2] = transform.translation().z();

  const Eigen::Vector3d rpy = transform.linear().eulerAngles(2, 1, 0).reverse();
  for (int i = 0; i < 3; ++i) {
    double angle = rpy[i];
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    pose[3 + i] = angle;
  }
  return pose;
}

inline double angularDistance(const std::vector<double> &lhs, const std::vector<double> &rhs) {
  const auto lhs_pose = sanitizePose(lhs);
  const auto rhs_pose = sanitizePose(rhs);
  const auto lhs_q = rpyToQuaternion(lhs_pose[3], lhs_pose[4], lhs_pose[5]);
  const auto rhs_q = rpyToQuaternion(rhs_pose[3], rhs_pose[4], rhs_pose[5]);
  return lhs_q.angularDistance(rhs_q);
}

inline std::vector<double> convertEndInRefToFlangeInBase(const std::vector<double> &target_end_in_ref,
                                                         const std::vector<double> &tool_pose,
                                                         const std::vector<double> &wobj_pose) {
  const Eigen::Isometry3d base_ref = poseToIsometry(wobj_pose);
  const Eigen::Isometry3d ref_end = poseToIsometry(target_end_in_ref);
  const Eigen::Isometry3d flange_end = poseToIsometry(tool_pose);
  return isometryToPose(base_ref * ref_end * flange_end.inverse());
}

inline std::vector<double> convertFlangeInBaseToEndInRef(const std::vector<double> &flange_in_base,
                                                         const std::vector<double> &tool_pose,
                                                         const std::vector<double> &wobj_pose) {
  const Eigen::Isometry3d base_flange = poseToIsometry(flange_in_base);
  const Eigen::Isometry3d flange_end = poseToIsometry(tool_pose);
  const Eigen::Isometry3d base_ref = poseToIsometry(wobj_pose);
  return isometryToPose(base_ref.inverse() * base_flange * flange_end);
}

}  // namespace rokae_xmate3_ros2::runtime::pose_utils

#endif
