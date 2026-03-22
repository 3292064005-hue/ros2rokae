#ifndef ROKAE_XMATE3_ROS2_GAZEBO_KINEMATICS_BACKEND_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_KINEMATICS_BACKEND_HPP

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace gazebo {

class xMate3Kinematics;

namespace detail {

class KinematicsBackend {
 public:
  using Matrix4d = Eigen::Matrix4d;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  virtual ~KinematicsBackend() = default;

  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual std::vector<Matrix4d> computeAllTransforms(const std::vector<double> &joints) const = 0;
  [[nodiscard]] virtual Matrix6d computeJacobian(const std::vector<double> &joints) const = 0;
};

[[nodiscard]] std::shared_ptr<KinematicsBackend> makePreferredKinematicsBackend();

}  // namespace detail
}  // namespace gazebo

#endif
