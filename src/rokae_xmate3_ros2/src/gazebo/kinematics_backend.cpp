#include "gazebo/kinematics_backend.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace gazebo::detail {
namespace {

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr std::array<double, 6> kDhA{0.0, 0.0, 0.394, 0.0, 0.0, 0.0};
constexpr std::array<double, 6> kDhAlpha{
    0.0,
    -M_PI / 2.0,
    0.0,
    M_PI / 2.0,
    -M_PI / 2.0,
    M_PI / 2.0,
};
constexpr std::array<double, 6> kDhD{0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};
constexpr std::array<double, 6> kJointOffset{0.0, -M_PI / 2.0, M_PI / 2.0, 0.0, 0.0, 0.0};
constexpr std::array<double, 6> kSmokeReferenceJoints{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
constexpr const char *kPackageName = "rokae_xmate3_ros2";
constexpr const char *kModelRoot = "xMate3_base";
constexpr const char *kModelTip = "xMate3_link6";

Matrix4d legacyDhTransform(std::size_t index, double theta) {
  Matrix4d transform = Matrix4d::Identity();
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(kDhAlpha[index]);
  const double sa = std::sin(kDhAlpha[index]);

  transform << ct, -st, 0.0, kDhA[index], st * ca, ct * ca, -sa, -kDhD[index] * sa, st * sa, ct * sa, ca,
      kDhD[index] * ca, 0.0, 0.0, 0.0, 1.0;
  return transform;
}

std::vector<double> adjustedLegacyJoints(const std::vector<double> &joints) {
  std::vector<double> adjusted = joints;
  if (adjusted.size() >= 3) {
    adjusted[1] -= M_PI / 2.0;
    adjusted[2] += M_PI / 2.0;
  }
  return adjusted;
}

Matrix4d legacyForwardKinematics(const std::vector<double> &joints) {
  const auto adjusted = adjustedLegacyJoints(joints);
  Matrix4d transform = Matrix4d::Identity();
  for (std::size_t index = 0; index < 6 && index < adjusted.size(); ++index) {
    transform *= legacyDhTransform(index, adjusted[index]);
  }
  return transform;
}

std::vector<Matrix4d> legacyAllTransforms(const std::vector<double> &joints) {
  std::vector<Matrix4d> transforms(7, Matrix4d::Identity());
  const auto adjusted = adjustedLegacyJoints(joints);
  for (std::size_t index = 0; index < 6 && index < adjusted.size(); ++index) {
    transforms[index + 1] = transforms[index] * legacyDhTransform(index, adjusted[index]);
  }
  return transforms;
}

Vector6d poseError(const Matrix4d &target, const Matrix4d &current) {
  Vector6d error = Vector6d::Zero();
  error.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

  const Eigen::Matrix3d rotation_delta = target.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose();
  const Eigen::AngleAxisd angle_axis(rotation_delta);
  Eigen::Vector3d orientation_error = angle_axis.angle() * angle_axis.axis();
  if (!std::isfinite(orientation_error.norm()) || angle_axis.angle() < 1e-6) {
    orientation_error << rotation_delta(2, 1) - rotation_delta(1, 2),
        rotation_delta(0, 2) - rotation_delta(2, 0),
        rotation_delta(1, 0) - rotation_delta(0, 1);
    orientation_error *= 0.5;
  }
  error.tail<3>() = orientation_error;
  return error;
}

std::filesystem::path fallbackSourceShareDir() {
  auto path = std::filesystem::path(__FILE__);
  for (int i = 0; i < 3; ++i) {
    path = path.parent_path();
  }
  return path;
}

std::string resolvePackageShareDir() {
  const char *env_share = std::getenv("ROKAE_XMATE3_ROS2_SHARE_DIR");
  if (env_share != nullptr && *env_share != '\0' && std::filesystem::exists(env_share)) {
    return env_share;
  }
  try {
    return ament_index_cpp::get_package_share_directory(kPackageName);
  } catch (const std::exception &) {
    const auto fallback = fallbackSourceShareDir();
    if (std::filesystem::exists(fallback / "urdf" / "xMate3.xacro")) {
      return fallback.string();
    }
  }
  return {};
}

std::string runCommand(const std::string &command) {
  std::string output;
  FILE *pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    return output;
  }
  std::array<char, 4096> buffer{};
  while (true) {
    const auto bytes = std::fread(buffer.data(), 1, buffer.size(), pipe);
    if (bytes == 0) {
      break;
    }
    output.append(buffer.data(), bytes);
  }
  if (pclose(pipe) != 0) {
    return {};
  }
  return output;
}

std::string renderUrdfFromXacro(const std::filesystem::path &xacro_path) {
  if (xacro_path.empty() || !std::filesystem::exists(xacro_path)) {
    return {};
  }
  std::ostringstream command;
  command << "xacro '" << xacro_path.string()
          << "' enable_ros2_control:=false enable_xcore_plugin:=false backend_mode:=hybrid";
  return runCommand(command.str());
}

Matrix4d toEigen(const KDL::Frame &frame) {
  Matrix4d out = Matrix4d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out(row, col) = frame.M(row, col);
    }
    out(row, 3) = frame.p(row);
  }
  return out;
}

std::vector<Matrix4d> computeKdlTransforms(const KDL::Chain &chain, const std::vector<double> &joints) {
  std::vector<Matrix4d> transforms(7, Matrix4d::Identity());
  KDL::Frame pose = KDL::Frame::Identity();
  std::size_t joint_index = 0;
  std::size_t transform_index = 1;
  for (unsigned int segment_index = 0; segment_index < chain.getNrOfSegments(); ++segment_index) {
    const auto &segment = chain.getSegment(segment_index);
    double position = 0.0;
    if (segment.getJoint().getType() != KDL::Joint::Fixed && joint_index < joints.size()) {
      position = joints[joint_index++];
    }
    pose = pose * segment.pose(position);
    if (segment.getJoint().getType() != KDL::Joint::Fixed && transform_index < transforms.size()) {
      transforms[transform_index++] = toEigen(pose);
    }
  }
  return transforms;
}

struct SharedKdlModel {
  bool valid = false;
  std::string reason;
  KDL::Chain chain;
};

bool smokeCheckKdlChain(const KDL::Chain &chain) {
  const std::vector<double> zero_joints(6, 0.0);
  const auto legacy_zero = legacyForwardKinematics(zero_joints);
  const auto kdl_zero = computeKdlTransforms(chain, zero_joints).back();
  const auto zero_error = poseError(legacy_zero, kdl_zero);

  const std::vector<double> reference_joints(kSmokeReferenceJoints.begin(), kSmokeReferenceJoints.end());
  const auto legacy_reference = legacyForwardKinematics(reference_joints);
  const auto kdl_reference = computeKdlTransforms(chain, reference_joints).back();
  const auto reference_error = poseError(legacy_reference, kdl_reference);

  return zero_error.head<3>().norm() < 1e-4 && zero_error.tail<3>().norm() < 1e-3 &&
         reference_error.head<3>().norm() < 1e-3 && reference_error.tail<3>().norm() < 1e-2;
}

SharedKdlModel buildSharedKdlModel() {
  SharedKdlModel model;
  const auto share_dir = resolvePackageShareDir();
  if (share_dir.empty()) {
    model.reason = "package share directory is unavailable";
    return model;
  }

  const auto xacro_path = std::filesystem::path(share_dir) / "urdf" / "xMate3.xacro";
  const auto urdf_xml = renderUrdfFromXacro(xacro_path);
  if (urdf_xml.empty()) {
    model.reason = "failed to render URDF from xacro";
    return model;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    model.reason = "kdl_parser failed to build a tree from the rendered URDF";
    return model;
  }

  if (!tree.getChain(kModelRoot, kModelTip, model.chain)) {
    model.reason = "failed to extract xMate3_base -> xMate3_link6 chain";
    return model;
  }

  if (model.chain.getNrOfJoints() != 6) {
    model.reason = "unexpected joint count in URDF-derived KDL chain";
    return model;
  }

  if (!smokeCheckKdlChain(model.chain)) {
    model.reason = "URDF-derived KDL chain failed legacy DH smoke validation";
    return model;
  }

  model.valid = true;
  model.reason = "ok";
  return model;
}

const SharedKdlModel &sharedKdlModel() {
  static const SharedKdlModel model = buildSharedKdlModel();
  return model;
}

class LegacyKinematicsBackend final : public KinematicsBackend {
 public:
  [[nodiscard]] std::string name() const override { return "legacy"; }

  [[nodiscard]] std::vector<Matrix4d> computeAllTransforms(const std::vector<double> &joints) const override {
    return legacyAllTransforms(joints);
  }

  [[nodiscard]] Matrix6d computeJacobian(const std::vector<double> &joints) const override {
    Matrix6d jacobian = Matrix6d::Zero();
    const Matrix4d current = legacyForwardKinematics(joints);
    constexpr double kDelta = 1e-6;
    for (int axis = 0; axis < 6 && axis < static_cast<int>(joints.size()); ++axis) {
      std::vector<double> joints_plus = joints;
      joints_plus[axis] += kDelta;
      const Matrix4d plus = legacyForwardKinematics(joints_plus);
      jacobian.col(axis) = poseError(plus, current) / kDelta;
    }
    return jacobian;
  }
};

class KdlKinematicsBackend final : public KinematicsBackend {
 public:
  KdlKinematicsBackend() {
    const auto &shared = sharedKdlModel();
    if (!shared.valid) {
      valid_ = false;
      failure_reason_ = shared.reason;
      return;
    }
    chain_ = shared.chain;
    jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    valid_ = jacobian_solver_ != nullptr;
    if (!valid_) {
      failure_reason_ = "failed to construct KDL Jacobian solver";
    }
  }

  [[nodiscard]] std::string name() const override { return valid_ ? "kdl-urdf" : "legacy"; }

  [[nodiscard]] std::vector<Matrix4d> computeAllTransforms(const std::vector<double> &joints) const override {
    if (!valid_) {
      return LegacyKinematicsBackend{}.computeAllTransforms(joints);
    }
    return computeKdlTransforms(chain_, joints);
  }

  [[nodiscard]] Matrix6d computeJacobian(const std::vector<double> &joints) const override {
    if (!valid_ || joints.size() < 6) {
      return LegacyKinematicsBackend{}.computeJacobian(joints);
    }

    KDL::JntArray joint_array(6);
    for (unsigned int index = 0; index < 6; ++index) {
      joint_array(index) = joints[index];
    }
    KDL::Jacobian jacobian(6);
    if (jacobian_solver_->JntToJac(joint_array, jacobian) < 0) {
      return LegacyKinematicsBackend{}.computeJacobian(joints);
    }

    Matrix6d result = Matrix6d::Zero();
    for (unsigned int row = 0; row < 6; ++row) {
      for (unsigned int col = 0; col < 6; ++col) {
        result(row, col) = jacobian(row, col);
      }
    }
    return result;
  }

 private:
  bool valid_ = false;
  std::string failure_reason_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
};

}  // namespace

std::shared_ptr<KinematicsBackend> makePreferredKinematicsBackend() {
  auto backend = std::make_shared<KdlKinematicsBackend>();
  if (backend->name() != "legacy") {
    return backend;
  }
  return std::make_shared<LegacyKinematicsBackend>();
}

}  // namespace gazebo::detail
