#ifndef ROKAE_XMATE3_ROS2_GAZEBO_KINEMATICS_BACKEND_HPP
#define ROKAE_XMATE3_ROS2_GAZEBO_KINEMATICS_BACKEND_HPP

#include <array>
#include <limits>
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
  using VectorJ = std::vector<double>;

  struct SeededIkRequest {
    Matrix4d target_transform = Matrix4d::Identity();
    VectorJ seed_joints;
    VectorJ joint_limits_min;
    VectorJ joint_limits_max;
    int max_iter = 16;
    double position_tolerance = 1e-5;
    double orientation_tolerance = 1e-3;
    double orientation_weight = 0.8;
    double max_joint_step = 0.05;
    double min_lambda = 0.02;
    double max_lambda = 0.5;
    double solution_valid_threshold = 5e-3;
    double wrist_singularity_threshold = 0.15;
    double jacobian_singularity_threshold = 0.08;
    double singularity_avoidance_offset = 0.2;
    double continuity_weight = 0.02;
    double joint_limit_weight = 0.01;
    double singularity_weight = 0.1;
  };
  using IKRequest = SeededIkRequest;

  struct SeededIkCandidateMetrics {
    double branch_distance = std::numeric_limits<double>::infinity();
    double continuity_cost = std::numeric_limits<double>::infinity();
    double joint_limit_penalty = std::numeric_limits<double>::infinity();
    double singularity_metric = 1.0;
    double singularity_cost = std::numeric_limits<double>::infinity();
    bool near_singularity = true;
    bool valid = false;

    [[nodiscard]] double totalCost() const noexcept {
      return continuity_cost + joint_limit_penalty + singularity_cost;
    }
  };

  struct IKCandidate {
    VectorJ joints;
    double pose_error_score = std::numeric_limits<double>::infinity();
    double branch_distance = std::numeric_limits<double>::infinity();
    double continuity_cost = std::numeric_limits<double>::infinity();
    double singularity_metric = 1.0;
    double joint_limit_penalty = std::numeric_limits<double>::infinity();
    double conf_penalty = 0.0;
    double total_cost = std::numeric_limits<double>::infinity();
    bool valid = false;
    std::string branch_id;
    std::string message;
  };

  struct CartesianIkOptions {
    std::vector<int> requested_conf;
    bool strict_conf = false;
    bool avoid_singularity = true;
    bool soft_limit_enabled = false;
    std::array<std::array<double, 2>, 6> soft_limits{{
        {{-3.14, 3.14}},
        {{-3.14, 3.14}},
        {{-3.14, 3.14}},
        {{-3.14, 3.14}},
        {{-3.14, 3.14}},
        {{-3.14, 3.14}},
    }};
    VectorJ joint_limits_min;
    VectorJ joint_limits_max;
    double branch_jump_threshold = 0.75;
  };

  struct IkScoringConfig {
    double pose_error_weight = 1.0;
    double branch_distance_weight = 2.0;
    double conf_penalty_weight = 1.0;
    double singularity_weight = 40.0;
    double joint_limit_weight = 10.0;
    double branch_switch_penalty = 2.0;
    double wrist_flip_penalty = 4.0;
    double hard_wrist_singularity_threshold = 0.075;
    double hard_jacobian_singularity_threshold = 0.04;
  };

  struct CartesianIkSelectionResult {
    bool success = false;
    VectorJ joints;
    std::string message;
  };

  struct IKResult {
    bool success = false;
    VectorJ joints;
    std::vector<IKCandidate> candidates;
    std::string chosen_branch;
    double singularity_metric = 1.0;
    std::string message;
  };

  struct IKTrajectoryRequest {
    std::vector<Matrix4d> target_transforms;
    VectorJ initial_seed;
    CartesianIkOptions options;
    std::size_t dp_window = 32;
    std::size_t dp_overlap = 8;
    std::size_t max_candidates_per_point = 8;
  };

  struct IKTrajectoryResult {
    bool success = false;
    std::vector<VectorJ> q_path;
    std::vector<double> singularity_metric_path;
    std::vector<std::string> chosen_branch_path;
    std::size_t failed_index = std::numeric_limits<std::size_t>::max();
    std::string message;
  };

  virtual ~KinematicsBackend() = default;

  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual Matrix4d computeForwardTransform(const std::vector<double> &joints) const = 0;
  [[nodiscard]] virtual std::vector<Matrix4d> computeAllTransforms(const std::vector<double> &joints) const = 0;
  [[nodiscard]] virtual Matrix6d computeJacobian(const std::vector<double> &joints) const = 0;
  [[nodiscard]] virtual Matrix4d computeFrameTransform(const std::vector<double> &joints, std::size_t frame_index) const {
    const auto transforms = computeAllTransforms(joints);
    if (transforms.empty()) {
      return Matrix4d::Identity();
    }
    const auto resolved_index = std::min(frame_index, transforms.size() - 1);
    return transforms[resolved_index];
  }
  [[nodiscard]] virtual double branchDistance(const VectorJ &lhs, const VectorJ &rhs) const = 0;
  [[nodiscard]] virtual double continuityCost(const SeededIkRequest &request, const VectorJ &candidate) const = 0;
  [[nodiscard]] virtual double singularityMetric(const SeededIkRequest &request, const VectorJ &candidate) const = 0;
  [[nodiscard]] virtual SeededIkCandidateMetrics evaluateSeededIkCandidate(const SeededIkRequest &request,
                                                                           const VectorJ &candidate) const = 0;
  [[nodiscard]] virtual IKResult solveSeeded(const IKRequest &request) const = 0;
  [[nodiscard]] virtual IKResult solveMultiBranch(const IKRequest &request) const = 0;
  [[nodiscard]] virtual IKTrajectoryResult solveTrajectory(const IKTrajectoryRequest &request) const = 0;
  [[nodiscard]] virtual VectorJ inverseKinematicsSeeded(const SeededIkRequest &request) const = 0;
  [[nodiscard]] virtual std::vector<VectorJ> inverseKinematicsMultiBranch(const SeededIkRequest &request) const = 0;
  [[nodiscard]] virtual CartesianIkSelectionResult selectBestCartesianIkSolution(
      const std::vector<VectorJ> &candidates,
      const Matrix4d &target_transform,
      const VectorJ &seed_joints,
      const CartesianIkOptions &options) const = 0;
  [[nodiscard]] virtual bool buildCartesianJointTrajectory(
      const std::vector<Matrix4d> &cartesian_transforms,
      const VectorJ &initial_seed,
      const CartesianIkOptions &options,
      std::vector<VectorJ> &joint_trajectory,
      VectorJ &last_joints,
      std::string &error_message) const = 0;
  [[nodiscard]] virtual bool projectCartesianJointDerivatives(
      const std::vector<Matrix4d> &cartesian_transforms,
      const std::vector<VectorJ> &joint_trajectory,
      double trajectory_dt,
      std::vector<VectorJ> &joint_velocity_trajectory,
      std::vector<VectorJ> &joint_acceleration_trajectory) const = 0;
};

[[nodiscard]] std::shared_ptr<KinematicsBackend> makePreferredKinematicsBackend();

}  // namespace detail
}  // namespace gazebo

#endif
