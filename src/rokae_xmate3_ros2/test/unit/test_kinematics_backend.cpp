#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

// ============================================================================
// Forward Kinematics Tests
// ============================================================================

TEST(KinematicsBackendTest, ForwardKinematicsZeroConfigIsValid) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> zero_joints(6, 0.0);
  const auto pose = kinematics.forwardKinematicsRPY(zero_joints);

  ASSERT_EQ(pose.size(), 6u);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(pose[i])) << "Pose element " << i << " is not finite";
  }
  // At zero configuration, the end-effector should be along the Z axis
  EXPECT_NEAR(pose[0], 0.0, 0.01);  // x ≈ 0
  EXPECT_NEAR(pose[1], 0.0, 0.01);  // y ≈ 0
  EXPECT_GT(pose[2], 0.3);          // z > 0.3 (robot reaches upward)
}

TEST(KinematicsBackendTest, ForwardKinematicsHomogeneousMatrixIsValid) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};
  const auto T = kinematics.forwardKinematics(joints);

  // Bottom row of a homogeneous transform must be [0 0 0 1]
  EXPECT_NEAR(T(3, 0), 0.0, 1e-12);
  EXPECT_NEAR(T(3, 1), 0.0, 1e-12);
  EXPECT_NEAR(T(3, 2), 0.0, 1e-12);
  EXPECT_NEAR(T(3, 3), 1.0, 1e-12);

  // Rotation sub-matrix must be orthonormal: R^T * R = I
  const auto R = T.block<3, 3>(0, 0);
  const Eigen::Matrix3d RtR = R.transpose() * R;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(RtR(i, j), (i == j) ? 1.0 : 0.0, 1e-6)
          << "R^T*R[" << i << "," << j << "] deviation";
    }
  }
}

TEST(KinematicsBackendTest, ForwardKinematicsRPYConsistentWithMatrix) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.1, -0.3, 0.5, 0.2, -0.8, 1.0};
  const auto T = kinematics.forwardKinematics(joints);
  const auto rpy_pose = kinematics.forwardKinematicsRPY(joints);

  ASSERT_EQ(rpy_pose.size(), 6u);
  // Position from matrix must match RPY pose
  EXPECT_NEAR(rpy_pose[0], T(0, 3), 1e-6);
  EXPECT_NEAR(rpy_pose[1], T(1, 3), 1e-6);
  EXPECT_NEAR(rpy_pose[2], T(2, 3), 1e-6);
}

TEST(KinematicsBackendTest, ForwardKinematicsMultipleConfigurationsAllFinite) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<std::vector<double>> configs = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {1.0, -1.0, 1.0, -1.0, 1.0, -1.0},
      {-2.0, 1.5, -1.5, 2.0, -1.5, 3.0},
      {3.05, 2.09, 2.09, 3.05, 2.09, 6.10},  // near joint limits
  };
  for (const auto &joints : configs) {
    const auto pose = kinematics.forwardKinematicsRPY(joints);
    ASSERT_EQ(pose.size(), 6u);
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_TRUE(std::isfinite(pose[i]))
          << "Non-finite pose[" << i << "] for config";
    }
  }
}

// ============================================================================
// Inverse Kinematics Tests
// ============================================================================

TEST(KinematicsBackendTest, InverseKinematicsRoundTrip) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> original_joints = {0.1, 0.2, 1.0, -0.1, 0.8, 0.5};
  const auto target_pose = kinematics.forwardKinematicsRPY(original_joints);

  const auto solved_joints = kinematics.inverseKinematics(target_pose, original_joints);
  ASSERT_EQ(solved_joints.size(), 6u);

  // FK of solved joints should match the target pose
  const auto recovered_pose = kinematics.forwardKinematicsRPY(solved_joints);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(recovered_pose[i], target_pose[i], 1e-4)
        << "Position mismatch at index " << i;
  }
}

TEST(KinematicsBackendTest, InverseKinematicsSeededFastRoundTrip) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> seed = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};
  const auto target_pose = kinematics.forwardKinematicsRPY(seed);

  const auto result = kinematics.inverseKinematicsSeededFast(target_pose, seed);
  ASSERT_EQ(result.size(), 6u);

  const auto recovered = kinematics.forwardKinematicsRPY(result);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(recovered[i], target_pose[i], 1e-4)
        << "Seeded fast IK position mismatch at index " << i;
  }
}

TEST(KinematicsBackendTest, InverseKinematicsMultiSolutionReturnsMultipleCandidates) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};
  const auto target_pose = kinematics.forwardKinematicsRPY(joints);

  const auto solutions = kinematics.inverseKinematicsMultiSolution(target_pose, joints);
  EXPECT_GE(solutions.size(), 1u) << "Expected at least one IK solution";

  // Each solution should produce a valid FK close to the target
  for (const auto &sol : solutions) {
    ASSERT_EQ(sol.size(), 6u);
    const auto sol_pose = kinematics.forwardKinematicsRPY(sol);
    for (size_t i = 0; i < 3; ++i) {
      EXPECT_NEAR(sol_pose[i], target_pose[i], 1e-3)
          << "Multi-solution FK mismatch at position index " << i;
    }
  }
}

// ============================================================================
// Jacobian Tests
// ============================================================================

TEST(KinematicsBackendTest, JacobianIsFinite) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.2, -0.1, 0.8, 0.3, -0.5, 1.5};
  const auto J = kinematics.computeJacobian(joints);

  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_TRUE(std::isfinite(J(i, j)))
          << "Jacobian[" << i << "," << j << "] is not finite";
    }
  }
}

TEST(KinematicsBackendTest, JacobianNumericalConsistency) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};
  const auto J = kinematics.computeJacobian(joints);

  // Verify Jacobian via numerical differentiation: dFK/dq ≈ J column
  constexpr double kEps = 1e-6;
  const auto base_T = kinematics.forwardKinematics(joints);

  for (int col = 0; col < 6; ++col) {
    auto perturbed = joints;
    perturbed[col] += kEps;
    const auto perturbed_T = kinematics.forwardKinematics(perturbed);

    // Linear velocity part (rows 0-2): dp/dq
    for (int row = 0; row < 3; ++row) {
      const double numerical = (perturbed_T(row, 3) - base_T(row, 3)) / kEps;
      EXPECT_NEAR(J(row, col), numerical, 0.05)
          << "Jacobian linear velocity mismatch at (" << row << "," << col << ")";
    }
  }
}

// ============================================================================
// Singularity Tests
// ============================================================================

TEST(KinematicsBackendTest, SingularityMeasureIsNormalized) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};
  const double measure = kinematics.computeSingularityMeasure(joints);

  EXPECT_GE(measure, 0.0);
  EXPECT_LE(measure, 1.0);
}

TEST(KinematicsBackendTest, IsNearSingularityReturnsConsistentMeasure) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> nominal_joints = {0.1, 0.5, 1.0, 0.1, 0.8, 0.5};
  const bool near_singular = kinematics.isNearSingularity(nominal_joints);
  const double measure = kinematics.computeSingularityMeasure(nominal_joints);

  // If not near singularity, measure should be below a reasonable threshold
  if (!near_singular) {
    EXPECT_LT(measure, 0.95);
  }
}

// ============================================================================
// IK Candidate Evaluation Tests
// ============================================================================

TEST(KinematicsBackendTest, IkCandidateMetricsSelfSeedHasBestCost) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};

  const auto metrics = kinematics.evaluateIkCandidate(joints, joints);
  EXPECT_TRUE(metrics.valid);
  EXPECT_NEAR(metrics.branch_distance, 0.0, 1e-9);
  EXPECT_NEAR(metrics.continuity_cost, 0.0, 1e-9);
}

// ============================================================================
// Debug Counters Tests
// ============================================================================

TEST(KinematicsBackendTest, DebugCountersIncrement) {
  gazebo::xMate3Kinematics kinematics;
  kinematics.resetDebugCounters();

  const std::vector<double> joints = {0.0, 0.15, 1.55, 0.0, 1.35, M_PI};
  kinematics.computeJacobian(joints);
  kinematics.computeJacobian(joints);

  const auto counters = kinematics.debugCounters();
  EXPECT_GE(counters.jacobian_calls, 2u);
}

TEST(KinematicsBackendTest, DebugCountersResetToZero) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  kinematics.computeJacobian(joints);

  kinematics.resetDebugCounters();
  const auto counters = kinematics.debugCounters();
  EXPECT_EQ(counters.jacobian_calls, 0u);
  EXPECT_EQ(counters.svd_calls, 0u);
}

// ============================================================================
// Backend Name Test
// ============================================================================

TEST(KinematicsBackendTest, BackendNameIsNonNull) {
  gazebo::xMate3Kinematics kinematics;
  const auto name = kinematics.backendName();
  ASSERT_NE(name, nullptr);
  EXPECT_GT(std::string(name).size(), 0u);
}
