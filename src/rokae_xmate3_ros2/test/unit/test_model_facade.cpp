#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "rokae_xmate3_ros2/gazebo/model_facade.hpp"

namespace gm = rokae_xmate3_ros2::gazebo_model;

class ModelFacadeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    kinematics_ = std::make_unique<gazebo::xMate3Kinematics>();
    facade_ = std::make_unique<gm::ModelFacade>(*kinematics_);
  }

  std::unique_ptr<gazebo::xMate3Kinematics> kinematics_;
  std::unique_ptr<gm::ModelFacade> facade_;

  // Standard test configurations
  const std::array<double, 6> zero_joints_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 6> nominal_joints_{0.1, 0.2, 1.0, -0.1, 0.8, 0.5};
  const std::array<double, 6> zero_velocity_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::array<double, 6> low_velocity_{0.01, -0.02, 0.03, -0.01, 0.02, -0.01};
};

// ============================================================================
// Cartesian Pose Tests
// ============================================================================

TEST_F(ModelFacadeTest, CartPoseZeroConfigIsFinite) {
  const auto pose = facade_->cartPose<6>(zero_joints_);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(pose[i]))
        << "CartPose[" << i << "] is not finite at zero config";
  }
}

TEST_F(ModelFacadeTest, CartPoseNominalConfigIsFinite) {
  const auto pose = facade_->cartPose<6>(nominal_joints_);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(pose[i]))
        << "CartPose[" << i << "] is not finite at nominal config";
  }
}

TEST_F(ModelFacadeTest, CartPoseWithToolOffsetDiffersFromNoTool) {
  const auto no_tool_pose = facade_->cartPose<6>(nominal_joints_);

  const std::array<double, 6> tool_pose = {0.0, 0.0, 0.1, 0.0, 0.0, 0.0};  // 100mm Z offset
  facade_->setToolPose(tool_pose);
  const auto tool_applied_pose = facade_->cartPose<6>(nominal_joints_);

  // With a tool offset, at least the position should differ
  double position_diff = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    position_diff += std::fabs(tool_applied_pose[i] - no_tool_pose[i]);
  }
  EXPECT_GT(position_diff, 0.01) << "Tool offset had no effect on cartesian pose";
}

// ============================================================================
// Cartesian Velocity Tests
// ============================================================================

TEST_F(ModelFacadeTest, CartVelocityZeroJointVelocityIsZero) {
  const auto cart_vel = facade_->cartVelocity<6>(nominal_joints_, zero_velocity_);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(cart_vel[i], 0.0, 1e-9)
        << "Non-zero cartesian velocity at index " << i << " with zero joint velocity";
  }
}

TEST_F(ModelFacadeTest, CartVelocityNonZeroJointVelocityProducesNonZeroResult) {
  const auto cart_vel = facade_->cartVelocity<6>(nominal_joints_, low_velocity_);

  double norm = 0.0;
  for (size_t i = 0; i < 6; ++i) {
    norm += cart_vel[i] * cart_vel[i];
    EXPECT_TRUE(std::isfinite(cart_vel[i]));
  }
  EXPECT_GT(norm, 0.0) << "Cartesian velocity is zero despite non-zero joint velocity";
}

// ============================================================================
// Jacobian Tests
// ============================================================================

TEST_F(ModelFacadeTest, JacobianIsFinite) {
  const auto J = facade_->jacobian<6>(nominal_joints_);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_TRUE(std::isfinite(J(i, j)))
          << "Jacobian[" << i << "," << j << "] is not finite";
    }
  }
}

TEST_F(ModelFacadeTest, JacobianIsNonSingularAtNominal) {
  const auto J = facade_->jacobian<6>(nominal_joints_);
  const double det = J.determinant();
  EXPECT_GT(std::fabs(det), 1e-10)
      << "Jacobian is singular at nominal configuration";
}

// ============================================================================
// Mass Matrix Tests
// ============================================================================

TEST_F(ModelFacadeTest, MassMatrixIsSymmetric) {
  const auto M = facade_->massMatrix<6>(nominal_joints_);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(M(i, j), M(j, i), 1e-12)
          << "Mass matrix not symmetric at (" << i << "," << j << ")";
    }
  }
}

TEST_F(ModelFacadeTest, MassMatrixDiagonalIsPositive) {
  const auto M = facade_->massMatrix<6>(nominal_joints_);
  for (int i = 0; i < 6; ++i) {
    EXPECT_GT(M(i, i), 0.0)
        << "Mass matrix diagonal element " << i << " is non-positive";
  }
}

// ============================================================================
// Dynamics Breakdown Tests
// ============================================================================

TEST_F(ModelFacadeTest, DynamicsBreakdownSumsToFull) {
  const std::array<double, 6> joint_acc = {0.1, -0.2, 0.3, -0.1, 0.2, -0.1};
  const std::array<double, 6> ext_force = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  const auto breakdown = facade_->dynamics<6>(nominal_joints_, low_velocity_, joint_acc, ext_force);

  for (size_t i = 0; i < 6; ++i) {
    const double expected_full = breakdown.gravity[i] + breakdown.coriolis[i] +
                                  breakdown.inertia[i] + breakdown.external[i];
    EXPECT_NEAR(breakdown.full[i], expected_full, 1e-9)
        << "Dynamics breakdown sum mismatch at joint " << i;
  }
}

TEST_F(ModelFacadeTest, GravityIsNonZeroAtNominalConfig) {
  const auto grav = facade_->gravity<6>(nominal_joints_);

  double norm = 0.0;
  for (double val : grav) {
    norm += std::fabs(val);
  }
  EXPECT_GT(norm, 0.0) << "Gravity torque is zero at non-zero configuration";
}

TEST_F(ModelFacadeTest, CoriolisIsZeroAtZeroVelocity) {
  const auto coriolis = facade_->coriolis<6>(nominal_joints_, zero_velocity_);
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(coriolis[i], 0.0, 1e-9)
        << "Coriolis non-zero at zero velocity, joint " << i;
  }
}

// ============================================================================
// Inverse Dynamics Tests
// ============================================================================

TEST_F(ModelFacadeTest, InverseDynamicsIsFinite) {
  const std::array<double, 6> acc = {0.5, -0.3, 0.2, 0.1, -0.4, 0.3};
  const auto torques = facade_->inverseDynamics<6>(
      nominal_joints_, low_velocity_, acc);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(torques[i]))
        << "Inverse dynamics torque[" << i << "] is not finite";
  }
}

// ============================================================================
// Expected Torque Tests
// ============================================================================

TEST_F(ModelFacadeTest, ExpectedTorqueMatchesGravityAtZeroVelocity) {
  const auto expected = facade_->expectedTorque(nominal_joints_, zero_velocity_);
  const auto grav = facade_->gravity<6>(nominal_joints_);

  // At zero velocity, expected torque should approximately equal gravity
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(expected[i], grav[i], 1e-3)
        << "Expected torque differs from gravity at zero velocity, joint " << i;
  }
}

// ============================================================================
// Load and Tool Configuration Tests
// ============================================================================

TEST_F(ModelFacadeTest, SetLoadAffectsMassMatrix) {
  const auto M_no_load = facade_->massMatrix<6>(nominal_joints_);

  gm::LoadContext load;
  load.mass = 5.0;
  load.com = {0.0, 0.0, 0.05};
  facade_->setLoad(load);

  const auto M_with_load = facade_->massMatrix<6>(nominal_joints_);

  // With additional load, diagonal elements should increase
  bool any_increased = false;
  for (int i = 0; i < 6; ++i) {
    if (M_with_load(i, i) > M_no_load(i, i) + 1e-9) {
      any_increased = true;
      break;
    }
  }
  EXPECT_TRUE(any_increased) << "Adding load did not increase any mass matrix diagonal";
}

TEST_F(ModelFacadeTest, LoadRoundTrip) {
  gm::LoadContext load;
  load.mass = 2.5;
  load.com = {0.01, 0.02, 0.03};
  facade_->setLoad(load);

  const auto readback = facade_->load();
  EXPECT_NEAR(readback.mass, 2.5, 1e-9);
  EXPECT_NEAR(readback.com[0], 0.01, 1e-9);
  EXPECT_NEAR(readback.com[1], 0.02, 1e-9);
  EXPECT_NEAR(readback.com[2], 0.03, 1e-9);
}

TEST_F(ModelFacadeTest, ToolPoseRoundTrip) {
  const std::array<double, 6> tool = {0.0, 0.0, 0.15, 0.0, 0.0, 0.0};
  facade_->setToolPose(tool);

  const auto readback = facade_->toolPose();
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(readback[i], tool[i], 1e-9);
  }
}

// ============================================================================
// Model Observability Tests
// ============================================================================

TEST_F(ModelFacadeTest, ObservabilityReportsApproximateFlags) {
  const auto obs = facade_->observability();
  EXPECT_TRUE(obs.uses_approximate_jacobian);
  EXPECT_TRUE(obs.uses_simplified_inertia);
}

TEST_F(ModelFacadeTest, ObservabilityEffectivePayloadMatchesLoad) {
  gm::LoadContext load;
  load.mass = 3.0;
  facade_->setLoad(load);

  const auto obs = facade_->observability();
  EXPECT_NEAR(obs.effective_payload, 3.0, 1e-9);
}

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST(ModelFacadeUtilTest, MeanAbsIsCorrect) {
  const std::array<double, 6> values = {1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
  const double expected = (1.0 + 2.0 + 3.0 + 4.0 + 5.0 + 6.0) / 6.0;
  EXPECT_NEAR(gm::meanAbs(values), expected, 1e-9);
}

TEST(ModelFacadeUtilTest, ComNormIsCorrect) {
  gm::LoadContext load;
  load.com = {3.0, 4.0, 0.0};
  EXPECT_NEAR(gm::comNorm(load), 5.0, 1e-9);
}

TEST(ModelFacadeUtilTest, PoseMatrixRoundTrip) {
  const std::array<double, 6> pose = {0.5, -0.3, 1.2, 0.1, 0.2, -0.3};
  const auto matrix = gm::poseToMatrix(pose);
  const auto recovered = gm::matrixToPose(matrix);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(recovered[i], pose[i], 1e-6)
        << "Pose<->Matrix round-trip mismatch at index " << i;
  }
}

TEST(ModelFacadeUtilTest, ToVector6RoundTrip) {
  const std::array<double, 6> arr = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const auto vec = gm::toVector6(arr);
  const auto recovered = gm::toArray6(vec);

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(recovered[i], arr[i]);
  }
}

TEST(ModelFacadeUtilTest, MakeModelFacadeFactoryFunction) {
  gazebo::xMate3Kinematics kin;
  const auto facade = gm::makeModelFacade(kin);
  const auto pose = facade.cartPose<6>(std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  for (size_t i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isfinite(pose[i]));
  }
}
