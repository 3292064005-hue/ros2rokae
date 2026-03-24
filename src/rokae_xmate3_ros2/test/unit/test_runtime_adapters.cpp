#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "rokae/model.h"
#include "rokae_xmate3_ros2/gazebo/approximate_model.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"
#include "runtime/operation_state_adapter.hpp"
#include "runtime/request_adapter.hpp"
#include "runtime/runtime_state.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

namespace {

using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr std::array<double, 6> kLegacyDhA{0.0, 0.0, 0.394, 0.0, 0.0, 0.0};
constexpr std::array<double, 6> kLegacyDhAlpha{
    0.0,
    -M_PI / 2.0,
    0.0,
    M_PI / 2.0,
    -M_PI / 2.0,
    M_PI / 2.0,
};
constexpr std::array<double, 6> kLegacyDhD{0.3415, 0.0, 0.0, 0.366, 0.0, 0.2503};

Matrix4d legacyDhTransform(std::size_t index, double theta) {
  Matrix4d transform = Matrix4d::Identity();
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(kLegacyDhAlpha[index]);
  const double sa = std::sin(kLegacyDhAlpha[index]);
  transform << ct, -st, 0.0, kLegacyDhA[index], st * ca, ct * ca, -sa, -kLegacyDhD[index] * sa, st * sa,
      ct * sa, ca, kLegacyDhD[index] * ca, 0.0, 0.0, 0.0, 1.0;
  return transform;
}

std::vector<double> adjustedLegacyJoints(const std::vector<double> &joints) {
  auto adjusted = joints;
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

Vector6d legacyPoseError(const Matrix4d &target, const Matrix4d &current) {
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

Matrix6d legacyFiniteDifferenceJacobian(const std::vector<double> &joints) {
  Matrix6d jacobian = Matrix6d::Zero();
  const Matrix4d current = legacyForwardKinematics(joints);
  constexpr double kDelta = 1e-6;
  for (int axis = 0; axis < 6; ++axis) {
    auto joints_plus = joints;
    joints_plus[axis] += kDelta;
    const Matrix4d plus = legacyForwardKinematics(joints_plus);
    jacobian.col(axis) = legacyPoseError(plus, current) / kDelta;
  }
  return jacobian;
}

std::string shellQuote(const std::string &value) {
  std::string quoted = "'";
  for (char ch : value) {
    if (ch == '\'') {
      quoted += "'\\''";
    } else {
      quoted.push_back(ch);
    }
  }
  quoted.push_back('\'');
  return quoted;
}

void writeTextFile(const std::filesystem::path &path, const std::string &content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path);
  ASSERT_TRUE(stream.good()) << "failed to open " << path;
  stream << content;
  ASSERT_TRUE(stream.good()) << "failed to write " << path;
}

std::string readTextFile(const std::filesystem::path &path) {
  std::ifstream stream(path);
  EXPECT_TRUE(stream.good()) << "failed to open " << path;
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

void createArchiveFixtureTree(const std::filesystem::path &root, bool dirty_archive) {
  writeTextFile(root / ".gitignore", "build/\nlog/\n__pycache__/\n*.pyc\n");
  writeTextFile(root / "src/rokae_xmate3_ros2/src/runtime/owner_arbiter.hpp",
                "#pragma once\n// archive fixture\n");
  writeTextFile(root / "src/rokae_xmate3_ros2/urdf/xMate3.xacro",
                R"(<robot name="xmate3">
  <link name="flange"/>
  <link name="tool0"/>
  <link name="tcp"/>
  <link name="payload"/>
</robot>
)");
  if (dirty_archive) {
    writeTextFile(root / "src/rokae_xmate3_ros2/test/harness/__pycache__/stale.cpython-312.pyc", "pyc");
    writeTextFile(root / "build/generated.cache", "build artifact");
  }
}

int runShellCommand(const std::string &command) {
  return std::system(command.c_str());
}

}  // namespace

TEST(RuntimeRequestAdapterTest, BuildsMoveAppendRequestWithOffsetsAndDefaults) {
  rokae_xmate3_ros2::action::MoveAppend::Goal goal;
  goal.l_cmds.resize(1);
  auto &line = goal.l_cmds.front();
  line.target.x = 0.40;
  line.target.y = 0.10;
  line.target.z = 0.50;
  line.target.rx = 3.14;
  line.target.conf_data = {1, 0, 30, 0, 0, 0};
  line.speed = 0;
  line.zone = 7;
  line.offset_type = 1;
  line.offset_pose = {0.01, -0.02, 0.03, 0.0, 0.0, 0.0};

  rt::MotionRequestContext context;
  context.request_id = "req_1";
  context.start_joints = {0.0, 0.1, 1.5, 0.0, 1.3, 3.14};
  context.default_speed = 42;
  context.default_zone = 3;
  context.strict_conf = true;

  rt::MotionRequest request;
  std::string error;
  ASSERT_TRUE(rt::build_motion_request(goal, context, request, error)) << error;
  ASSERT_EQ(request.commands.size(), 1u);
  EXPECT_EQ(request.request_id, "req_1");
  EXPECT_DOUBLE_EQ(request.default_speed, 42.0);
  EXPECT_TRUE(request.strict_conf);
  EXPECT_EQ(request.commands.front().kind, rt::MotionKind::move_l);
  EXPECT_DOUBLE_EQ(request.commands.front().speed, 42.0);
  EXPECT_EQ(request.commands.front().zone, 7);
  EXPECT_EQ(request.commands.front().requested_conf.size(), 6u);
  EXPECT_NEAR(request.commands.front().target_cartesian[0], 0.41, 1e-9);
  EXPECT_NEAR(request.commands.front().target_cartesian[1], 0.08, 1e-9);
  EXPECT_NEAR(request.commands.front().target_cartesian[2], 0.53, 1e-9);
}

TEST(RuntimeRequestAdapterTest, BuildsReplayRequestFromRecordedPath) {
  rt::ReplayPathAsset asset;
  asset.samples = {
      {0.0, {0.0, 0.1, 1.5, 0.0, 1.3, 3.14}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {0.4, {0.05, 0.2, 1.4, 0.0, 1.2, 3.10}, {0.3, 0.2, -0.2, 0.0, -0.1, -0.1}},
  };

  rt::MotionRequestContext context;
  context.request_id = "replay_1";
  context.start_joints.assign(asset.samples.front().joint_position.begin(),
                              asset.samples.front().joint_position.end());
  context.default_speed = 50;
  context.default_zone = 5;
  context.trajectory_dt = 0.02;

  rt::MotionRequest request;
  std::string error;
  ASSERT_TRUE(rt::build_replay_request(asset, 0.5, context, request, error)) << error;
  ASSERT_EQ(request.commands.size(), 1u);
  EXPECT_EQ(request.commands.front().kind, rt::MotionKind::move_absj);
  EXPECT_TRUE(request.commands.front().use_preplanned_trajectory);
  EXPECT_EQ(request.commands.front().target_joints,
            std::vector<double>(asset.samples.back().joint_position.begin(), asset.samples.back().joint_position.end()));
  EXPECT_FALSE(request.commands.front().preplanned_trajectory.empty());
  EXPECT_EQ(request.commands.front().preplanned_trajectory.size(),
            request.commands.front().preplanned_velocity_trajectory.size());
  EXPECT_EQ(request.commands.front().preplanned_trajectory.size(),
            request.commands.front().preplanned_acceleration_trajectory.size());
  EXPECT_NEAR(request.commands.front().preplanned_dt, 0.02, 1e-9);
  EXPECT_NEAR(request.commands.front().preplanned_dt *
                  static_cast<double>(request.commands.front().preplanned_trajectory.size() - 1),
              0.8,
              1e-9);
  EXPECT_DOUBLE_EQ(request.commands.front().speed, 25.0);
  EXPECT_NEAR(request.commands.front().preplanned_velocity_trajectory.back()[0], 0.15, 1e-9);
  EXPECT_NEAR(request.commands.front().preplanned_velocity_trajectory.back()[2], -0.1, 1e-9);
}

TEST(RuntimeRequestAdapterTest, ReplayRequestRetimesByRecordedTimelineInsteadOfDtOnly) {
  rt::ReplayPathAsset asset;
  asset.samples = {
      {0.0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {0.2, {0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.4, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {0.5, {0.2, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.2, 0.0, 0.0, 0.0, 0.0, 0.0}},
  };

  rt::MotionRequestContext context;
  context.request_id = "replay_rate_contract";
  context.start_joints.assign(asset.samples.front().joint_position.begin(),
                              asset.samples.front().joint_position.end());
  context.default_speed = 120.0;
  context.default_zone = 0;
  context.trajectory_dt = 0.05;

  rt::MotionRequest nominal_request;
  std::string error;
  ASSERT_TRUE(rt::build_replay_request(asset, 1.0, context, nominal_request, error)) << error;
  rt::MotionRequest fast_request;
  ASSERT_TRUE(rt::build_replay_request(asset, 2.0, context, fast_request, error)) << error;

  const auto &nominal = nominal_request.commands.front();
  const auto &fast = fast_request.commands.front();
  ASSERT_FALSE(nominal.preplanned_trajectory.empty());
  ASSERT_FALSE(fast.preplanned_trajectory.empty());

  const double nominal_total =
      nominal.preplanned_dt * static_cast<double>(nominal.preplanned_trajectory.size() - 1);
  const double fast_total =
      fast.preplanned_dt * static_cast<double>(fast.preplanned_trajectory.size() - 1);
  EXPECT_NEAR(nominal_total, 0.5, 1e-9);
  EXPECT_NEAR(fast_total, 0.25, 1e-9);
  EXPECT_DOUBLE_EQ(nominal.preplanned_trajectory.back()[0], 0.2);
  EXPECT_DOUBLE_EQ(fast.preplanned_trajectory.back()[0], 0.2);
  EXPECT_GT(fast.preplanned_velocity_trajectory[1][0], nominal.preplanned_velocity_trajectory[1][0]);
}

TEST(RuntimeRequestAdapterTest, ReplayAssetPreservesMetadataAndToolingContext) {
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  tooling_state.setToolset("tcp_demo", "fixture_demo",
                           {0.0, 0.0, 0.08, 0.0, 0.0, 0.0},
                           {0.2, -0.1, 0.3, 0.0, 0.0, 0.0});
  tooling_state.setToolDynamics("tcp_demo", 0.6, {0.0, 0.0, 0.05});

  program_state.startRecordingPath(tooling_state.toolset(), "unit_test_record");
  program_state.recordPathSample(10.0, {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  program_state.recordPathSample(10.2, {0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, {0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
  program_state.stopRecordingPath();
  program_state.saveRecordedPath("asset_demo");

  rt::ReplayPathAsset asset;
  ASSERT_TRUE(program_state.getReplayAsset("asset_demo", asset));
  EXPECT_EQ(asset.metadata.version, "v1");
  EXPECT_EQ(asset.metadata.robot, "xMate3");
  EXPECT_EQ(asset.metadata.source, "unit_test_record");
  EXPECT_DOUBLE_EQ(asset.metadata.created_at_sec, 10.0);
  EXPECT_EQ(asset.toolset.tool_name, "tcp_demo");
  EXPECT_EQ(asset.toolset.wobj_name, "fixture_demo");
  EXPECT_DOUBLE_EQ(asset.toolset.base_pose[0], 0.2);
  EXPECT_DOUBLE_EQ(asset.toolset.tool_mass, 0.6);
  EXPECT_DOUBLE_EQ(asset.toolset.tool_com[2], 0.05);
}

TEST(RuntimeOperationStateAdapterTest, MapsStateAndFormatsLogEvent) {
  rt::RuntimeView view;
  view.has_request = true;
  view.active_motion = true;
  view.can_accept_request = false;
  view.status.request_id = "move_9";
  view.status.state = rt::ExecutionState::executing;
  view.status.current_segment_index = 1;
  view.status.total_segments = 3;
  view.status.completed_segments = 1;
  view.status.message = "tracking";
  view.status.revision = 7;
  view.status.execution_backend = rt::ExecutionBackend::jtc;
  view.status.control_owner = rt::ControlOwner::trajectory;
  view.status.runtime_phase = rt::RuntimePhase::executing;

  rt::OperationStateContext context;
  context.connected = true;
  context.power_on = true;
  EXPECT_EQ(rt::resolve_operation_state(view, context),
            rokae_xmate3_ros2::msg::OperationState::MOVING);

  const auto log_event = rt::build_runtime_log_event(view.status, 0);
  EXPECT_TRUE(log_event.should_log);
  EXPECT_FALSE(log_event.warning);
  EXPECT_EQ(log_event.revision, 7u);
  EXPECT_NE(log_event.text.find("request=move_9"), std::string::npos);
  EXPECT_NE(log_event.text.find("state=executing"), std::string::npos);
  EXPECT_NE(log_event.text.find("backend=jtc"), std::string::npos);
  EXPECT_NE(log_event.text.find("owner=trajectory"), std::string::npos);
  EXPECT_NE(log_event.text.find("phase=executing"), std::string::npos);
}

TEST(RuntimeSdkShimModelTest, KdlBackedKinematicsExposeNonZeroVelocityAndAccelerationMappings) {
  rokae::xMateModel<6> model;
  const std::array<double, 6> q{{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926}};
  const std::array<double, 6> qd{{0.1, -0.05, 0.02, 0.01, 0.03, -0.02}};
  const std::array<double, 6> qdd{{0.2, 0.1, -0.04, 0.02, 0.03, 0.01}};

  const auto jac = model.jacobian(q);
  EXPECT_EQ(jac.size(), 36u);
  double jac_norm = 0.0;
  for (double value : jac) {
    jac_norm += std::fabs(value);
  }
  EXPECT_GT(jac_norm, 0.0);

  const auto cart_vel = model.getCartVel(q, qd);
  const auto cart_acc = model.getCartAcc(q, qd, qdd);
  const auto joint_acc = model.getJointAcc(cart_acc, q, qd);
  gazebo::xMate3Kinematics kinematics;
  const auto expected_cart_vel = rokae_xmate3_ros2::gazebo_model::cartesianVelocity(kinematics, q, qd);
  const auto expected_cart_acc =
      rokae_xmate3_ros2::gazebo_model::cartesianAcceleration(kinematics, q, qd, qdd);
  const auto expected_joint_acc =
      rokae_xmate3_ros2::gazebo_model::jointAccelerationFromCartesian(kinematics, q, cart_acc);

  double cart_vel_norm = 0.0;
  double cart_acc_norm = 0.0;
  double joint_acc_norm = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    cart_vel_norm += std::fabs(cart_vel[i]);
    cart_acc_norm += std::fabs(cart_acc[i]);
    joint_acc_norm += std::fabs(joint_acc[i]);
    EXPECT_NEAR(cart_vel[i], expected_cart_vel[i], 1e-9);
    EXPECT_NEAR(cart_acc[i], expected_cart_acc[i], 1e-6);
    EXPECT_NEAR(joint_acc[i], expected_joint_acc[i], 1e-6);
  }
  EXPECT_GT(cart_vel_norm, 0.0);
  EXPECT_GT(cart_acc_norm, 0.0);
  EXPECT_GT(joint_acc_norm, 0.0);
}

TEST(RuntimeSdkShimModelTest, ModelFacadeUnifiesToolPoseLoadAndExpectedTorqueChain) {
  gazebo::xMate3Kinematics kinematics;
  rokae_xmate3_ros2::gazebo_model::ModelFacade facade(kinematics);
  facade.setToolPose({0.0, 0.0, 0.08, 0.0, 0.0, 0.0})
      .setLoad({0.6, {0.0, 0.0, 0.05}});

  const std::array<double, 6> q{{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926}};
  const std::array<double, 6> qd{{0.1, -0.05, 0.02, 0.01, 0.03, -0.02}};
  const std::array<double, 6> qdd{{0.2, 0.1, -0.04, 0.02, 0.03, 0.01}};
  const std::array<double, 6> wrench{{0.0, 0.0, 3.0, 0.0, 0.0, 0.0}};

  const auto cart_pose = facade.cartPose(q);
  const auto expected_pose = rokae_xmate3_ros2::gazebo_model::cartesianPose(
      q, kinematics, {0.0, 0.0, 0.08, 0.0, 0.0, 0.0});
  const auto mass_matrix = facade.massMatrix(q);
  const auto gravity = facade.gravity(q);
  const auto coriolis = facade.coriolis(q, qd);
  const auto inverse_dynamics = facade.inverseDynamics(q, qd, qdd, wrench);
  const auto dynamics = facade.dynamics(q, qd, qdd, wrench);
  const auto expected = rokae_xmate3_ros2::gazebo_model::computeApproximateDynamics(
      kinematics, q, qd, qdd, wrench, {0.6, {0.0, 0.0, 0.05}});
  const auto expected_torque = facade.expectedTorque(q, qd);

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(cart_pose[i], expected_pose[i], 1e-9);
    EXPECT_NEAR(dynamics.full[i], expected.full[i], 1e-9);
    EXPECT_NEAR(dynamics.gravity[i], expected.gravity[i], 1e-9);
    EXPECT_NEAR(gravity[i], expected.gravity[i], 1e-9);
    EXPECT_NEAR(coriolis[i], expected.coriolis[i], 1e-9);
    EXPECT_NEAR(inverse_dynamics[i], expected.full[i], 1e-9);
    EXPECT_GT(mass_matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)), 0.0);
  }
  double torque_norm = 0.0;
  for (double value : expected_torque) {
    torque_norm += std::fabs(value);
  }
  EXPECT_GT(torque_norm, 0.0);
}

TEST(RuntimeSdkShimModelTest, KdlBackendTracksLegacyDhReferenceWithinSmokeTolerance) {
  gazebo::xMate3Kinematics kinematics;
  EXPECT_STREQ(kinematics.backendName(), "kdl");

  const std::vector<double> q{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const Matrix4d kdl_fk = kinematics.forwardKinematics(q);
  const Matrix4d legacy_fk = legacyForwardKinematics(q);
  const Vector6d fk_error = legacyPoseError(legacy_fk, kdl_fk);

  EXPECT_LT(fk_error.head<3>().norm(), 1e-3);
  EXPECT_LT(fk_error.tail<3>().norm(), 1e-2);

  const Matrix6d kdl_jac = kinematics.computeJacobian(q);
  const Matrix6d legacy_jac = legacyFiniteDifferenceJacobian(q);
  EXPECT_LT((kdl_jac - legacy_jac).norm(), 1e-1);
}

TEST(RuntimeSdkShimModelTest, KdlBackedIkCandidateMetricsPreferContinuousNonSingularBranches) {
  gazebo::xMate3Kinematics kinematics;
  const std::vector<double> seed{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  auto continuous = seed;
  continuous[0] += 0.02;
  continuous[2] -= 0.03;
  auto discontinuous = seed;
  discontinuous[0] += 0.9;
  discontinuous[2] -= 0.7;
  auto near_singular = seed;
  near_singular[4] = 0.0;

  const auto continuous_metrics = kinematics.evaluateIkCandidate(continuous, seed);
  const auto discontinuous_metrics = kinematics.evaluateIkCandidate(discontinuous, seed);
  const auto singular_metrics = kinematics.evaluateIkCandidate(near_singular, seed);

  ASSERT_TRUE(continuous_metrics.valid);
  ASSERT_TRUE(discontinuous_metrics.valid);
  ASSERT_TRUE(singular_metrics.valid);
  EXPECT_LT(continuous_metrics.branch_distance, discontinuous_metrics.branch_distance);
  EXPECT_LT(continuous_metrics.continuity_cost, discontinuous_metrics.continuity_cost);
  EXPECT_LT(continuous_metrics.singularity_metric, singular_metrics.singularity_metric);
  EXPECT_FALSE(continuous_metrics.near_singularity);
  EXPECT_TRUE(singular_metrics.near_singularity);
}

TEST(RuntimeSdkShimModelTest, SeededIkReproducesForwardKinematicsTargetContinuously) {
  gazebo::xMate3Kinematics kinematics;
  EXPECT_STREQ(kinematics.backendName(), "kdl");

  const std::vector<double> q{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto target = kinematics.forwardKinematicsRPY(q);
  const auto solved = kinematics.inverseKinematicsSeededFast(target, q);

  ASSERT_EQ(solved.size(), 6u);
  const auto solved_fk = kinematics.forwardKinematics(solved);
  const auto expected_fk = kinematics.forwardKinematics(q);
  const auto fk_error = legacyPoseError(expected_fk, solved_fk);
  EXPECT_LT(fk_error.head<3>().norm(), 1e-4);
  EXPECT_LT(fk_error.tail<3>().norm(), 1e-3);

  double joint_delta = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    joint_delta += std::fabs(solved[i] - q[i]);
  }
  EXPECT_LT(joint_delta, 0.1);
}

TEST(RuntimeSdkShimModelTest, MultiBranchIkReturnsContinuousCandidatesForSamePose) {
  gazebo::xMate3Kinematics kinematics;
  EXPECT_STREQ(kinematics.backendName(), "kdl");

  const std::vector<double> q{0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  const auto target = kinematics.forwardKinematicsRPY(q);
  const auto solutions = kinematics.inverseKinematicsMultiSolution(target, q);

  ASSERT_FALSE(solutions.empty());
  const auto solved_fk = kinematics.forwardKinematics(solutions.front());
  const auto expected_fk = kinematics.forwardKinematics(q);
  const auto fk_error = legacyPoseError(expected_fk, solved_fk);
  EXPECT_LT(fk_error.head<3>().norm(), 1e-4);
  EXPECT_LT(fk_error.tail<3>().norm(), 1e-3);
}

TEST(RuntimeArchiveVerificationTest, RejectsForbiddenArtifactsInSourceArchive) {
  const auto temp_root = std::filesystem::temp_directory_path() /
                         ("rokae_archive_verify_" +
                          std::to_string(static_cast<long long>(
                              std::chrono::steady_clock::now().time_since_epoch().count())));
  const auto source_tree = temp_root / "source_tree";
  const auto clean_tree = temp_root / "clean_tree";
  const auto dirty_tree = temp_root / "dirty_tree";
  const auto clean_archive = temp_root / "clean.zip";
  const auto dirty_archive = temp_root / "dirty.zip";

  std::filesystem::create_directories(source_tree);
  std::filesystem::create_directories(clean_tree);
  std::filesystem::create_directories(dirty_tree);

  createArchiveFixtureTree(source_tree, false);
  createArchiveFixtureTree(clean_tree, false);
  createArchiveFixtureTree(dirty_tree, true);

  const std::string zipper_script =
      "import pathlib, sys, zipfile\n"
      "root = pathlib.Path(sys.argv[1])\n"
      "archive = pathlib.Path(sys.argv[2])\n"
      "with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as zf:\n"
      "  for path in sorted(root.rglob('*')):\n"
      "    if path.is_file():\n"
      "      zf.write(path, path.relative_to(root))\n";

  const auto make_clean_archive =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " -c " + shellQuote(zipper_script) + " " +
      shellQuote(clean_tree.string()) + " " + shellQuote(clean_archive.string());
  const auto make_dirty_archive =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " -c " + shellQuote(zipper_script) + " " +
      shellQuote(dirty_tree.string()) + " " + shellQuote(dirty_archive.string());
  ASSERT_EQ(runShellCommand(make_clean_archive), 0);
  ASSERT_EQ(runShellCommand(make_dirty_archive), 0);

  const auto verify_clean =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " " +
      shellQuote(std::string(ROKAE_TEST_VERIFY_SOURCE_ARCHIVE_PY)) + " --source-root " +
      shellQuote(source_tree.string()) + " --archive " + shellQuote(clean_archive.string());
  const auto verify_dirty =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " " +
      shellQuote(std::string(ROKAE_TEST_VERIFY_SOURCE_ARCHIVE_PY)) + " --source-root " +
      shellQuote(source_tree.string()) + " --archive " + shellQuote(dirty_archive.string());

  EXPECT_EQ(runShellCommand(verify_clean), 0);
  EXPECT_NE(runShellCommand(verify_dirty), 0);

  std::error_code cleanup_error;
  std::filesystem::remove_all(temp_root, cleanup_error);
}

TEST(RuntimeArchiveVerificationTest, RejectsArchiveMissingWorkspaceRootGitignoreEvenIfNestedGitignoreExists) {
  const auto temp_root = std::filesystem::temp_directory_path() /
                         ("rokae_archive_root_gitignore_" +
                          std::to_string(static_cast<long long>(
                              std::chrono::steady_clock::now().time_since_epoch().count())));
  const auto source_tree = temp_root / "source_tree";
  const auto nested_only_tree = temp_root / "nested_only_tree";
  const auto nested_only_archive = temp_root / "nested_only.zip";

  std::filesystem::create_directories(source_tree);
  std::filesystem::create_directories(nested_only_tree);

  createArchiveFixtureTree(source_tree, false);
  createArchiveFixtureTree(nested_only_tree, false);
  std::filesystem::remove(nested_only_tree / ".gitignore");
  writeTextFile(nested_only_tree / "src/rokae_xmate3_ros2/.gitignore", "__pycache__/\n*.pyc\n");

  const std::string zipper_script =
      "import pathlib, sys, zipfile\n"
      "root = pathlib.Path(sys.argv[1])\n"
      "archive = pathlib.Path(sys.argv[2])\n"
      "with zipfile.ZipFile(archive, 'w', zipfile.ZIP_DEFLATED) as zf:\n"
      "  for path in sorted(root.rglob('*')):\n"
      "    if path.is_file():\n"
      "      zf.write(path, path.relative_to(root))\n";
  const auto make_archive =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " -c " + shellQuote(zipper_script) + " " +
      shellQuote(nested_only_tree.string()) + " " + shellQuote(nested_only_archive.string());
  ASSERT_EQ(runShellCommand(make_archive), 0);

  const auto verify_nested_only =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " " +
      shellQuote(std::string(ROKAE_TEST_VERIFY_SOURCE_ARCHIVE_PY)) + " --source-root " +
      shellQuote(source_tree.string()) + " --archive " + shellQuote(nested_only_archive.string());
  EXPECT_NE(runShellCommand(verify_nested_only), 0);

  std::error_code cleanup_error;
  std::filesystem::remove_all(temp_root, cleanup_error);
}

TEST(RuntimeArchiveVerificationTest, PackagingScriptCreatesVerifiedCleanArchiveFromTrackedFiles) {
  const auto temp_root = std::filesystem::temp_directory_path() /
                         ("rokae_archive_package_" +
                          std::to_string(static_cast<long long>(
                              std::chrono::steady_clock::now().time_since_epoch().count())));
  const auto source_tree = temp_root / "source_tree";
  const auto output_archive = temp_root / "candidate.zip";
  const auto manifest_path = temp_root / "candidate.zip.manifest.txt";
  const auto sha256_path = temp_root / "candidate.zip.sha256";

  std::filesystem::create_directories(source_tree);
  createArchiveFixtureTree(source_tree, false);
  writeTextFile(source_tree / "src/rokae_xmate3_ros2/test/harness/__pycache__/stale.cpython-312.pyc", "pyc");

  const auto git_init = "git -C " + shellQuote(source_tree.string()) + " init --quiet";
  const auto git_add = "git -C " + shellQuote(source_tree.string()) + " add .";
  ASSERT_EQ(runShellCommand(git_init), 0);
  ASSERT_EQ(runShellCommand(git_add), 0);

  const auto package_archive =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " " +
      shellQuote(std::string(ROKAE_TEST_CREATE_VERIFIED_SOURCE_ARCHIVE_PY)) + " --source-root " +
      shellQuote(source_tree.string()) + " --archive " + shellQuote(output_archive.string());
  ASSERT_EQ(runShellCommand(package_archive), 0);

  const auto verify_packaged =
      shellQuote(std::string(ROKAE_TEST_PYTHON_EXECUTABLE)) + " " +
      shellQuote(std::string(ROKAE_TEST_VERIFY_SOURCE_ARCHIVE_PY)) + " --source-root " +
      shellQuote(source_tree.string()) + " --archive " + shellQuote(output_archive.string());
  EXPECT_EQ(runShellCommand(verify_packaged), 0);
  ASSERT_TRUE(std::filesystem::exists(manifest_path));
  ASSERT_TRUE(std::filesystem::exists(sha256_path));

  const auto manifest_text = readTextFile(manifest_path);
  const auto sha256_text = readTextFile(sha256_path);
  EXPECT_NE(manifest_text.find("archive=candidate.zip"), std::string::npos);
  EXPECT_NE(manifest_text.find("file_count="), std::string::npos);
  EXPECT_NE(manifest_text.find(".gitignore"), std::string::npos);
  EXPECT_NE(sha256_text.find("candidate.zip"), std::string::npos);

  std::error_code cleanup_error;
  std::filesystem::remove_all(temp_root, cleanup_error);
}
