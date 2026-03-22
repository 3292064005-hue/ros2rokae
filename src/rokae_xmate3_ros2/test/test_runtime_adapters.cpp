#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include "rokae/model.h"
#include "runtime/operation_state_adapter.hpp"
#include "runtime/request_adapter.hpp"
#include "runtime/runtime_state.hpp"
#include "rokae_xmate3_ros2/msg/operation_state.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

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

  double cart_vel_norm = 0.0;
  double cart_acc_norm = 0.0;
  double joint_acc_norm = 0.0;
  for (double value : cart_vel) {
    cart_vel_norm += std::fabs(value);
  }
  for (double value : cart_acc) {
    cart_acc_norm += std::fabs(value);
  }
  for (double value : joint_acc) {
    joint_acc_norm += std::fabs(value);
  }
  EXPECT_GT(cart_vel_norm, 0.0);
  EXPECT_GT(cart_acc_norm, 0.0);
  EXPECT_GT(joint_acc_norm, 0.0);
}
