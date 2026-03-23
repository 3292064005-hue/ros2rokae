#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "runtime/joint_retimer.hpp"
#include "runtime/runtime_state.hpp"
#include "runtime/motion_runtime.hpp"
#include "runtime/pose_utils.hpp"
#include "runtime/service_facade.hpp"
#include "rokae_xmate3_ros2/gazebo/approximate_model.hpp"
#include "rokae_xmate3_ros2/gazebo/kinematics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

namespace {

class FakeBackend final : public rt::BackendInterface {
 public:
  rt::RobotSnapshot readSnapshot() const override { return snapshot; }

  void applyControl(const rt::ControlCommand &command) override {
    last_command = command;
    apply_count++;
  }

  void clearControl() override { clear_count++; }

  void setBrakeLock(const rt::RobotSnapshot &, bool locked) override {
    brake_locked = locked;
    brake_set_count++;
  }

  bool brakesLocked() const override { return brake_locked; }

  mutable rt::RobotSnapshot snapshot{};
  rt::ControlCommand last_command{};
  int apply_count = 0;
  int clear_count = 0;
  int brake_set_count = 0;
  bool brake_locked = false;
};

}  // namespace

TEST(ServiceFacadeTest, ControlFacadeCoordinatesPowerDragAndStopWithRuntime) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, motion_runtime);
  FakeBackend backend;
  rt::ControlFacade facade(session_state, motion_options_state, &backend, &motion_runtime, &coordinator);

  rokae_xmate3_ros2::srv::Connect::Request connect_req;
  rokae_xmate3_ros2::srv::Connect::Response connect_res;
  connect_req.remote_ip = "127.0.0.1";
  facade.handleConnect(connect_req, connect_res);
  ASSERT_TRUE(connect_res.success);
  EXPECT_TRUE(session_state.connected());

  rokae_xmate3_ros2::srv::SetPowerState::Request power_req;
  rokae_xmate3_ros2::srv::SetPowerState::Response power_res;
  power_req.on = true;
  facade.handleSetPowerState(power_req, power_res);
  ASSERT_TRUE(power_res.success);
  EXPECT_TRUE(session_state.powerOn());
  EXPECT_FALSE(backend.brakesLocked());

  rokae_xmate3_ros2::srv::EnableDrag::Request drag_req;
  rokae_xmate3_ros2::srv::EnableDrag::Response drag_res;
  facade.handleEnableDrag(drag_req, drag_res);
  ASSERT_TRUE(drag_res.success);
  EXPECT_TRUE(session_state.dragMode());
  EXPECT_EQ(backend.clear_count, 1);

  rokae_xmate3_ros2::srv::DisableDrag::Request undrag_req;
  rokae_xmate3_ros2::srv::DisableDrag::Response undrag_res;
  facade.handleDisableDrag(undrag_req, undrag_res);
  ASSERT_TRUE(undrag_res.success);
  EXPECT_FALSE(session_state.dragMode());
  EXPECT_EQ(backend.clear_count, 2);

  rokae_xmate3_ros2::srv::AdjustSpeedOnline::Request speed_req;
  rokae_xmate3_ros2::srv::AdjustSpeedOnline::Response speed_res;
  speed_req.scale = 1.6;
  facade.handleAdjustSpeedOnline(speed_req, speed_res);
  ASSERT_TRUE(speed_res.success);
  EXPECT_DOUBLE_EQ(motion_options_state.speedScale(), 1.6);
  EXPECT_DOUBLE_EQ(motion_runtime.activeSpeedScale(), 1.6);

  rokae_xmate3_ros2::srv::EnableCollisionDetection::Request collision_req;
  rokae_xmate3_ros2::srv::EnableCollisionDetection::Response collision_res;
  collision_req.sensitivity = {2.0, 1.5, 1.0, 0.5, 3.0, 4.0};
  collision_req.behaviour = 2;
  collision_req.fallback = 0.7;
  facade.handleEnableCollisionDetection(collision_req, collision_res);
  ASSERT_TRUE(collision_res.success);
  const auto collision_snapshot = session_state.collisionDetection();
  EXPECT_TRUE(collision_snapshot.enabled);
  EXPECT_EQ(collision_snapshot.behaviour, 2u);
  EXPECT_DOUBLE_EQ(collision_snapshot.fallback, 0.7);
  EXPECT_DOUBLE_EQ(collision_snapshot.sensitivity[0], 2.0);

  rokae_xmate3_ros2::srv::DisableCollisionDetection::Request collision_disable_req;
  rokae_xmate3_ros2::srv::DisableCollisionDetection::Response collision_disable_res;
  facade.handleDisableCollisionDetection(collision_disable_req, collision_disable_res);
  ASSERT_TRUE(collision_disable_res.success);
  EXPECT_FALSE(session_state.collisionDetectionEnabled());

  rokae_xmate3_ros2::srv::Stop::Request stop_req;
  rokae_xmate3_ros2::srv::Stop::Response stop_res;
  facade.handleStop(stop_req, stop_res);
  ASSERT_TRUE(stop_res.success);
  EXPECT_EQ(backend.clear_count, 3);
  EXPECT_TRUE(motion_runtime.status().state == rt::ExecutionState::stopped ||
              motion_runtime.status().state == rt::ExecutionState::idle);

  power_req.on = false;
  facade.handleSetPowerState(power_req, power_res);
  ASSERT_TRUE(power_res.success);
  EXPECT_FALSE(session_state.powerOn());
  EXPECT_TRUE(backend.brakesLocked());
  EXPECT_GE(backend.brake_set_count, 1);
  EXPECT_TRUE(motion_runtime.status().state == rt::ExecutionState::stopped ||
              motion_runtime.status().state == rt::ExecutionState::idle);
}

TEST(ServiceFacadeTest, PathFacadeSubmitsReplayRequestsThroughCoordinator) {
  rt::ProgramState program_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::SessionState session_state;
  session_state.setPowerOn(true);
  rt::MotionRuntime motion_runtime;
  rt::MotionRequestCoordinator coordinator(motion_options_state, tooling_state, motion_runtime);

  auto joint_state_fetcher = [](std::array<double, 6> &position,
                                std::array<double, 6> &velocity,
                                std::array<double, 6> &torque) {
    position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    velocity.fill(0.0);
    torque.fill(0.0);
  };
  auto dt_provider = []() { return 0.01; };
  auto request_id_generator = [](const std::string &prefix) { return prefix + "_001"; };

  rt::PathFacade facade(
      program_state, tooling_state, &coordinator, joint_state_fetcher, dt_provider, request_id_generator);

  program_state.startRecordingPath(tooling_state.toolset(), "test_record");
  program_state.recordPathSample(0.00, {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  program_state.recordPathSample(0.20, {0.2, 0.3, 0.4, 0.5, 0.6, 0.7}, {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  program_state.stopRecordingPath();
  program_state.saveRecordedPath("demo_path");

  rokae_xmate3_ros2::srv::ReplayPath::Request req;
  rokae_xmate3_ros2::srv::ReplayPath::Response res;
  req.name = "demo_path";
  req.rate = 1.0;
  facade.handleReplayPath(req, res);

  ASSERT_TRUE(res.success) << res.message;
  const auto view = coordinator.currentView();
  EXPECT_TRUE(view.has_request);
  EXPECT_EQ(view.status.request_id, "replay_demo_path_001");
  EXPECT_EQ(view.status.state, rt::ExecutionState::planning);
}

TEST(ServiceFacadeTest, QueryFacadeAppliesToolingCoordinateSemanticsAndApproximateServices) {
  rt::SessionState session_state;
  rt::MotionOptionsState motion_options_state;
  rt::ToolingState tooling_state;
  rt::DataStoreState data_store_state;
  rt::ProgramState program_state;
  gazebo::xMate3Kinematics kinematics;

  const std::array<double, 6> joints = {0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926};
  auto joint_state_fetcher = [&](std::array<double, 6> &position,
                                 std::array<double, 6> &velocity,
                                 std::array<double, 6> &torque) {
    position = joints;
    velocity = {0.10, -0.05, 0.03, 0.0, 0.0, 0.0};
    torque = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5};
  };

  rt::QueryFacade facade(session_state,
                         motion_options_state,
                         tooling_state,
                         data_store_state,
                         program_state,
                         kinematics,
                         joint_state_fetcher,
                         []() { return rclcpp::Time(0); },
                         []() { return 0.01; },
                         6);

  rokae_xmate3_ros2::srv::SetToolset::Request set_tool_req;
  rokae_xmate3_ros2::srv::SetToolset::Response set_tool_res;
  set_tool_req.tool_name = "tcpA";
  set_tool_req.wobj_name = "fixtureA";
  set_tool_req.tool_pose = {0.0, 0.0, 0.10, 0.0, 0.0, 0.0};
  set_tool_req.wobj_pose = {0.20, -0.10, 0.30, 0.0, 0.0, 0.0};
  facade.handleSetToolset(set_tool_req, set_tool_res);
  ASSERT_TRUE(set_tool_res.success);
  tooling_state.setToolDynamics("tcpA", 0.6, {0.0, 0.0, 0.08});

  rokae_xmate3_ros2::srv::GetBaseFrame::Request base_req;
  rokae_xmate3_ros2::srv::GetBaseFrame::Response base_res;
  facade.handleGetBaseFrame(base_req, base_res);
  ASSERT_TRUE(base_res.success);
  EXPECT_DOUBLE_EQ(base_res.base_frame[0], 0.20);
  EXPECT_DOUBLE_EQ(base_res.base_frame[2], 0.30);

  rokae_xmate3_ros2::srv::GetPosture::Request flange_req;
  rokae_xmate3_ros2::srv::GetPosture::Response flange_res;
  flange_req.coordinate_type = 0;
  facade.handleGetPosture(flange_req, flange_res);
  ASSERT_TRUE(flange_res.success);

  rokae_xmate3_ros2::srv::GetPosture::Request end_req;
  rokae_xmate3_ros2::srv::GetPosture::Response end_res;
  end_req.coordinate_type = 1;
  facade.handleGetPosture(end_req, end_res);
  ASSERT_TRUE(end_res.success);

  const auto flange_pose = kinematics.forwardKinematicsRPY(std::vector<double>(joints.begin(), joints.end()));
  const auto expected_end_pose = rt::pose_utils::convertFlangeInBaseToEndInRef(
      flange_pose, std::vector<double>(set_tool_req.tool_pose.begin(), set_tool_req.tool_pose.end()),
      std::vector<double>(set_tool_req.wobj_pose.begin(), set_tool_req.wobj_pose.end()));
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(flange_res.posture[i], flange_pose[i], 1e-9);
    EXPECT_NEAR(end_res.posture[i], expected_end_pose[i], 1e-9);
  }

  rokae_xmate3_ros2::srv::SetToolsetByName::Request named_req;
  rokae_xmate3_ros2::srv::SetToolsetByName::Response named_res;
  named_req.tool_name = "tcpA";
  named_req.wobj_name = "fixtureA";
  facade.handleSetToolsetByName(named_req, named_res);
  EXPECT_TRUE(named_res.success);
  named_req.tool_name = "missing";
  facade.handleSetToolsetByName(named_req, named_res);
  EXPECT_FALSE(named_res.success);
  EXPECT_EQ(named_res.message, "unknown tool_name or wobj_name");

  rokae_xmate3_ros2::srv::CalcFk::Request fk_req;
  rokae_xmate3_ros2::srv::CalcFk::Response fk_res;
  fk_req.joint_positions = joints;
  facade.handleCalcFk(fk_req, fk_res);
  ASSERT_TRUE(fk_res.success);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(fk_res.posture[i], expected_end_pose[i], 1e-9);
  }

  rokae_xmate3_ros2::srv::CalcIk::Request ik_req;
  rokae_xmate3_ros2::srv::CalcIk::Response ik_res;
  ik_req.target_posture = fk_res.posture;
  facade.handleCalcIk(ik_req, ik_res);
  ASSERT_TRUE(ik_res.success) << ik_res.message;
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(ik_res.joint_positions[i], joints[i], 1e-3);
  }

  rokae_xmate3_ros2::srv::GetPosture::Request invalid_posture_req;
  rokae_xmate3_ros2::srv::GetPosture::Response invalid_posture_res;
  invalid_posture_req.coordinate_type = 9;
  facade.handleGetPosture(invalid_posture_req, invalid_posture_res);
  EXPECT_FALSE(invalid_posture_res.success);
  EXPECT_EQ(invalid_posture_res.message, "unsupported coordinate_type");

  rokae_xmate3_ros2::srv::GetCartPosture::Request invalid_cart_req;
  rokae_xmate3_ros2::srv::GetCartPosture::Response invalid_cart_res;
  invalid_cart_req.coordinate_type = 9;
  facade.handleGetCartPosture(invalid_cart_req, invalid_cart_res);
  EXPECT_FALSE(invalid_cart_res.success);
  EXPECT_EQ(invalid_cart_res.message, "unsupported coordinate_type");

  rokae_xmate3_ros2::srv::CalcJointTorque::Request torque_req;
  rokae_xmate3_ros2::srv::CalcJointTorque::Response torque_res;
  torque_req.joint_pos = joints;
  torque_req.joint_vel = {0.10, 0.05, -0.02, 0.0, 0.0, 0.0};
  torque_req.joint_acc = {0.2, 0.1, -0.1, 0.0, 0.0, 0.0};
  torque_req.external_force = {0.0, 0.0, 3.0, 0.0, 0.0, 0.0};
  facade.handleCalcJointTorque(torque_req, torque_res);
  ASSERT_TRUE(torque_res.success);
  const auto expected_torque = rokae_xmate3_ros2::gazebo_model::computeApproximateDynamics(
      kinematics,
      torque_req.joint_pos,
      torque_req.joint_vel,
      torque_req.joint_acc,
      torque_req.external_force,
      rokae_xmate3_ros2::gazebo_model::LoadContext{0.6, {0.0, 0.0, 0.08}});
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(torque_res.joint_torque[i], expected_torque.full[i], 1e-9);
    EXPECT_NEAR(torque_res.gravity_torque[i], expected_torque.gravity[i], 1e-9);
    EXPECT_NEAR(torque_res.coriolis_torque[i], expected_torque.coriolis[i], 1e-9);
  }
  torque_req.joint_pos[0] = std::numeric_limits<double>::quiet_NaN();
  facade.handleCalcJointTorque(torque_req, torque_res);
  EXPECT_FALSE(torque_res.success);

  rokae_xmate3_ros2::srv::GenerateSTrajectory::Request traj_req;
  rokae_xmate3_ros2::srv::GenerateSTrajectory::Response traj_res;
  traj_req.start_joint_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  traj_req.target_joint_pos = {0.2, -0.1, 0.05, 0.0, 0.0, 0.0};
  traj_req.max_velocity = 1.0;
  traj_req.max_acceleration = 2.0;
  traj_req.blend_radius = 0.1;
  traj_req.is_cartesian = false;
  facade.handleGenerateSTrajectory(traj_req, traj_res);
  ASSERT_TRUE(traj_res.success);
  EXPECT_GT(traj_res.total_time, 0.0);
  ASSERT_GT(traj_res.trajectory_points.size(), 2u);
  const auto planner_config = gazebo::TrajectoryPlanner::config();
  rt::JointRetimerConfig retimer_config;
  retimer_config.joint_speed_limits_rad_per_sec = planner_config.joint_speed_limits_rad_per_sec;
  retimer_config.joint_acc_limits_rad_per_sec2 = planner_config.joint_acc_limits_rad_per_sec2;
  retimer_config.sample_dt = 0.01;
  retimer_config.min_sample_dt = planner_config.min_sample_dt;
  retimer_config.max_sample_dt = planner_config.max_sample_dt;
  retimer_config.max_joint_step_rad = planner_config.max_joint_step_rad;
  std::vector<double> start_joint(traj_req.start_joint_pos.begin(), traj_req.start_joint_pos.end());
  std::vector<double> target_joint(traj_req.target_joint_pos.begin(), traj_req.target_joint_pos.end());
  std::array<double, 6> velocity_limits{};
  std::array<double, 6> acceleration_limits{};
  const double smoothing_scale = 1.0 / (1.0 + 0.25 * std::clamp(traj_req.blend_radius, 0.0, 1.0));
  velocity_limits.fill(std::clamp(traj_req.max_velocity, 0.05, M_PI) * smoothing_scale);
  acceleration_limits.fill(
      std::clamp(traj_req.max_acceleration, 0.05, 10.0 * M_PI) * smoothing_scale * smoothing_scale);
  const auto retimed =
      rt::retimeJointQuintic(start_joint, target_joint, retimer_config, velocity_limits, acceleration_limits);
  EXPECT_NEAR(traj_res.total_time, retimed.total_time, 1e-9);
  EXPECT_EQ(traj_res.trajectory_points.size(), retimed.positions.size());
  EXPECT_DOUBLE_EQ(traj_res.trajectory_points.front().pos[0], retimed.positions.front()[0]);
  EXPECT_DOUBLE_EQ(traj_res.trajectory_points.back().pos[0], retimed.positions.back()[0]);
  traj_req.is_cartesian = true;
  traj_req.max_velocity = 0.6;
  traj_req.max_acceleration = 1.5;
  facade.handleGenerateSTrajectory(traj_req, traj_res);
  ASSERT_TRUE(traj_res.success) << traj_res.error_msg;
  EXPECT_GT(traj_res.total_time, 0.0);
  ASSERT_GT(traj_res.trajectory_points.size(), 2u);
  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(traj_res.trajectory_points.front().pos[j], traj_req.start_joint_pos[j], 1e-9);
    EXPECT_NEAR(traj_res.trajectory_points.back().pos[j], traj_req.target_joint_pos[j], 1e-3);
  }

  rt::ControlFacade control_facade(session_state, motion_options_state, nullptr, nullptr, nullptr);
  rokae_xmate3_ros2::srv::SetDefaultZone::Request zone_req;
  rokae_xmate3_ros2::srv::SetDefaultZone::Response zone_res;
  zone_req.zone = 250;
  control_facade.handleSetDefaultZone(zone_req, zone_res);
  EXPECT_FALSE(zone_res.success);

  motion_options_state.setZoneValidRange(0, 80);
  zone_req.zone = 81;
  control_facade.handleSetDefaultZone(zone_req, zone_res);
  EXPECT_FALSE(zone_res.success);
  EXPECT_EQ(zone_res.message, "default zone must be within [0, 80] mm");
  zone_req.zone = 60;
  control_facade.handleSetDefaultZone(zone_req, zone_res);
  EXPECT_TRUE(zone_res.success);
}
