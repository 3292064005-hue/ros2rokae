#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>

#include <rclcpp/time.hpp>

#include "runtime/data_store_state.hpp"
#include "runtime/service_facade.hpp"
#include "rokae_xmate3_ros2/runtime/rt_semantic_topics.hpp"

namespace rt = rokae_xmate3_ros2::runtime;

TEST(RegisterSemantics, RegisterBankPreservesScalarValuesAndEnumerationOrder) {
  rt::DataStoreState data_store;
  data_store.setRegister("register0[0]", "1.25");
  data_store.setRegister("register2[0]", "1");

  EXPECT_EQ(data_store.registerValue("register0[0]"), "1.25");
  EXPECT_EQ(data_store.registerValue("register2[0]"), "1");

  const auto keys = data_store.registerKeys();
  EXPECT_NE(std::find(keys.begin(), keys.end(), "register0[0]"), keys.end());
  EXPECT_NE(std::find(keys.begin(), keys.end(), "register2[0]"), keys.end());
}

TEST(RegisterSemantics, SemanticTopicsUpdateTypedRtSnapshotWithoutReparsingAtReadTime) {
  rt::DataStoreState data_store;
  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigJointImpedance, "100,200,300,40,50,60");
  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kConfigFilterLimit,
                           "limit_rate=true;cutoff_frequency=55");
  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kControlDispatchMode, "rt_loop");
  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kCatalogProvenance, "runtime_authoritative");

  const auto control = data_store.rtControlSnapshot();
  EXPECT_TRUE(control.joint_impedance_configured);
  EXPECT_DOUBLE_EQ(control.joint_impedance[0], 100.0);
  EXPECT_TRUE(control.filter_limit_configured);
  EXPECT_TRUE(control.filter_limit_enabled);
  EXPECT_DOUBLE_EQ(control.filter_limit_cutoff_frequency, 55.0);

  const auto semantic = data_store.rtSemanticSnapshot();
  EXPECT_EQ(semantic.dispatch_mode, "rt_loop");
  EXPECT_EQ(semantic.catalog_provenance, "runtime_authoritative");
}

TEST(RegisterSemantics, CartesianForceControlConfigUpdatesTypedRtSnapshot) {
  rt::DataStoreState data_store;
  data_store.setCustomData(
      rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianForceControl,
      "enabled=1;kp=1,2,3,4,5,6;ki=0.1,0.2,0.3,0.4,0.5,0.6;"
      "deadband=0.01,0.02,0.03,0.04,0.05,0.06;"
      "max_feedback_wrench=7,8,9,10,11,12;cutoff_frequency_hz=25;"
      "integral_limit=0.7,0.8,0.9,1.0,1.1,1.2");

  const auto control = data_store.rtControlSnapshot().cartesian_force_control;
  EXPECT_TRUE(control.configured);
  EXPECT_TRUE(control.enabled);
  EXPECT_DOUBLE_EQ(control.kp[0], 1.0);
  EXPECT_DOUBLE_EQ(control.ki[5], 0.6);
  EXPECT_DOUBLE_EQ(control.deadband[2], 0.03);
  EXPECT_DOUBLE_EQ(control.max_feedback_wrench[4], 11.0);
  EXPECT_DOUBLE_EQ(control.cutoff_frequency_hz, 25.0);
  EXPECT_DOUBLE_EQ(control.integral_limit[3], 1.0);

  data_store.setCustomData(
      rokae_xmate3_ros2::runtime::rt_topics::kConfigCartesianForceControl,
      "enabled=1;kp=1,nan,3,4,5,6");
  const auto invalid = data_store.rtControlSnapshot().cartesian_force_control;
  EXPECT_FALSE(invalid.configured);
  EXPECT_FALSE(invalid.enabled);
}

TEST(RegisterSemantics, ExternalWrenchTopicUpdatesTypedRtSnapshot) {
  rt::DataStoreState data_store;
  data_store.setCustomData(
      rokae_xmate3_ros2::runtime::rt_topics::kSensorExternalWrench,
      "timestamp=42.5;type=" + std::to_string(static_cast<int>(rokae::FrameType::tool)) +
          ";values=1,2,3,4,5,6;frame=1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1");

  const auto wrench = data_store.rtControlSnapshot().external_wrench;
  EXPECT_TRUE(wrench.present);
  EXPECT_TRUE(wrench.valid);
  EXPECT_DOUBLE_EQ(wrench.wrench[0], 1.0);
  EXPECT_DOUBLE_EQ(wrench.wrench[5], 6.0);
  EXPECT_DOUBLE_EQ(wrench.timestamp_sec, 42.5);
  EXPECT_EQ(wrench.frame.type, rokae::FrameType::tool);
  EXPECT_TRUE(wrench.frame.configured);
  EXPECT_DOUBLE_EQ(wrench.frame.frame[15], 1.0);

  data_store.setCustomData(rokae_xmate3_ros2::runtime::rt_topics::kSensorExternalWrench,
                           "timestamp=bad;values=1,2,3,4,5,6");
  const auto invalid = data_store.rtControlSnapshot().external_wrench;
  EXPECT_TRUE(invalid.present);
  EXPECT_FALSE(invalid.valid);
}

#if ROKAE_ENABLE_INTERNAL_SURFACE && ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES
TEST(RegisterSemantics, IoProgramFacadeRejectsEmptyRegisterKeysAndNames) {
  rt::SessionState session_state;
  rt::DataStoreState data_store;
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  rt::IoProgramFacade facade(session_state, data_store, program_state, tooling_state, [] { return rclcpp::Time(0); });

  rokae_xmate3_ros2::srv::ReadRegister::Request read_req;
  rokae_xmate3_ros2::srv::ReadRegister::Response read_res;
  facade.handleReadRegister(read_req, read_res);
  EXPECT_FALSE(read_res.success);
  EXPECT_EQ(read_res.error_code, 12002);
  EXPECT_EQ(read_res.error_msg, "register key must not be empty");

  rokae_xmate3_ros2::srv::WriteRegister::Request write_req;
  rokae_xmate3_ros2::srv::WriteRegister::Response write_res;
  facade.handleWriteRegister(write_req, write_res);
  EXPECT_FALSE(write_res.success);
  EXPECT_EQ(write_res.error_code, 12002);
  EXPECT_EQ(write_res.error_msg, "register key must not be empty");

  rokae_xmate3_ros2::srv::ReadRegisterEx::Request read_ex_req;
  rokae_xmate3_ros2::srv::ReadRegisterEx::Response read_ex_res;
  facade.handleReadRegisterEx(read_ex_req, read_ex_res);
  EXPECT_FALSE(read_ex_res.success);
  EXPECT_EQ(read_ex_res.error_code, 12003);
  EXPECT_EQ(read_ex_res.error_msg, "register name must not be empty");

  rokae_xmate3_ros2::srv::WriteRegisterEx::Request write_ex_req;
  rokae_xmate3_ros2::srv::WriteRegisterEx::Response write_ex_res;
  facade.handleWriteRegisterEx(write_ex_req, write_ex_res);
  EXPECT_FALSE(write_ex_res.success);
  EXPECT_EQ(write_ex_res.error_code, 12003);
  EXPECT_EQ(write_ex_res.error_msg, "register name must not be empty");
}

TEST(RegisterSemantics, IoProgramFacadeRejectsEmptyCustomTopicAndInvalidXPanelMode) {
  rt::SessionState session_state;
  rt::DataStoreState data_store;
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  rt::IoProgramFacade facade(session_state, data_store, program_state, tooling_state, [] { return rclcpp::Time(0); });

  rokae_xmate3_ros2::srv::SendCustomData::Request custom_req;
  rokae_xmate3_ros2::srv::SendCustomData::Response custom_res;
  custom_req.data_topic.clear();
  custom_req.custom_data = "payload";
  facade.handleSendCustomData(custom_req, custom_res);
  EXPECT_FALSE(custom_res.success);
  EXPECT_EQ(custom_res.error_code, 12000);
  EXPECT_EQ(custom_res.error_msg, "data_topic must not be empty");

  rokae_xmate3_ros2::srv::SetXPanelVout::Request xpanel_req;
  rokae_xmate3_ros2::srv::SetXPanelVout::Response xpanel_res;
  xpanel_req.mode = 9;
  facade.handleSetXPanelVout(xpanel_req, xpanel_res);
  EXPECT_FALSE(xpanel_res.success);
  EXPECT_EQ(xpanel_res.error_code, 12005);
  EXPECT_EQ(xpanel_res.error_msg, "xpanel mode must be one of off/reserve/supply12v/supply24v");
}

TEST(RegisterSemantics, IoProgramFacadeRejectsDisconnectedRequestsAndInvalidIndices) {
  rt::SessionState session_state;
  rt::DataStoreState data_store;
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  rt::IoProgramFacade facade(session_state, data_store, program_state, tooling_state, [] { return rclcpp::Time(0); });

  rokae_xmate3_ros2::srv::ReadRegister::Request read_req;
  rokae_xmate3_ros2::srv::ReadRegister::Response read_res;
  read_req.key = "register0[0]";
  facade.handleReadRegister(read_req, read_res);
  EXPECT_FALSE(read_res.success);
  EXPECT_EQ(read_res.error_code, 12006);

  session_state.connect("127.0.0.1");
  rokae_xmate3_ros2::srv::ReadRegisterEx::Request read_ex_req;
  rokae_xmate3_ros2::srv::ReadRegisterEx::Response read_ex_res;
  read_ex_req.name = "register0";
  read_ex_req.index = -1;
  facade.handleReadRegisterEx(read_ex_req, read_ex_res);
  EXPECT_FALSE(read_ex_res.success);
  EXPECT_EQ(read_ex_res.error_code, 12007);

  rokae_xmate3_ros2::srv::SetAO::Request ao_req;
  rokae_xmate3_ros2::srv::SetAO::Response ao_res;
  ao_req.board = 0;
  ao_req.port = 0;
  ao_req.value = 11.0;
  facade.handleSetAO(ao_req, ao_res);
  EXPECT_FALSE(ao_res.success);
  EXPECT_EQ(ao_res.message, "AO value must be within [0.0, 10.0]");
}
#endif
