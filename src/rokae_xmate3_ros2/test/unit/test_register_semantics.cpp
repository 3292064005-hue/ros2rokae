#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <string>

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
  data_store.setCustomData(rokae_xmate3_ros2::runtime::kRtJointImpedanceTopic, "100,200,300,40,50,60");
  data_store.setCustomData(rokae_xmate3_ros2::runtime::kRtFilterLimitTopic,
                           "limit_rate=true;cutoff_frequency=55");
  data_store.setCustomData(rokae_xmate3_ros2::runtime::kRtSemanticStateTopic,
                           "dispatch_mode=rt_loop;catalog_provenance=runtime_authoritative");

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

TEST(RegisterSemantics, IoProgramFacadeRejectsEmptyRegisterKeysAndNames) {
  rt::SessionState session_state;
  rt::DataStoreState data_store;
  rt::ProgramState program_state;
  rt::ToolingState tooling_state;
  rt::IoProgramFacade facade(session_state, data_store, program_state, tooling_state, [] { return std::chrono::steady_clock::now(); });

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
  rt::IoProgramFacade facade(session_state, data_store, program_state, tooling_state, [] { return std::chrono::steady_clock::now(); });

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
  rt::IoProgramFacade facade(session_state, data_store, program_state, tooling_state, [] { return std::chrono::steady_clock::now(); });

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
