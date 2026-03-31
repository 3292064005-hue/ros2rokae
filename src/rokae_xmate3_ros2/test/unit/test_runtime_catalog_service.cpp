#include <gtest/gtest.h>

#include "runtime/motion_options_state.hpp"
#include "runtime/program_state.hpp"
#include "runtime/session_state.hpp"
#include "runtime/runtime_catalog_service.hpp"
#include "runtime/tooling_state.hpp"

namespace rokae_xmate3_ros2::runtime {

TEST(RuntimeCatalogService, ActiveToolsetAndProjectAreSurfaced) {
  ToolingState tooling_state;
  tooling_state.setToolset("probe_tool", "patient_wobj", {0.1, 0.2, 0.3, 0.0, 0.1, 0.2}, {0.4, 0.5, 0.6, 0.0, 0.0, 0.0});
  tooling_state.setToolDynamics("probe_tool", 1.5, {0.0, 0.0, 0.02});

  ProgramState program_state;
  program_state.loadRlProject("/tmp/demo.rlproj", "demo_project");
  program_state.setRlProjectRunningOptions(0.75, true);
  program_state.setRlProjectRunning(true, 4);

  const auto tools = buildRuntimeToolCatalog(tooling_state);
  const auto wobjs = buildRuntimeWobjCatalog(tooling_state);
  const auto projects = buildRuntimeProjectCatalog(program_state);

  ASSERT_FALSE(tools.empty());
  ASSERT_FALSE(wobjs.empty());
  ASSERT_FALSE(projects.empty());

  EXPECT_EQ(tools.back().name, "probe_tool");
  EXPECT_TRUE(tools.back().active);
  EXPECT_DOUBLE_EQ(tools.back().mass, 1.5);

  EXPECT_EQ(wobjs.back().name, "patient_wobj");
  EXPECT_TRUE(wobjs.back().active);
  EXPECT_FALSE(wobjs.back().robot_held);

  EXPECT_EQ(projects.back().name, "demo_project");
  EXPECT_TRUE(projects.back().active);
  EXPECT_TRUE(projects.back().is_running);
  EXPECT_EQ(projects.back().current_episode, 4);
}

TEST(RuntimeCatalogService, BuildsRegisterNamespaceDescriptors) {
  DataStoreState data_store_state;
  data_store_state.setRegister("register0[0]", "1.0");
  data_store_state.setRegister("register0[1]", "2.0");
  data_store_state.setRegister("status_word", "ok");

  const auto catalog = buildRuntimeRegisterCatalog(data_store_state);
  ASSERT_EQ(catalog.size(), 2u);
  EXPECT_EQ(catalog.front().name, "register0");
  EXPECT_TRUE(catalog.front().indexed);
  EXPECT_EQ(catalog.front().element_count, 2u);
  EXPECT_EQ(catalog.back().name, "status_word");
  EXPECT_FALSE(catalog.back().indexed);
}


TEST(RuntimeCatalogService, BuildsRuntimeOptionDescriptors) {
  MotionOptionsState motion_options;
  SessionState session_state;
  motion_options.setDefaultSpeed(80.0);
  motion_options.setDefaultZone(15);
  motion_options.setDefaultConfOpt(true);
  motion_options.setAvoidSingularity(false);
  session_state.setSimulationMode(true);
  session_state.setMotionMode(1);
  session_state.setRtControlMode(2);

  const auto options = buildRuntimeOptionCatalog(motion_options, session_state);
  ASSERT_GE(options.size(), 8u);
  const auto summary = summarizeRuntimeOptionCatalog(options);
  EXPECT_NE(summary.find("default_zone=15"), std::string::npos);
  EXPECT_NE(summary.find("rt_control_mode=2"), std::string::npos);
}

}  // namespace rokae_xmate3_ros2::runtime
