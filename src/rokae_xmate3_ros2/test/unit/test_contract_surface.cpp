#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readText(const std::filesystem::path &path) {
  std::ifstream stream(path);
  EXPECT_TRUE(stream.is_open()) << "failed to open " << path;
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

const std::filesystem::path kProjectRoot = ROKAE_TEST_PROJECT_ROOT;

TEST(ContractSurface, PreferredAndLegacyContractsAreDocumented) {
  const auto matrix = readText(kProjectRoot / "docs" / "API_ALIGNMENT_MATRIX.md");
  EXPECT_NE(matrix.find("ReadRegisterEx"), std::string::npos);
  EXPECT_NE(matrix.find("ReadRegister"), std::string::npos);
  EXPECT_NE(matrix.find("Legacy facade"), std::string::npos);
  EXPECT_NE(matrix.find("GetEndWrench"), std::string::npos);
  EXPECT_NE(matrix.find("GetEndEffectorTorque"), std::string::npos);
}

TEST(ContractSurface, RuntimeAndKinematicsExpansionDocsExist) {
  const auto extension = readText(kProjectRoot / "docs" / "EXTENSION_FRAMEWORK.md");
  EXPECT_NE(extension.find("Runtime is the only state authority"), std::string::npos);
  EXPECT_NE(extension.find("KDL"), std::string::npos);
  EXPECT_NE(extension.find("improved_dh"), std::string::npos);

  const auto kinematics = readText(kProjectRoot / "docs" / "KINEMATICS_POLICY.md");
  EXPECT_NE(kinematics.find("single request must use exactly one primary backend"), std::string::npos);
}

TEST(ContractSurface, PreferredProfileFilesStaySplit) {
  const auto nrt = readText(kProjectRoot / "config" / "ros2_control_nrt.yaml");
  const auto rt = readText(kProjectRoot / "config" / "ros2_control_rt.yaml");
  EXPECT_NE(nrt.find("update_rate: 250"), std::string::npos);
  EXPECT_NE(rt.find("update_rate: 1000"), std::string::npos);
  EXPECT_NE(rt.find("SimApprox RT profile"), std::string::npos);
}

TEST(ContractSurface, ExpandedContractsExistOnDisk) {
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "ReadRegisterEx.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "WriteRegisterEx.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "GetEndWrench.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "GetRlProjectInfo.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "SetXPanelVout.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "GetRuntimeStateSnapshot.srv"));
}

TEST(ContractSurface, ReadmeStopsClaimingGenericIoParity) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("## xMate6 public contract hardening"), std::string::npos);
  EXPECT_NE(readme.find("GetEndWrench"), std::string::npos);
}

TEST(ContractSurface, RobotHeaderAgainTransitivelyIncludesPlannerHeader) {
  const auto robot_header = readText(kProjectRoot / "include" / "rokae" / "robot.h");
  EXPECT_NE(robot_header.find("#include \"rokae/planner.h\""), std::string::npos);
}

TEST(ContractSurface, ReadmeDocumentsSimulationGradePpToMain) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("simulation-grade `ppToMain()`"), std::string::npos);
  EXPECT_NE(readme.find("重新装载最近一次成功加载的工程路径"), std::string::npos);
}


TEST(ContractSurface, ReadmeDocumentsCalibrationAsUnsupportedCompatibilityStub) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("不支持任何坐标系标定功能"), std::string::npos);
  EXPECT_NE(readme.find("function_not_supported"), std::string::npos);
}

TEST(ContractSurface, CalibrationApiIsRetainedOnlyAsUnsupportedCompatibilityStub) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find("std::errc::function_not_supported"), std::string::npos);
  const auto robot_model = readText(kProjectRoot / "src" / "sdk" / "robot_model.cpp");
  EXPECT_NE(robot_model.find("function_not_supported"), std::string::npos);
}

TEST(ContractSurface, PlannerHeaderUsesStrictJerkLimitedProfileInsteadOfQuinticBlend) {
  const auto planner_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_planner.hpp");
  EXPECT_NE(planner_header.find("StrictJerkLimitedProfile"), std::string::npos);
  EXPECT_EQ(planner_header.find("kQuinticPeakVelocityScale"), std::string::npos);
}

TEST(ContractSurface, RuntimeRetimerUsesStrictJerkLimitedProfileInsteadOfQuinticBlend) {
  const auto retimer_cpp = readText(kProjectRoot / "src" / "runtime" / "unified_retimer.cpp");
  const auto retimer_joint_cpp = readText(kProjectRoot / "src" / "runtime" / "joint_retimer.cpp");
  EXPECT_NE(retimer_cpp.find("StrictJerkLimitedScalarProfile"), std::string::npos);
  EXPECT_EQ(retimer_cpp.find("sample_quintic_blend"), std::string::npos);
  EXPECT_EQ(retimer_joint_cpp.find("kQuinticPeakVelocityCoeff"), std::string::npos);
}

TEST(ContractSurface, PublicHeadersDoNotDependOnSourceOnlyRuntimeHeaders) {
  const auto planner_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_planner.hpp");
  EXPECT_EQ(planner_header.find("runtime/unified_retimer.hpp"), std::string::npos);
}

TEST(ContractSurface, CartMotionGeneratorUsesSynchronizedDeltaFromInitialArcLength) {
  const auto planner_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_planner.hpp");
  EXPECT_NE(planner_header.find("profile_.configure(s_goal_ - s_init_"), std::string::npos);
}

TEST(ContractSurface, ModelContextIsSharedBetweenSessionModelAndRtFacade) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find("session_->model_load_cache"), std::string::npos);
  EXPECT_NE(shim_header.find("session_->model_f_t_ee"), std::string::npos);
  EXPECT_NE(shim_header.find("syncContextFromSession()"), std::string::npos);
}

TEST(ContractSurface, ReadmeDocumentsForceFrameAndRciCompatibilitySemantics) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("setFcCoor()"), std::string::npos);
  EXPECT_NE(readme.find("useRciClient(true)"), std::string::npos);
  EXPECT_NE(readme.find("setRtNetworkTolerance()"), std::string::npos);
}

TEST(ContractSurface, RtNetworkToleranceNowValidatesRangeAndPublishesRuntimeConfig) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find("percent > 100u"), std::string::npos);
  EXPECT_NE(shim_header.find("kConfigRtNetworkTolerance"), std::string::npos);
}

TEST(ContractSurface, ToolsetQueriesDoNotSilentlyResetModelStiffnessFrame) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find("if (session_->model_ee_t_k == std::array<double, 16>{})"), std::string::npos);
}

TEST(ContractSurface, FollowPositionHasExceptionSafeRtLoopCleanup) {
  const auto planner_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_planner.hpp");
  EXPECT_NE(planner_header.find("controller->stopLoop();"), std::string::npos);
  EXPECT_NE(planner_header.find("controller->stopMove();"), std::string::npos);
  EXPECT_NE(planner_header.find("} catch (...) {"), std::string::npos);
}

TEST(ContractSurface, MoveJStartValidationUsesParameterException) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find(R"(throw_if_error<RealtimeParameterException>(ec, "RtMotionControl::MoveJ start validation failed"))"), std::string::npos);
}

TEST(ContractSurface, SixAxisRtSurfaceNoLongerAdvertisesElbowPlaceholders) {
  const auto registry = readText(kProjectRoot / "src" / "runtime" / "rt_field_registry.cpp");
  const auto rt_cpp = readText(kProjectRoot / "src" / "sdk" / "robot_rt.cpp");
  const auto example = readText(kProjectRoot / "examples" / "cpp" / "17_state_stream_cache.cpp");
  EXPECT_EQ(registry.find("synthetic_placeholder"), std::string::npos);
  EXPECT_EQ(registry.find("RtSupportedFields::elbow_m"), std::string::npos);
  EXPECT_EQ(rt_cpp.find("RtSupportedFields::elbow_m"), std::string::npos);
  EXPECT_EQ(example.find("RtSupportedFields::elbow_m"), std::string::npos);
}


TEST(ContractSurface, ShimSdkSurfaceNowExposesStartReceiveRobotStateAndDelegatesToNativeStrictRtCache) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find("void startReceiveRobotState(std::chrono::steady_clock::duration interval,"), std::string::npos);
  EXPECT_NE(shim_header.find("session_->robot->startReceiveRobotState(interval, fields);"), std::string::npos);
  EXPECT_NE(shim_header.find("session_->robot->updateRobotState(timeout);"), std::string::npos);
  EXPECT_NE(shim_header.find("getStateDataMatrix16"), std::string::npos);
}

TEST(ContractSurface, ShimRtStateRefreshNoLongerFallsBackToNrtSnapshots) {
  const auto shim_header = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(shim_header.find("getRtJointData(joints, joint_vel, joint_tau, ec)"), std::string::npos);
  EXPECT_EQ(shim_header.find("jointPos(ec);"), std::string::npos);
  EXPECT_EQ(shim_header.find("jointVel(ec);"), std::string::npos);
  EXPECT_EQ(shim_header.find("jointTorques(ec);"), std::string::npos);
}

TEST(ContractSurface, NativeRtStateCacheExposesMatrix16GetterForShimReuse) {
  const auto robot_header = readText(kProjectRoot / "include" / "rokae_xmate3_ros2" / "robot.hpp");
  const auto state_cache_cpp = readText(kProjectRoot / "src" / "sdk" / "robot_state_cache.cpp");
  EXPECT_NE(robot_header.find("getStateDataMatrix16"), std::string::npos);
  EXPECT_NE(state_cache_cpp.find("getStateDataMatrix16"), std::string::npos);
}

TEST(ContractSurface, NativeSdkFacadeTracksLastErrorAcrossPublicErrorCodeEntrypoints) {
  const auto internal = readText(kProjectRoot / "src" / "sdk" / "robot_internal.hpp");
  const auto connection = readText(kProjectRoot / "src" / "sdk" / "robot_connection.cpp");
  const auto motion = readText(kProjectRoot / "src" / "sdk" / "robot_motion.cpp");
  const auto project = readText(kProjectRoot / "src" / "sdk" / "robot_project.cpp");
  const auto rt = readText(kProjectRoot / "src" / "sdk" / "robot_rt.cpp");
  EXPECT_NE(internal.find("class ScopedLastError final"), std::string::npos);
  EXPECT_NE(internal.find("remember_last_error"), std::string::npos);
  EXPECT_NE(connection.find("track_last_error(impl_, ec)"), std::string::npos);
  EXPECT_NE(motion.find("track_last_error(impl_, ec)"), std::string::npos);
  EXPECT_NE(project.find("track_last_error(impl_, ec)"), std::string::npos);
  EXPECT_NE(rt.find("track_last_error(impl_, ec)"), std::string::npos);
}

}  // namespace


TEST(ContractSurface, InstallFacingConfigPublishesNativeStaticProvider) {
  const auto config = readText(kProjectRoot / "cmake" / "xCoreSDKConfig.cmake.in");
  EXPECT_NE(config.find("xCoreSDK_STATIC_PROVIDER \"native-static\""), std::string::npos);
  EXPECT_NE(config.find("find_dependency(rclcpp REQUIRED CONFIG)"), std::string::npos);
  EXPECT_NE(config.find("find_dependency(gazebo_ros REQUIRED CONFIG)"), std::string::npos);
  EXPECT_EQ(config.find("shared-alias"), std::string::npos);
}

TEST(ContractSurface, CompatRtMoveCSurfaceNowImplementsGeometricArcExecution) {
  const auto rt_cpp = readText(kProjectRoot / "src" / "compat" / "rt_api.cpp");
  EXPECT_NE(rt_cpp.find("RtMotionControlCobot::MoveC invalid radius"), std::string::npos);
  EXPECT_NE(rt_cpp.find("RtMotionControlCobot::MoveC degenerate arc"), std::string::npos);
  EXPECT_NE(rt_cpp.find("slerp(alpha, q_target)"), std::string::npos);
  EXPECT_EQ(rt_cpp.find("function_not_supported"), std::string::npos);
}

TEST(ContractSurface, RtConfigSurfaceValidatesRangesBeforePublishing) {
  const auto rt_cpp = readText(kProjectRoot / "src" / "compat" / "rt_api.cpp");
  EXPECT_NE(rt_cpp.find("cutoff_frequency < 0.0 || cutoff_frequency > 1000.0"), std::string::npos);
  EXPECT_NE(rt_cpp.find("jointFrequency < 1.0 || jointFrequency > 1000.0"), std::string::npos);
  EXPECT_NE(rt_cpp.find("std::fabs(torque[i]) > kDesiredWrenchLimit[i]"), std::string::npos);
  EXPECT_NE(rt_cpp.find("valid_force_control_type(type)"), std::string::npos);
  EXPECT_NE(rt_cpp.find("std::make_error_code(std::errc::invalid_argument)"), std::string::npos);
}

TEST(ContractSurface, CompatRtMetadataFieldsAreDocumentedAndProducedByTheNativeStateCache) {
  const auto readme = readText(kProjectRoot / "README.md");
  const auto abi_doc = readText(kProjectRoot / "docs" / "COMPAT_ABI.md");
  const auto data_types = readText(kProjectRoot / "include" / "rokae" / "data_types.h");
  const auto registry = readText(kProjectRoot / "src" / "runtime" / "rt_field_registry.cpp");
  const auto rt_cpp = readText(kProjectRoot / "src" / "sdk" / "robot_rt.cpp");
  const auto shim = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(data_types.find("namespace RtCompatFields"), std::string::npos);
  EXPECT_NE(data_types.find("samplePeriod_s"), std::string::npos);
  EXPECT_NE(data_types.find("sampleFresh"), std::string::npos);
  EXPECT_NE(registry.find("RtCompatFields::samplePeriod_s"), std::string::npos);
  EXPECT_NE(registry.find("RtCompatFields::sampleFresh"), std::string::npos);
  EXPECT_NE(rt_cpp.find("RtCompatFields::samplePeriod_s"), std::string::npos);
  EXPECT_NE(rt_cpp.find("RtCompatFields::sampleFresh"), std::string::npos);
  EXPECT_NE(shim.find("RtCompatFields::samplePeriod_s"), std::string::npos);
  EXPECT_NE(shim.find("RtCompatFields::sampleFresh"), std::string::npos);
  EXPECT_NE(readme.find("RtCompatFields::samplePeriod_s"), std::string::npos);
  EXPECT_NE(abi_doc.find("RtCompatFields::samplePeriod_s"), std::string::npos);
}

TEST(ContractSurface, SoftLimitPublicSurfaceRetainsOfficialDefaultSentinelSemantics) {
  const auto robot_header = readText(kProjectRoot / "include" / "rokae" / "robot.h");
  const auto backend_header = readText(kProjectRoot / "include" / "rokae_xmate3_ros2" / "robot.hpp");
  const auto model_cpp = readText(kProjectRoot / "src" / "sdk" / "robot_model.cpp");
  EXPECT_NE(robot_header.find("{{DBL_MAX, DBL_MAX}}"), std::string::npos);
  EXPECT_NE(backend_header.find("{{DBL_MAX, DBL_MAX}}"), std::string::npos);
  EXPECT_NE(model_cpp.find("use_existing_limits"), std::string::npos);
  EXPECT_NE(model_cpp.find("getSoftLimit(current_limits, ec)"), std::string::npos);
}

TEST(ContractSurface, CompatInstallTreeSkeletonExists) {
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "docs" / "COMPAT_ABI.md"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "CMakeLists.txt"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_connect.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_connect_overload.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_model.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_robot_t.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_soft_limit.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_execute_movec.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_rt_header.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_planner.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_static_link_only.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "test" / "compat" / "install_tree" / "minimal_shared_link_only.cpp"));
}

TEST(ContractSurface, ReadmeAndQuickstartDocumentInstallFacingCompatSdk) {
  const auto readme = readText(kProjectRoot / "README.md");
  const auto quickstart = readText(kProjectRoot / "docs" / "QUICKSTART.md");
  EXPECT_NE(readme.find("xCoreSDK::xCoreSDK_shared"), std::string::npos);
  EXPECT_NE(readme.find("xCoreSDK::xCoreSDK_static"), std::string::npos);
  EXPECT_NE(readme.find("ABI 兼容安装面"), std::string::npos);
  EXPECT_NE(readme.find("ROS2/Gazebo-backed"), std::string::npos);
  EXPECT_NE(readme.find("真实静态目标"), std::string::npos);
  EXPECT_NE(readme.find("配置期同样需要系统可见的 ROS2/Gazebo 依赖"), std::string::npos);
  EXPECT_NE(quickstart.find("find_package(xCoreSDK CONFIG REQUIRED)"), std::string::npos);
  EXPECT_NE(quickstart.find("xCoreSDK::xCoreSDK_static"), std::string::npos);
  EXPECT_NE(quickstart.find("xCoreSDK::xCoreSDK_shared"), std::string::npos);
  EXPECT_NE(quickstart.find("rokae/sdk_shim*.hpp"), std::string::npos);
  EXPECT_NE(quickstart.find("ROS2/Gazebo-backed"), std::string::npos);
}

TEST(ContractSurface, CompatAbiDocDocumentsRosBackedConsumerConstraint) {
  const auto abi_doc = readText(kProjectRoot / "docs" / "COMPAT_ABI.md");
  EXPECT_NE(abi_doc.find("ROS2/Gazebo-backed"), std::string::npos);
  EXPECT_NE(abi_doc.find("xCoreSDK::xCoreSDK_shared"), std::string::npos);
  EXPECT_NE(abi_doc.find("xCoreSDK::xCoreSDK_static"), std::string::npos);
  EXPECT_NE(abi_doc.find("runtime execution still requires"), std::string::npos);
  EXPECT_NE(abi_doc.find("native static library"), std::string::npos);
  EXPECT_NE(abi_doc.find("not part of the public header contract"), std::string::npos);
}

TEST(ContractSurface, ExportedSymbolHarnessChecksDynamicTableOnly) {
  const auto harness = readText(kProjectRoot / "test" / "harness" / "check_exported_symbols.py");
  EXPECT_NE(harness.find("nm', '-D', '-C'"), std::string::npos);
}


TEST(ContractSurface, EnvironmentLockPreflightScriptIsWired) {
  const auto quick_gate = readText(kProjectRoot / "tools" / "run_quick_gate.sh");
  const auto release_gate = readText(kProjectRoot / "tools" / "run_release_gate.sh");
  const auto target_acceptance = readText(kProjectRoot / "tools" / "run_target_env_acceptance.sh");
  const auto env_lock = readText(kProjectRoot / "docs" / "ENVIRONMENT_LOCK.md");
  EXPECT_NE(quick_gate.find("check_target_environment.sh\" --quiet"), std::string::npos);
  EXPECT_NE(release_gate.find("check_target_environment.sh\" --quiet"), std::string::npos);
  EXPECT_NE(target_acceptance.find("check_target_environment.sh --quiet"), std::string::npos);
  EXPECT_NE(target_acceptance.find("write_target_env_report.py"), std::string::npos);
  EXPECT_NE(target_acceptance.find("acceptance_report_container.json"), std::string::npos);
  EXPECT_NE(env_lock.find("tools/check_target_environment.sh"), std::string::npos);
  EXPECT_NE(env_lock.find("write_target_env_report.py"), std::string::npos);
  EXPECT_NE(env_lock.find("artifacts/target_env_acceptance/"), std::string::npos);
}



TEST(ContractSurface, AcceptanceWorkflowUploadsReportArtifact) {
  const auto workflow = readText(kProjectRoot / ".github" / "workflows" / "acceptance-humble-gazebo11.yml");
  EXPECT_NE(workflow.find("upload-artifact@v4"), std::string::npos);
  EXPECT_NE(workflow.find("acceptance-humble-gazebo11-report"), std::string::npos);
  EXPECT_NE(workflow.find("--report-dir"), std::string::npos);
}

TEST(ContractSurface, ServiceContractManifestCentralizesPrimaryAndCompatibilitySurfaces) {
  const auto manifest = readText(kProjectRoot / "src" / "runtime" / "service_contract_manifest.hpp");
  EXPECT_NE(manifest.find("ROKAE_PUBLIC_XMATE6_PRIMARY_SERVICE_CONTRACTS"), std::string::npos);
  EXPECT_NE(manifest.find("ROKAE_INTERNAL_BACKEND_PRIMARY_SERVICE_CONTRACTS"), std::string::npos);
  EXPECT_NE(manifest.find("ROKAE_COMPATIBILITY_ALIAS_CONTRACTS"), std::string::npos);
  EXPECT_NE(manifest.find("/xmate3/internal/get_runtime_state_snapshot"), std::string::npos);
}

TEST(ContractSurface, ReadmeDocumentsCanonicalUrdfAndRuntimeSnapshot) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("xMate3.description.json"), std::string::npos);
  EXPECT_NE(readme.find("/xmate3/internal/get_runtime_state_snapshot"), std::string::npos);
  EXPECT_NE(readme.find("service_contract_manifest.hpp"), std::string::npos);
}


TEST(ContractSurface, LaunchEntryPointsGateDeveloperOnlyNonCanonicalOverrides) {
  const auto sim_alias = readText(kProjectRoot / "launch" / "xmate3_simulation.launch.py");
  const auto gazebo_alias = readText(kProjectRoot / "launch" / "xmate3_gazebo.launch.py");
  const auto rviz_only = readText(kProjectRoot / "launch" / "rviz_only.launch.py");
  const auto render_helper = readText(kProjectRoot / "tools" / "render_robot_description.py");
  const auto smoke = readText(kProjectRoot / "tools" / "run_launch_smoke.sh");
  EXPECT_NE(sim_alias.find("allow_noncanonical_model"), std::string::npos);
  EXPECT_NE(gazebo_alias.find("allow_noncanonical_model"), std::string::npos);
  EXPECT_NE(rviz_only.find("allow_noncanonical_model"), std::string::npos);
  EXPECT_NE(render_helper.find("non-canonical model override is disabled by default"), std::string::npos);
  EXPECT_NE(smoke.find("allow-noncanonical-model false"), std::string::npos);
  EXPECT_NE(smoke.find("allow-noncanonical-model true"), std::string::npos);
}


TEST(ContractSurface, CompatHarnessDocumentsInstallTreeAndSymbolChecks) {
  const auto compat_doc = readText(kProjectRoot / "docs" / "COMPAT_ABI.md");
  EXPECT_NE(compat_doc.find("run_install_tree_consumer.py"), std::string::npos);
  EXPECT_NE(compat_doc.find("check_exported_symbols.py"), std::string::npos);
}

TEST(ContractSurface, ExamplesAreSplitBetweenPublicCompatAndInternalBackendGroups) {
  const auto examples_cmake = readText(kProjectRoot / "cmake" / "targets_examples.cmake");
  const auto examples_readme = readText(kProjectRoot / "examples" / "README.md");
  EXPECT_NE(examples_cmake.find("ROKAE_PUBLIC_COMPAT_EXAMPLES"), std::string::npos);
  EXPECT_NE(examples_cmake.find("ROKAE_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
  EXPECT_NE(examples_readme.find("Public compat examples"), std::string::npos);
  EXPECT_NE(examples_readme.find("Internal/backend examples"), std::string::npos);
}

TEST(ContractSurface, PackagingOnlyInstallsPublicExamplesByDefault) {
  const auto root = readText(kProjectRoot / "CMakeLists.txt");
  const auto packaging = readText(kProjectRoot / "cmake" / "targets_packaging.cmake");
  EXPECT_NE(root.find("ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
  EXPECT_NE(packaging.find("${PUBLIC_COMPAT_EXAMPLE_TARGETS}"), std::string::npos);
  EXPECT_NE(packaging.find("ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
}

TEST(ContractSurface, InstallFacingPackagingSeparatesPrivateBackendExportSet) {
  const auto packaging = readText(kProjectRoot / "cmake" / "targets_packaging.cmake");
  const auto config = readText(kProjectRoot / "cmake" / "xCoreSDKConfig.cmake.in");
  EXPECT_NE(packaging.find("EXPORT xCoreSDKTargets"), std::string::npos);
  EXPECT_NE(packaging.find("EXPORT xCoreSDKPrivateTargets"), std::string::npos);
  EXPECT_NE(config.find("xCoreSDKPrivateTargets.cmake"), std::string::npos);
}


TEST(ContractSurface, InstallFacingCompatTargetsUseBuildAndInstallInterfaceIncludes) {
  const auto compat = readText(kProjectRoot / "cmake" / "targets_sdk_compat.cmake");
  EXPECT_NE(compat.find("$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"), std::string::npos);
  EXPECT_NE(compat.find("$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"), std::string::npos);
  EXPECT_NE(compat.find("src/runtime/rt_command_bridge.cpp"), std::string::npos);
}

TEST(ContractSurface, InstallFacingPackagingExportsIncludeDestinationForSdkTargets) {
  const auto packaging = readText(kProjectRoot / "cmake" / "targets_packaging.cmake");
  EXPECT_NE(packaging.find("INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}"), std::string::npos);
}

TEST(ContractSurface, CompatRtSurfaceUsesUnifiedRuntimeCommandBridgeAndPoseToleranceHelpers) {
  const auto rt_cpp = readText(kProjectRoot / "src" / "compat" / "rt_api.cpp");
  const auto bridge = readText(kProjectRoot / "src" / "runtime" / "rt_command_bridge.cpp");
  const auto shim = readText(kProjectRoot / "include" / "rokae" / "detail" / "sdk_shim_core.hpp");
  EXPECT_NE(rt_cpp.find("rt_command_bridge::publishCommand"), std::string::npos);
  EXPECT_NE(rt_cpp.find("cartesianPoseWithinTolerance"), std::string::npos);
  EXPECT_NE(shim.find("rt_command_bridge::publishCommand"), std::string::npos);
  EXPECT_NE(bridge.find("publishCommand"), std::string::npos);
}
