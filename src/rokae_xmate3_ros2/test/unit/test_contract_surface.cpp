#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
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
  const auto matrix = readText(kProjectRoot / "docs" / "reference" / "SDK_ALIGNMENT.md");
  EXPECT_NE(matrix.find("ReadRegisterEx"), std::string::npos);
  EXPECT_NE(matrix.find("ReadRegister"), std::string::npos);
  EXPECT_NE(matrix.find("Legacy facade"), std::string::npos);
  EXPECT_NE(matrix.find("GetEndWrench"), std::string::npos);
  EXPECT_NE(matrix.find("GetEndEffectorTorque"), std::string::npos);
}

TEST(ContractSurface, RuntimeAndKinematicsExpansionDocsExist) {
  const auto extension = readText(kProjectRoot / "docs" / "architecture" / "PROVIDER_BOUNDARY.md");
  EXPECT_NE(extension.find("RuntimeBackendProviderHost -> RuntimeBackendProvider -> BackendInterface"), std::string::npos);
  EXPECT_NE(extension.find("generic backend factory request"), std::string::npos);

  const auto kinematics = readText(kProjectRoot / "docs" / "public" / "KINEMATICS_AND_MODEL.md");
  EXPECT_NE(kinematics.find("single request must use exactly one primary backend"), std::string::npos);
  EXPECT_NE(kinematics.find("KDL"), std::string::npos);
  EXPECT_NE(kinematics.find("improved_dh"), std::string::npos);
}

TEST(ContractSurface, PreferredProfileFilesStaySplit) {
  const auto nrt = readText(kProjectRoot / "config" / "ros2_control_nrt.yaml");
  const auto rt = readText(kProjectRoot / "config" / "ros2_control_rt.yaml");
  EXPECT_NE(nrt.find("update_rate: 250"), std::string::npos);
  EXPECT_NE(rt.find("update_rate: 1000"), std::string::npos);
  EXPECT_NE(rt.find("SimApprox RT profile"), std::string::npos);
}

TEST(ContractSurface, ExpandedContractsExistOnDisk) {
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "internal_interfaces" / "srv" / "ReadRegisterEx.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "internal_interfaces" / "srv" / "WriteRegisterEx.srv"));
  EXPECT_FALSE(std::filesystem::exists(kProjectRoot / "srv" / "WriteRegisterEx.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "GetEndWrench.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "internal_interfaces" / "srv" / "GetRlProjectInfo.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "internal_interfaces" / "srv" / "SetXPanelVout.srv"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "srv" / "GetRuntimeStateSnapshot.srv"));
}

TEST(ContractSurface, ReadmeStopsClaimingGenericIoParity) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("public lane 只承诺"), std::string::npos);
  EXPECT_NE(readme.find("不承诺坐标系标定、RL 工程、通用 IO"), std::string::npos);
  EXPECT_NE(readme.find("GetEndWrench"), std::string::npos);
  EXPECT_NE(readme.find("MoveAppend"), std::string::npos);
}

TEST(ContractSurface, RobotHeaderAgainTransitivelyIncludesPlannerHeader) {
  const auto robot_header = readText(kProjectRoot / "include" / "rokae" / "robot.h");
  EXPECT_NE(robot_header.find("#include \"rokae/planner.h\""), std::string::npos);
}

TEST(ContractSurface, AuditDocumentsSimulationGradePpToMain) {
  const auto audit = readText(kProjectRoot / "docs" / "archive" / "audits" / "IMPLEMENTATION_AUDIT.md");
  EXPECT_NE(audit.find("Simulation-grade"), std::string::npos);
  EXPECT_NE(audit.find("ppToMain()"), std::string::npos);
  EXPECT_NE(audit.find("last successfully loaded project path"), std::string::npos);
}


TEST(ContractSurface, ReadmeDocumentsCalibrationAsUnsupportedCompatibilityStub) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("坐标系标定类接口只保留兼容签名"), std::string::npos);
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
  const auto audit = readText(kProjectRoot / "docs" / "archive" / "audits" / "IMPLEMENTATION_AUDIT.md");
  EXPECT_NE(audit.find("setFcCoor()"), std::string::npos);
  EXPECT_NE(audit.find("useRciClient(true)"), std::string::npos);
  EXPECT_NE(audit.find("setRtNetworkTolerance()"), std::string::npos);
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

TEST(ContractSurface, SixAxisRtSurfaceNoLongerAdvertisesElbowFields) {
  const auto registry = readText(kProjectRoot / "src" / "runtime" / "rt_field_registry.cpp");
  const auto rt_cpp = readText(kProjectRoot / "src" / "sdk" / "robot_rt.cpp");
  const auto example = readText(kProjectRoot / "examples" / "cpp" / "17_state_stream_cache.cpp");
  EXPECT_EQ(registry.find(std::string("synthetic_") + "marker"), std::string::npos);
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
  const auto update_begin = shim_header.find("unsigned updateRobotState(std::chrono::steady_clock::duration timeout)");
  ASSERT_NE(update_begin, std::string::npos);
  const auto update_end = shim_header.find("template <typename R>", update_begin);
  ASSERT_NE(update_end, std::string::npos);
  const auto update_body = shim_header.substr(update_begin, update_end - update_begin);
  EXPECT_EQ(update_body.find("jointPos(ec);"), std::string::npos);
  EXPECT_EQ(update_body.find("jointVel(ec);"), std::string::npos);
  EXPECT_EQ(update_body.find("jointTorques(ec);"), std::string::npos);
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



TEST(ContractSurface, TextSourceFilesDoNotContainEmbeddedNulBytes) {
  static constexpr const char *kExtensions[] = {
      ".cpp", ".hpp", ".h", ".ipp", ".py", ".sh", ".cmake", ".md", ".srv", ".msg", ".action", ".launch.py", ".xacro", ".xml", ".yaml", ".yml"};

  for (const auto &entry : std::filesystem::recursive_directory_iterator(kProjectRoot)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto path = entry.path();
    const auto filename = path.filename().string();
    if (filename == ".gitkeep") {
      continue;
    }
    const auto extension = path.extension().string();
    bool should_check = false;
    for (const auto *candidate : kExtensions) {
      const std::string suffix(candidate);
      if (suffix == ".launch.py") {
        if (filename.size() >= suffix.size() &&
            filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0) {
          should_check = true;
          break;
        }
        continue;
      }
      if (extension == suffix) {
        should_check = true;
        break;
      }
    }
    if (!should_check) {
      continue;
    }

    std::ifstream stream(path, std::ios::binary);
    ASSERT_TRUE(stream.is_open()) << "failed to open " << path;
    const std::string payload((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    EXPECT_EQ(payload.find('\0'), std::string::npos) << "embedded NUL in " << path;
  }
}

TEST(ContractSurface, InstallFacingConfigPublishesNativeStaticProvider) {
  const auto config = readText(kProjectRoot / "cmake" / "xCoreSDKConfig.cmake.in");
  EXPECT_NE(config.find("xCoreSDK_STATIC_PROVIDER \"native-static\""), std::string::npos);
  EXPECT_NE(config.find("find_dependency(rclcpp REQUIRED CONFIG)"), std::string::npos);
  EXPECT_EQ(config.find("find_dependency(gazebo_ros REQUIRED CONFIG)"), std::string::npos);
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
  const auto audit = readText(kProjectRoot / "docs" / "archive" / "audits" / "IMPLEMENTATION_AUDIT.md");
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
  EXPECT_NE(audit.find("RtCompatFields::samplePeriod_s"), std::string::npos);
  EXPECT_NE(audit.find("RtCompatFields::sampleFresh"), std::string::npos);
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
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "docs" / "reference" / "SDK_ALIGNMENT.md"));
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

TEST(ContractSurface, ReadmeAndQuickstartDocumentCorePrimaryAndExplicitRuntimeBridgeTargets) {
  const auto readme = readText(kProjectRoot / "README.md");
  const auto quickstart = readText(kProjectRoot / "docs" / "public" / "QUICKSTART.md");
  EXPECT_NE(readme.find("xCoreSDK::xCoreSDK_shared"), std::string::npos);
  EXPECT_NE(readme.find("xCoreSDK::xCoreSDK_core"), std::string::npos);
  EXPECT_NE(readme.find("默认 install-facing C++ core SDK 入口"), std::string::npos);
  EXPECT_NE(readme.find("ROS bridge"), std::string::npos);
  EXPECT_NE(quickstart.find("find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)"), std::string::npos);
  EXPECT_NE(quickstart.find("xCoreSDK::xCoreSDK_shared"), std::string::npos);
  EXPECT_NE(quickstart.find("xCoreSDK::xCoreSDK_core"), std::string::npos);
  EXPECT_NE(quickstart.find("rokae/sdk_shim*.hpp"), std::string::npos);
  EXPECT_NE(quickstart.find("ROS2/Gazebo-backed"), std::string::npos);
}

TEST(ContractSurface, CompatAbiDocDocumentsPrimaryRosBridgeCoreOnlyAndCompatTargets) {
  const auto abi_doc = readText(kProjectRoot / "docs" / "reference" / "SDK_ALIGNMENT.md");
  EXPECT_NE(abi_doc.find("ROS2/Gazebo-backed"), std::string::npos);
  EXPECT_NE(abi_doc.find("xCoreSDK::xCoreSDK_core"), std::string::npos);
  EXPECT_NE(abi_doc.find("xCoreSDK::xCoreSDK_ros_bridge"), std::string::npos);
  EXPECT_NE(abi_doc.find("xCoreSDK::xCoreSDK_static"), std::string::npos);
  EXPECT_NE(abi_doc.find("xCoreSDK_PRIMARY_INSTALL_CONSUMER = cxx_sdk_core_consumer"), std::string::npos);
  EXPECT_NE(abi_doc.find("runtime execution still requires"), std::string::npos);
  EXPECT_NE(abi_doc.find("native static library"), std::string::npos);
  EXPECT_NE(abi_doc.find("not installed as part of the public SDK surface"), std::string::npos);
}

TEST(ContractSurface, ExportedSymbolHarnessChecksDynamicTableOnly) {
  const auto harness = readText(kProjectRoot / "test" / "harness" / "check_exported_symbols.py");
  EXPECT_NE(harness.find("nm', '-D', '-C'"), std::string::npos);
}


TEST(ContractSurface, EnvironmentLockPreflightScriptIsWired) {
  const auto quick_gate = readText(kProjectRoot / "tools" / "run_quick_gate.sh");
  const auto release_gate = readText(kProjectRoot / "tools" / "run_release_gate.sh");
  const auto full_source_gate = readText(kProjectRoot / "tools" / "run_full_source_tree_build_gate.sh");
  const auto portable_release_gate = readText(kProjectRoot / "tools" / "run_release_gate_portable.sh");
  const auto target_acceptance = readText(kProjectRoot / "tools" / "run_target_env_acceptance.sh");
  const auto env_lock = readText(kProjectRoot / "docs" / "release" / "ENVIRONMENT_LOCK.md");
  EXPECT_NE(quick_gate.find("run_full_source_tree_build_gate.sh"), std::string::npos);
  EXPECT_NE(release_gate.find("run_full_source_tree_build_gate.sh"), std::string::npos);
  EXPECT_NE(full_source_gate.find("check_target_environment.sh"), std::string::npos);
  EXPECT_NE(full_source_gate.find("--quiet"), std::string::npos);
  EXPECT_NE(portable_release_gate.find("run_target_env_acceptance.sh"), std::string::npos);
  EXPECT_NE(portable_release_gate.find("run_release_gate.sh"), std::string::npos);
  EXPECT_NE(target_acceptance.find("check_target_environment.sh"), std::string::npos);
  EXPECT_NE(target_acceptance.find("write_target_env_report.py"), std::string::npos);
  EXPECT_NE(target_acceptance.find("acceptance_report_container.json"), std::string::npos);
  EXPECT_NE(env_lock.find("tools/check_target_environment.sh"), std::string::npos);
  EXPECT_NE(env_lock.find("write_target_env_report.py"), std::string::npos);
  EXPECT_NE(env_lock.find("artifacts/target_env_acceptance/"), std::string::npos);
}

TEST(ContractSurface, RuntimeDiagGateIsExternalizedAndCalibratable) {
  const auto smoke = readText(kProjectRoot / "tools" / "run_main_chain_smoke.sh");
  const auto gate = readText(kProjectRoot / "tools" / "check_runtime_diag_gate.py");
  const auto derive = readText(kProjectRoot / "tools" / "derive_runtime_diag_gate.py");
  const auto quickstart = readText(kProjectRoot / "docs" / "public" / "QUICKSTART.md");
  EXPECT_NE(smoke.find("runtime_diag_gate.default.json"), std::string::npos);
  EXPECT_NE(smoke.find("ROKAE_RT_GATE_LIMITS_FILE"), std::string::npos);
  EXPECT_NE(gate.find("--limits-file"), std::string::npos);
  EXPECT_NE(gate.find("--print-effective-limits"), std::string::npos);
  EXPECT_NE(derive.find("Derive a reusable runtime-diagnostics gate profile"), std::string::npos);
  EXPECT_NE(quickstart.find("derive_runtime_diag_gate.py"), std::string::npos);
}

TEST(ContractSurface, AcceptanceWorkflowUploadsReportArtifact) {
  const auto workflow = readText(kProjectRoot / ".github" / "workflows" / "acceptance-humble-gazebo11.yml");
  const auto gates_workflow = readText(kProjectRoot / ".github" / "workflows" / "gates.yml");
  EXPECT_NE(workflow.find("upload-artifact@v4"), std::string::npos);
  EXPECT_NE(workflow.find("acceptance-humble-gazebo11-report"), std::string::npos);
  EXPECT_NE(workflow.find("--report-dir"), std::string::npos);
  EXPECT_NE(gates_workflow.find("run_target_env_acceptance.sh --release-gate --launch-smoke"), std::string::npos);
  EXPECT_NE(gates_workflow.find("release-gate-report"), std::string::npos);
}

TEST(ContractSurface, ServiceContractManifestCentralizesPrimaryAndCompatibilitySurfaces) {
  const auto manifest_hpp = readText(kProjectRoot / "src" / "runtime" / "service_contract_manifest.hpp");
  const auto manifest_cpp = readText(kProjectRoot / "src" / "runtime" / "service_contract_manifest.cpp");
  EXPECT_NE(manifest_hpp.find("buildPublicPrimaryServiceContractManifest"), std::string::npos);
  EXPECT_NE(manifest_hpp.find("buildPublicCompatibilityAliasContractManifest"), std::string::npos);
  EXPECT_NE(manifest_cpp.find("buildInternalPrimaryServiceContractManifest"), std::string::npos);
  EXPECT_NE(manifest_cpp.find("appendServiceDescriptor"), std::string::npos);
  EXPECT_NE(manifest_cpp.find("/xmate_er3/cobot/get_runtime_state_snapshot"), std::string::npos);
}

TEST(ContractSurface, ReadmeDocumentsCanonicalUrdfAndRuntimeSnapshot) {
  const auto readme = readText(kProjectRoot / "README.md");
  const auto xacro = readText(kProjectRoot / "urdf" / "xMateER3.xacro");
  const auto metadata = readText(kProjectRoot / "tools" / "generate_description_metadata.py");
  EXPECT_NE(readme.find("xMateER3.description.json"), std::string::npos);
  EXPECT_NE(readme.find("xMate3.description.json"), std::string::npos);
  EXPECT_NE(readme.find("/xmate_er3/cobot/get_runtime_state_snapshot"), std::string::npos);
  EXPECT_NE(readme.find("service_contract_manifest.hpp"), std::string::npos);
  EXPECT_NE(readme.find("service_contract_manifest.cpp"), std::string::npos);
  EXPECT_NE(xacro.find("service_exposure_profile"), std::string::npos);
  EXPECT_NE(metadata.find("source_xacro_package_relative"), std::string::npos);
  EXPECT_NE(metadata.find("service_exposure_profile"), std::string::npos);
}


TEST(ContractSurface, RuntimeHostBuilderCentralizesDaemonAndGazeboLifecycleAssembly) {
  const auto host_builder = readText(kProjectRoot / "src" / "runtime" / "runtime_host_builder.hpp");
  const auto host_builder_impl = readText(kProjectRoot / "src" / "runtime" / "runtime_host_builder.cpp");
  const auto sim_main = readText(kProjectRoot / "src" / "runtime" / "sim_runtime_main.cpp");
  const auto gazebo_bootstrap = readText(kProjectRoot / "src" / "gazebo" / "runtime_bootstrap.cpp");
  const auto profiles = readText(kProjectRoot / "docs" / "public" / "RUNTIME_PROFILES.md");
  EXPECT_NE(host_builder.find("Single builder for runtime-host bootstrap, assembly and executor lifecycle"), std::string::npos);
  EXPECT_NE(host_builder_impl.find("createPublishBridge"), std::string::npos);
  EXPECT_NE(host_builder_impl.find("createRosBindings"), std::string::npos);
  EXPECT_NE(host_builder_impl.find("createPublishTimer"), std::string::npos);
  EXPECT_NE(host_builder_impl.find("attachExecutor"), std::string::npos);
  EXPECT_NE(host_builder_impl.find("releaseExecutor"), std::string::npos);
  EXPECT_NE(sim_main.find("RuntimeHostBuilder host_builder(node)"), std::string::npos);
  EXPECT_NE(sim_main.find("createPublishTimer"), std::string::npos);
  EXPECT_NE(sim_main.find("attachExecutor(nullptr, false"), std::string::npos);
  EXPECT_NE(gazebo_bootstrap.find("host_builder_ = std::make_unique<runtime::RuntimeHostBuilder>(node_);"), std::string::npos);
  EXPECT_NE(gazebo_bootstrap.find("host_builder_->createPublishBridge"), std::string::npos);
  EXPECT_NE(gazebo_bootstrap.find("host_builder_->createRosBindings"), std::string::npos);
  EXPECT_NE(gazebo_bootstrap.find("host_builder_->createPublishTimer"), std::string::npos);
  EXPECT_NE(gazebo_bootstrap.find("host_builder_->attachExecutor"), std::string::npos);
  EXPECT_NE(gazebo_bootstrap.find("host_builder_->releaseExecutor"), std::string::npos);
  EXPECT_NE(profiles.find("runtime_host_builder"), std::string::npos);
}

TEST(ContractSurface, LaunchProfilesAndCanonicalPublicAliasExist) {
  const auto support = readText(kProjectRoot / "launch" / "_simulation_support.py");
  const auto profile = readText(kProjectRoot / "launch" / "_launch_profile.py");
  const auto canonical = readText(kProjectRoot / "launch" / "xmate_er3_public.launch.py");
  EXPECT_NE(profile.find("public_xmate_er3_jtc"), std::string::npos);
  EXPECT_NE(profile.find("public_xmate_er3_headless_sdk_smoke"), std::string::npos);
  EXPECT_NE(profile.find("public_xmate_er3_experimental_rt"), std::string::npos);
  EXPECT_NE(profile.find("daemon_hard_rt"), std::string::npos);
  EXPECT_NE(profile.find("unknown launch_profile"), std::string::npos);
  EXPECT_NE(support.find("validate_launch_profile_action"), std::string::npos);
  EXPECT_NE(support.find("build_runtime_host_group"), std::string::npos);
  EXPECT_NE(support.find("launch_profile"), std::string::npos);
  EXPECT_NE(support.find("runtime_host"), std::string::npos);
  EXPECT_NE(canonical.find("simulation.launch.py"), std::string::npos);
}

TEST(ContractSurface, PackagingSeparatesPublicAndInternalInstallComponents) {
  const auto packaging = readText(kProjectRoot / "cmake" / "targets_packaging.cmake");
  const auto root = readText(kProjectRoot / "CMakeLists.txt");
  EXPECT_NE(packaging.find("COMPONENT public_sdk"), std::string::npos);
  EXPECT_NE(packaging.find("COMPONENT internal_runtime"), std::string::npos);
  EXPECT_NE(packaging.find("COMPONENT internal_devel"), std::string::npos);
  EXPECT_NE(packaging.find("docs/public/PUBLIC_SDK_ARTIFACT.md"), std::string::npos);
  EXPECT_NE(packaging.find("xmate_er3_public.launch.py"), std::string::npos);
  EXPECT_NE(root.find("--component public_sdk"), std::string::npos);
}

TEST(ContractSurface, QueryAuthorityAndExtensionContractsAreRuntimeBacked) {
  const auto profile_service = readText(kProjectRoot / "src" / "runtime" / "query_profile_service.cpp");
  const auto state_service = readText(kProjectRoot / "src" / "runtime" / "query_state_service.cpp");
  const auto query_facade = readText(kProjectRoot / "src" / "runtime" / "query_facade.cpp");
  const auto query_diag = readText(kProjectRoot / "src" / "runtime" / "query_diagnostics_service.cpp");
  const auto query_kin = readText(kProjectRoot / "src" / "runtime" / "query_kinematics_service.cpp");
  const auto contract_header = readText(kProjectRoot / "src" / "runtime" / "motion_extension_contract.hpp");
  const auto runtime_context = readText(kProjectRoot / "src" / "runtime" / "runtime_context.cpp");
  const auto coordinator_impl = readText(kProjectRoot / "src" / "runtime" / "request_coordinator.cpp");
  EXPECT_NE(profile_service.find("summarizeMotionExtensionContracts"), std::string::npos);
  EXPECT_NE(profile_service.find("validateMotionExtensionContracts"), std::string::npos);
  EXPECT_NE(runtime_context.find("validateMotionExtensionContracts"), std::string::npos);
  EXPECT_NE(state_service.find("query_authority=runtime_request_coordinator"), std::string::npos);
  EXPECT_NE(query_facade.find("request_coordinator_.readAuthorityJointState"), std::string::npos);
  EXPECT_NE(coordinator_impl.find("motion_runtime_.readAuthoritativeSnapshot(snapshot)"), std::string::npos);
  EXPECT_EQ(query_facade.find("joint_state_fetcher_(pos, vel, tau)"), std::string::npos);
  EXPECT_EQ(query_diag.find("joint_state_fetcher_(pos, vel, tau_array)"), std::string::npos);
  EXPECT_EQ(query_diag.find("joint_state_fetcher_(pos, vel, measured)"), std::string::npos);
  EXPECT_EQ(query_kin.find("joint_state_fetcher_(pos, vel, tau)"), std::string::npos);
  EXPECT_NE(contract_header.find("MotionExtensionContract"), std::string::npos);
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
  EXPECT_NE(render_helper.find("canonical metadata"), std::string::npos);
  EXPECT_NE(render_helper.find("source_xacro_package_relative"), std::string::npos);
  EXPECT_NE(smoke.find("allow-noncanonical-model false"), std::string::npos);
  EXPECT_NE(smoke.find("allow-noncanonical-model true"), std::string::npos);
  EXPECT_NE(smoke.find("canonical install-tree re-render does not match xacro for hybrid/internal_full"), std::string::npos);
  EXPECT_NE(sim_alias.find("compatibility_alias_policy"), std::string::npos);
  EXPECT_NE(gazebo_alias.find("compatibility_alias_policy"), std::string::npos);
  EXPECT_NE(rviz_only.find("compatibility_alias_policy"), std::string::npos);
  EXPECT_NE(render_helper.find("compatibility-alias-policy"), std::string::npos);
  EXPECT_NE(smoke.find("PUBLIC_COMPATIBILITY_ALIAS_POLICY"), std::string::npos);
}


TEST(ContractSurface, CompatHarnessDocumentsInstallTreeAndSymbolChecks) {
  const auto compat_doc = readText(kProjectRoot / "docs" / "reference" / "SDK_ALIGNMENT.md");
  EXPECT_NE(compat_doc.find("check_public_contract_manifest.py"), std::string::npos);
  EXPECT_NE(compat_doc.find("check_compat_public_abi.py"), std::string::npos);
}

TEST(ContractSurface, ExamplesAreSplitBetweenPublicCompatAndInternalBackendGroups) {
  const auto examples_cmake = readText(kProjectRoot / "cmake" / "targets_examples.cmake");
  const auto examples_doc = readText(kProjectRoot / "docs" / "public" / "EXAMPLES.md");
  EXPECT_NE(examples_cmake.find("ROKAE_PUBLIC_COMPAT_EXAMPLES"), std::string::npos);
  EXPECT_NE(examples_cmake.find("ROKAE_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
  EXPECT_NE(examples_doc.find("Public examples"), std::string::npos);
  EXPECT_NE(examples_doc.find("Internal/backend examples"), std::string::npos);
  EXPECT_NE(examples_cmake.find("  20_rt_joint_position"), std::string::npos);
  EXPECT_NE(examples_cmake.find("set(ROKAE_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
}


TEST(ContractSurface, PackagingOnlyInstallsPublicExamplesByDefault) {
  const auto root = readText(kProjectRoot / "CMakeLists.txt");
  const auto packaging = readText(kProjectRoot / "cmake" / "targets_packaging.cmake");
  EXPECT_NE(root.find("ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
  EXPECT_NE(packaging.find("${PUBLIC_COMPAT_EXAMPLE_TARGETS}"), std::string::npos);
  EXPECT_NE(packaging.find("ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES"), std::string::npos);
}

TEST(ContractSurface, RuntimeHostPolicyExportsUnifiedDefaultBackendModeAndRosBridgePrimaryConsumer) {
  const auto config = readText(kProjectRoot / "cmake" / "xCoreSDKConfig.cmake.in");
  const auto policy = readText(kProjectRoot / "config" / "default_runtime_host_policy.env");
  const auto readme = readText(kProjectRoot / "README.md");
  const auto quickstart = readText(kProjectRoot / "docs" / "public" / "QUICKSTART.md");
  const auto artifact = readText(kProjectRoot / "docs" / "public" / "PUBLIC_SDK_ARTIFACT.md");
  const auto install_tree = readText(kProjectRoot / "test" / "compat" / "install_tree" / "CMakeLists.txt");
  EXPECT_NE(config.find("xCoreSDK_PRIMARY_INSTALL_CONSUMER"), std::string::npos);
  EXPECT_NE(config.find("xCoreSDK_DEFAULT_BACKEND_MODE"), std::string::npos);
  EXPECT_NE(config.find("set(xCoreSDK_core_FOUND TRUE)"), std::string::npos);
  EXPECT_NE(config.find("set(xCoreSDK_BACKEND_MODE"), std::string::npos);
  EXPECT_NE(policy.find("ROKAE_DEFAULT_BACKEND_MODE=jtc"), std::string::npos);
  EXPECT_NE(policy.find("ROKAE_DEFAULT_METADATA_BACKEND_MODE=jtc"), std::string::npos);
  EXPECT_NE(readme.find("默认 install-facing C++ core SDK 入口"), std::string::npos);
  EXPECT_NE(quickstart.find("install-facing 主消费者"), std::string::npos);
  EXPECT_NE(artifact.find("smoke-verified by `ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON`"), std::string::npos);
  const auto packaging_contract = readText(kProjectRoot / "test" / "harness" / "check_public_sdk_packaging_contract.py");
  EXPECT_NE(packaging_contract.find("find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)"), std::string::npos);
  EXPECT_NE(install_tree.find("xCoreSDK_PRIMARY_INSTALL_CONSUMER"), std::string::npos);
  EXPECT_NE(install_tree.find("if(xCoreSDK_PRIMARY_INSTALL_CONSUMER STREQUAL \"cxx_sdk_core_consumer\")"), std::string::npos);
  EXPECT_NE(install_tree.find("set(XCORESDK_PRIMARY_INSTALL_LINK_TARGET xCoreSDK::xCoreSDK_core)"), std::string::npos);
  EXPECT_NE(install_tree.find("target_link_libraries(minimal_connect PRIVATE ${XCORESDK_RUNTIME_LINK_TARGET})"), std::string::npos);
  EXPECT_NE(install_tree.find("target_link_libraries(minimal_model PRIVATE ${XCORESDK_CORE_ONLY_LINK_TARGET})"), std::string::npos);
}

TEST(ContractSurface, ReplayPathIsDocumentedAsImmediateSubmitSideLane) {
  const auto compat = readText(kProjectRoot / "docs" / "public" / "COMPATIBILITY.md");
  const auto runtime_state = readText(kProjectRoot / "docs" / "reference" / "RUNTIME_STATE_MACHINE.md");
  const auto alignment = readText(kProjectRoot / "docs" / "reference" / "SDK_ALIGNMENT.md");
  const auto coordinator = readText(kProjectRoot / "src" / "runtime" / "request_coordinator.hpp");
  EXPECT_NE(compat.find("experimental immediate-submit side-lane"), std::string::npos);
  EXPECT_NE(runtime_state.find("immediate-submit side-lane"), std::string::npos);
  EXPECT_NE(alignment.find("replayPath()` = experimental immediate-submit side-lane"), std::string::npos);
  EXPECT_NE(coordinator.find("does not require `moveStart()`"), std::string::npos);
}

TEST(ContractSurface, InstallFacingPackagingSeparatesPrivateBackendExportSet) {
  const auto packaging = readText(kProjectRoot / "cmake" / "targets_packaging.cmake");
  const auto config = readText(kProjectRoot / "cmake" / "xCoreSDKConfig.cmake.in");
  const auto compat = readText(kProjectRoot / "cmake" / "targets_sdk_compat.cmake");
  const auto runtime = readText(kProjectRoot / "cmake" / "targets_runtime.cmake");
  const auto sdk_backend = readText(kProjectRoot / "cmake" / "targets_sdk_backend.cmake");
  EXPECT_NE(packaging.find("EXPORT xCoreSDKTargets"), std::string::npos);
  EXPECT_NE(packaging.find("EXPORT xCoreSDKPrivateTargets"), std::string::npos);
  EXPECT_EQ(config.find("find_dependency(gazebo_ros REQUIRED CONFIG)"), std::string::npos);
  EXPECT_EQ(compat.find("gazebo_ros"), std::string::npos);
  EXPECT_EQ(compat.find("${GAZEBO_LIBRARIES}"), std::string::npos);
  EXPECT_NE(runtime.find("${GAZEBO_LIBRARIES}"), std::string::npos);
  EXPECT_EQ(sdk_backend.find("gazebo_ros"), std::string::npos);
  EXPECT_EQ(sdk_backend.find("${GAZEBO_LIBRARIES}"), std::string::npos);
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
