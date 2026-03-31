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

TEST(ImplementationAudit, AuditDocExistsAndIsHonestAboutCoverage) {
  const auto audit = readText(kProjectRoot / "docs" / "IMPLEMENTATION_AUDIT.md");
  EXPECT_NE(audit.find("not yet a controller-grade parity implementation"), std::string::npos);
  EXPECT_NE(audit.find("4.5 Realtime control"), std::string::npos);
  EXPECT_NE(audit.find("Not fully closed"), std::string::npos);
  EXPECT_NE(audit.find("Simulation-grade"), std::string::npos);
  EXPECT_NE(audit.find("ReadRegisterEx"), std::string::npos);
  EXPECT_NE(audit.find("GetEndWrench"), std::string::npos);
}

TEST(ImplementationAudit, HardeningBacklogCapturesRemainingHighRiskWork) {
  const auto backlog = readText(kProjectRoot / "docs" / "HARDENING_BACKLOG.md");
  EXPECT_NE(backlog.find("single-primary-backend requests"), std::string::npos);
  EXPECT_NE(backlog.find("Planner preflight reporting"), std::string::npos);
  EXPECT_NE(backlog.find("RT subsystem hardening"), std::string::npos);
  EXPECT_NE(backlog.find("Runtime catalog normalization"), std::string::npos);
  EXPECT_NE(backlog.find("preferred vs legacy contract split"), std::string::npos);
  EXPECT_NE(backlog.find("rt_field_registry"), std::string::npos);
}

TEST(ImplementationAudit, ReadmeLinksToAuditAndBacklog) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("docs/IMPLEMENTATION_AUDIT.md"), std::string::npos);
  EXPECT_NE(readme.find("docs/HARDENING_BACKLOG.md"), std::string::npos);
  EXPECT_NE(readme.find("## 实现审计"), std::string::npos);
}

TEST(ImplementationAudit, RuntimeStateMachineAndPlannerPreflightExist) {
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "runtime_state_machine.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "runtime_state_machine.hpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "planner_preflight.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "planner_trace.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "rt_field_registry.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "rt_subscription_plan.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "rt_watchdog.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "rt_prearm_checks.cpp"));
}

TEST(ImplementationAudit, RuntimeCatalogPolicyDocExists) {
  const auto policy = readText(kProjectRoot / "docs" / "RUNTIME_CATALOG_POLICY.md");
  EXPECT_NE(policy.find("runtime is the only source of truth"), std::string::npos);
  EXPECT_NE(policy.find("GetToolCatalog"), std::string::npos);
}

TEST(ImplementationAudit, RtHardeningProfileDocExists) {
  const auto profile = readText(kProjectRoot / "docs" / "RT_HARDENING_PROFILE.md");
  EXPECT_NE(profile.find("RT field registry"), std::string::npos);
  EXPECT_NE(profile.find("rt_watchdog"), std::string::npos);
}

TEST(ImplementationAudit, QueryAndSdkStateDomainsStaySplitOnDisk) {
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "query_state_service.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "query_kinematics_service.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "query_diagnostics_service.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "query_catalog_service.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "query_profile_service.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "runtime" / "runtime_catalog_service.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "sdk" / "robot_state_cache.cpp"));
  EXPECT_TRUE(std::filesystem::exists(kProjectRoot / "src" / "sdk" / "robot_state_queries.cpp"));
}

}  // namespace
