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
  EXPECT_NE(backlog.find("preferred vs legacy contract split"), std::string::npos);
}

TEST(ImplementationAudit, ReadmeLinksToAuditAndBacklog) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("docs/IMPLEMENTATION_AUDIT.md"), std::string::npos);
  EXPECT_NE(readme.find("docs/HARDENING_BACKLOG.md"), std::string::npos);
  EXPECT_NE(readme.find("## 实现审计"), std::string::npos);
}

}  // namespace
