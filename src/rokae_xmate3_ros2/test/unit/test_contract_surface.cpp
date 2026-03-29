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
}

TEST(ContractSurface, ReadmeStopsClaimingGenericIoParity) {
  const auto readme = readText(kProjectRoot / "README.md");
  EXPECT_NE(readme.find("| **4.6** | IO 与通信接口 | 近似实现 |"), std::string::npos);
  EXPECT_NE(readme.find("GetEndWrench"), std::string::npos);
}

}  // namespace
