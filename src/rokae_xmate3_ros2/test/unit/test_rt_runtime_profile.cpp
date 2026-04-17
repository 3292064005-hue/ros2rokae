#include <gtest/gtest.h>

#include "runtime/rt_runtime_profile.hpp"

namespace rokae_xmate3_ros2::runtime {
namespace {

TEST(RuntimeRtProfileTest, Hard1kHzRequiresDaemonizedRuntimeHost) {
  const auto hard_for_daemon = resolveRuntimeRtProfile("hard_1khz", RuntimeHostKind::daemonized_runtime);
  EXPECT_TRUE(hard_for_daemon.supported);
  EXPECT_TRUE(hard_for_daemon.authoritative_servo_clock);
  EXPECT_TRUE(hard_for_daemon.require_shm_transport);
  EXPECT_FALSE(hard_for_daemon.allow_topic_rt_ingress);
  EXPECT_FALSE(hard_for_daemon.allow_legacy_rt_custom_data);
  EXPECT_TRUE(hard_for_daemon.fail_on_degraded_scheduler);
  EXPECT_EQ(hard_for_daemon.executor_threads, 1);

  const auto hard_for_plugin = resolveRuntimeRtProfile("hard_1khz", RuntimeHostKind::gazebo_plugin);
  EXPECT_FALSE(hard_for_plugin.supported);
  EXPECT_NE(hard_for_plugin.startup_error.find("daemonized runtime host"), std::string::npos);
}

TEST(RuntimeRtProfileTest, RtHardenedKeepsAuthoritativeServoWithoutShmOnlyContract) {
  const auto profile = resolveRuntimeRtProfile("rt_hardened", RuntimeHostKind::gazebo_plugin);
  EXPECT_TRUE(profile.supported);
  EXPECT_TRUE(profile.authoritative_servo_clock);
  EXPECT_FALSE(profile.require_shm_transport);
  EXPECT_TRUE(profile.allow_topic_rt_ingress);
  EXPECT_FALSE(profile.allow_legacy_rt_custom_data);
  EXPECT_FALSE(profile.fail_on_degraded_scheduler);
  EXPECT_DOUBLE_EQ(profile.publish_rates.joint_state_period_sec, 0.004);
  EXPECT_DOUBLE_EQ(profile.publish_rates.operation_state_period_sec, 0.020);
  EXPECT_DOUBLE_EQ(profile.publish_rates.diagnostics_period_sec, 0.020);

  const auto summary = summarizeRuntimeRtProfile(profile);
  EXPECT_NE(summary.find("authoritative_servo=true"), std::string::npos);
  EXPECT_NE(summary.find("allow_legacy_rt_custom_data=false"), std::string::npos);
}

}  // namespace
}  // namespace rokae_xmate3_ros2::runtime
