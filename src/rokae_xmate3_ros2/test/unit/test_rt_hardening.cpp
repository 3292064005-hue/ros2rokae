#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>

#include "runtime/rt_field_registry.hpp"
#include "runtime/rt_prearm_checks.hpp"
#include "runtime/session_state.hpp"
#include "runtime/rt_subscription_plan.hpp"
#include "runtime/rt_watchdog.hpp"
#include "rokae_xmate3_ros2/types.hpp"

namespace rokae_xmate3_ros2::runtime {

TEST(RtHardening, FieldRegistryRecognizesDefaultFields) {
  EXPECT_TRUE(isRtFieldSupported(rokae::RtSupportedFields::jointPos_m));
  EXPECT_TRUE(isRtFieldSupported(rokae::RtSupportedFields::jointVel_m));
  EXPECT_TRUE(isRtFieldSupported(rokae::RtSupportedFields::tau_m));
  EXPECT_TRUE(isRtFieldSupported(rokae::RtCompatFields::samplePeriod_s));
  EXPECT_TRUE(isRtFieldSupported(rokae::RtCompatFields::sampleFresh));
  EXPECT_FALSE(isRtFieldSupported("unknown_field"));
  EXPECT_GT(rtFieldBytes(rokae::RtSupportedFields::tcpPose_m), rtFieldBytes(rokae::RtSupportedFields::jointPos_m));
  EXPECT_EQ(rtFieldBytes(rokae::RtCompatFields::samplePeriod_s), sizeof(double));
}

TEST(RtHardening, SubscriptionPlanRejectsUnsupportedFieldsAndWrongInLoopRate) {
  const auto bad_fields = buildRtSubscriptionPlan({"unknown_field"}, std::chrono::milliseconds(1), true);
  EXPECT_FALSE(bad_fields.ok);
  EXPECT_EQ(bad_fields.status, "unsupported_fields");

  const auto wrong_rate = buildRtSubscriptionPlan(defaultRtFieldSet(), std::chrono::milliseconds(2), true);
  EXPECT_FALSE(wrong_rate.ok);
  EXPECT_EQ(wrong_rate.status, "in_loop_requires_1ms");

  const auto unsupported_interval = buildRtSubscriptionPlan(defaultRtFieldSet(), std::chrono::milliseconds(3), false);
  EXPECT_FALSE(unsupported_interval.ok);
  EXPECT_EQ(unsupported_interval.status, "unsupported_interval");

  const auto unsupported_50ms = buildRtSubscriptionPlan(defaultRtFieldSet(), std::chrono::milliseconds(50), false);
  EXPECT_FALSE(unsupported_50ms.ok);
  EXPECT_EQ(unsupported_50ms.status, "unsupported_interval");

  const auto polled_1s = buildRtSubscriptionPlan(defaultRtFieldSet(), std::chrono::seconds(1), false);
  EXPECT_TRUE(polled_1s.ok);
  EXPECT_EQ(polled_1s.status, "armed_polled");

  const auto ok = buildRtSubscriptionPlan(defaultRtFieldSet(), std::chrono::milliseconds(1), true);
  EXPECT_TRUE(ok.ok);
  EXPECT_EQ(ok.status, "armed_in_loop");
  EXPECT_FALSE(ok.accepted_fields.empty());
}

TEST(RtHardening, SubscriptionPlanRejectsServiceBackedFieldsForStrictInLoopRt) {
  const auto strict_pose = buildRtSubscriptionPlan({rokae::RtSupportedFields::jointPos_m,
                                                    rokae::RtSupportedFields::tcpPose_m},
                                                   std::chrono::milliseconds(1),
                                                   true);
  EXPECT_FALSE(strict_pose.ok);
  EXPECT_EQ(strict_pose.status, "unsupported_fields");
  EXPECT_NE(std::find(strict_pose.rejected_fields.begin(), strict_pose.rejected_fields.end(),
                      rokae::RtSupportedFields::tcpPose_m),
            strict_pose.rejected_fields.end());

  const auto polled_pose = buildRtSubscriptionPlan({rokae::RtSupportedFields::jointPos_m,
                                                    rokae::RtSupportedFields::tcpPose_m},
                                                   std::chrono::milliseconds(8),
                                                   false);
  EXPECT_TRUE(polled_pose.ok);
  EXPECT_EQ(polled_pose.status, "armed_polled");
}

TEST(RtHardening, CompatRtMetadataFieldsRemainStrictInLoopSafe) {
  EXPECT_TRUE(isStrictRtInLoopFieldSupported(rokae::RtCompatFields::samplePeriod_s));
  EXPECT_TRUE(isStrictRtInLoopFieldSupported(rokae::RtCompatFields::sampleFresh));

  const auto plan = buildRtSubscriptionPlan({rokae::RtSupportedFields::jointPos_m,
                                             rokae::RtCompatFields::samplePeriod_s,
                                             rokae::RtCompatFields::sampleFresh},
                                            std::chrono::milliseconds(1),
                                            true);
  EXPECT_TRUE(plan.ok);
  EXPECT_EQ(plan.status, "armed_in_loop");
  EXPECT_NE(std::find(plan.accepted_fields.begin(), plan.accepted_fields.end(), rokae::RtCompatFields::samplePeriod_s),
            plan.accepted_fields.end());
  EXPECT_NE(std::find(plan.accepted_fields.begin(), plan.accepted_fields.end(), rokae::RtCompatFields::sampleFresh),
            plan.accepted_fields.end());
}

TEST(RtHardening, StrictInLoopFieldClassificationMatchesRegistry) {
  EXPECT_TRUE(isStrictRtInLoopFieldSupported(rokae::RtSupportedFields::jointPos_m));
  EXPECT_TRUE(isStrictRtInLoopFieldSupported(rokae::RtSupportedFields::jointAcc_c));
  EXPECT_TRUE(isStrictRtInLoopFieldSupported(rokae::RtSupportedFields::tauVel_c));
  EXPECT_FALSE(isStrictRtInLoopFieldSupported(rokae::RtSupportedFields::tcpPose_m));
  EXPECT_FALSE(isStrictRtInLoopFieldSupported(rokae::RtSupportedFields::tauExt_inBase));
  EXPECT_FALSE(isStrictRtInLoopFieldSupported(rokae::RtSupportedFields::elbow_m));
}

TEST(RtHardening, PrearmRequiresRtModePowerCapabilityAndValidPlan) {
  RtPrearmCheckInput input;
  input.motion_mode = kSessionMotionModeRt;
  input.rt_mode = static_cast<int>(rokae::RtControllerMode::jointPosition);
  input.power_on = true;
  input.active_profile = "rt_simulated";
  input.network_tolerance_configured = true;
  input.capability_flags = {"rt.experimental", "trajectory_executor"};
  input.subscription_plan = buildRtSubscriptionPlan(defaultRtFieldSet(), std::chrono::milliseconds(1), true);

  const auto ready = evaluateRtPrearm(input);
  EXPECT_TRUE(ready.ok);
  EXPECT_EQ(ready.status, "ready");

  input.power_on = false;
  const auto power_off = evaluateRtPrearm(input);
  EXPECT_FALSE(power_off.ok);
  EXPECT_EQ(power_off.status, "power_off");

  input.power_on = true;
  input.capability_flags.clear();
  const auto no_capability = evaluateRtPrearm(input);
  EXPECT_FALSE(no_capability.ok);
  EXPECT_EQ(no_capability.status, "profile_not_rt");
}

TEST(RtHardening, PrearmRejectsIncompleteModeSpecificSubscription) {
  RtPrearmCheckInput input;
  input.motion_mode = kSessionMotionModeRt;
  input.rt_mode = static_cast<int>(rokae::RtControllerMode::torque);
  input.power_on = true;
  input.active_profile = "rt_simulated";
  input.network_tolerance_configured = true;
  input.capability_flags = {"rt.experimental", "effort_owner"};
  input.subscription_plan = buildRtSubscriptionPlan(
      {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m},
      std::chrono::milliseconds(1),
      true);

  const auto report = evaluateRtPrearm(input);
  EXPECT_FALSE(report.ok);
  EXPECT_EQ(report.status, "subscription_plan_incomplete");
}

TEST(RtHardening, WatchdogTracksLateCyclesGapAndTriggerReason) {
  RtWatchdog watchdog(0.004, 0.050);
  watchdog.observeCycle(0.001, true, true);
  watchdog.observeCycle(0.006, true, true);
  watchdog.observeCycle(0.003, false, false);
  const auto snapshot = watchdog.snapshot();
  EXPECT_EQ(snapshot.late_cycle_count, 1u);
  EXPECT_GT(snapshot.max_gap_ms, 5.0);
  EXPECT_GT(snapshot.avg_gap_ms, 0.0);
  EXPECT_TRUE(snapshot.stale_state);
  EXPECT_TRUE(snapshot.command_starvation);
  EXPECT_EQ(snapshot.stale_state_count, 1u);
  EXPECT_EQ(snapshot.command_starvation_windows, 1u);
  EXPECT_EQ(snapshot.last_trigger_reason, "command_starvation");
  EXPECT_NE(snapshot.summary.find("late_cycles=1"), std::string::npos);
  EXPECT_NE(snapshot.summary.find("avg_gap_ms="), std::string::npos);
}

}  // namespace rokae_xmate3_ros2::runtime
