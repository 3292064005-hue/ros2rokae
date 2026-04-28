#include <algorithm>

#include "rokae/planner.h"
#include "rokae_xmate3_ros2/runtime/strict_jerk_profile.hpp"

namespace rokae {
namespace {
using StrictProfile = rokae_xmate3_ros2::runtime::StrictJerkLimitedScalarProfile;
}

struct CartMotionGenerator::Impl {
  double speed_factor = 1.0;
  double s_goal = 0.0;
  double s_init = 0.0;
  double ds_max = 0.5;
  double dds_max_start = 0.5;
  double dds_max_end = 0.5;
  mutable bool dirty = true;
  mutable StrictProfile profile{};

  void refresh() const {
    if (!dirty) return;
    profile.configure(s_goal - s_init,
                      ds_max * std::max(speed_factor, 1e-6),
                      dds_max_start * std::max(speed_factor, 1e-6),
                      dds_max_end * std::max(speed_factor, 1e-6));
    dirty = false;
  }
};

CartMotionGenerator::CartMotionGenerator(double speed_factor, double s_goal)
    : impl_(std::make_shared<Impl>()) {
  impl_->speed_factor = speed_factor;
  impl_->s_goal = s_goal;
}
CartMotionGenerator::~CartMotionGenerator() = default;
void CartMotionGenerator::setMax(double ds_max, double dds_max_start, double dds_max_end) { impl_->ds_max = ds_max; impl_->dds_max_start = dds_max_start; impl_->dds_max_end = dds_max_end; impl_->dirty = true; }
double CartMotionGenerator::getTime() const { impl_->refresh(); return impl_->profile.total_time(); }
bool CartMotionGenerator::calculateDesiredValues(double t, double *delta_s_d) const { impl_->refresh(); if (delta_s_d) *delta_s_d = impl_->profile.position(t); return t >= impl_->profile.total_time(); }
void CartMotionGenerator::calculateSynchronizedValues(double s_init) { impl_->s_init = s_init; impl_->dirty = true; }

struct JointMotionGenerator::Impl {
  double speed_factor = 1.0;
  std::array<double, 6> q_goal{};
  std::array<double, 6> q_init{};
  std::array<double, 6> dq_max{{1,1,1,1,1,1}};
  std::array<double, 6> ddq_max_start{{1,1,1,1,1,1}};
  std::array<double, 6> ddq_max_end{{1,1,1,1,1,1}};
  mutable bool dirty = true;
  mutable std::array<StrictProfile, 6> profiles{};
  mutable double total_time = 0.0;
  void refresh() const {
    if (!dirty) return;
    total_time = 0.0;
    for (std::size_t i = 0; i < 6; ++i) {
      profiles[i].configure(q_goal[i] - q_init[i],
                            dq_max[i] * std::max(speed_factor, 1e-6),
                            ddq_max_start[i] * std::max(speed_factor, 1e-6),
                            ddq_max_end[i] * std::max(speed_factor, 1e-6));
      total_time = std::max(total_time, profiles[i].total_time());
    }
    dirty = false;
  }
};

JointMotionGenerator::JointMotionGenerator(double speed_factor, std::array<double, 6> q_goal)
    : impl_(std::make_shared<Impl>()) { impl_->speed_factor = speed_factor; impl_->q_goal = q_goal; }
JointMotionGenerator::~JointMotionGenerator() = default;
void JointMotionGenerator::setMax(const std::array<double, 6> &dq_max,const std::array<double, 6> &ddq_max_start,const std::array<double, 6> &ddq_max_end) { impl_->dq_max = dq_max; impl_->ddq_max_start = ddq_max_start; impl_->ddq_max_end = ddq_max_end; impl_->dirty = true; }
double JointMotionGenerator::getTime() const { impl_->refresh(); return impl_->total_time; }
bool JointMotionGenerator::calculateDesiredValues(double t, std::array<double, 6> &delta_q_d) const { impl_->refresh(); for (std::size_t i=0;i<6;++i){ if (impl_->total_time <= 1e-12) delta_q_d[i]=impl_->q_goal[i]-impl_->q_init[i]; else delta_q_d[i]=impl_->profiles[i].position(std::clamp(t/impl_->total_time,0.0,1.0)*impl_->profiles[i].total_time()); } return t >= impl_->total_time; }
void JointMotionGenerator::calculateSynchronizedValues(const std::array<double, 6> &q_init) { impl_->q_init = q_init; impl_->dirty = true; }

}  // namespace rokae
