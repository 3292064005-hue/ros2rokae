#include "rokae/planner.h"

#include <algorithm>
#include <thread>

#include "rokae/robot.h"
#include "rokae_xmate3_ros2/runtime/strict_jerk_profile.hpp"

namespace rokae {
namespace {
using StrictProfile = rokae_xmate3_ros2::runtime::StrictJerkLimitedScalarProfile;

constexpr double kFollowPositionMaxStepRadPerCycle = 0.005;

template <typename ExceptionT>
void throw_if_follow_error(const error_code &ec, const char *context) {
  if (ec) {
    throw ExceptionT(std::string(context) + ": " + ec.message(), ec);
  }
}
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

struct FollowPosition<6>::Impl {
  Cobot<6> *robot = nullptr;
  xMateModel<6> *model = nullptr;
  Eigen::Transform<double, 3, Eigen::Isometry> end_in_flange = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
  std::mutex mutex;
  std::array<double, 6> desired_joints{};
  std::array<double, 6> commanded_joints{};
  double scale = 0.5;
  bool active = false;
  std::shared_ptr<RtMotionControlCobot<6>> rt;
};

FollowPosition<6>::FollowPosition()
    : impl_(std::make_shared<Impl>()) {}

FollowPosition<6>::FollowPosition(Cobot<6> &robot,
                                  xMateModel<6> &model,
                                  const Eigen::Transform<double, 3, Eigen::Isometry> &endInFlange)
    : impl_(std::make_shared<Impl>()) {
  impl_->robot = &robot;
  impl_->model = &model;
  impl_->end_in_flange = endInFlange;
}

FollowPosition<6>::~FollowPosition() = default;

void FollowPosition<6>::init(Cobot<6> &robot, xMateModel<6> &model) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->robot = &robot;
  impl_->model = &model;
}

void FollowPosition<6>::start(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
  if (!impl_->robot || !impl_->model) {
    throw RealtimeControlException("FollowPosition is not initialized");
  }
  error_code ec;
  auto rt = impl_->robot->getRtMotionController().lock();
  if (!rt) {
    throw RealtimeControlException("FollowPosition failed to acquire RT controller");
  }
  impl_->robot->setOperateMode(OperateMode::automatic, ec);
  throw_if_follow_error<RealtimeControlException>(ec, "FollowPosition::start setOperateMode");
  impl_->robot->setPowerState(true, ec);
  throw_if_follow_error<RealtimeControlException>(ec, "FollowPosition::start setPowerState");
  impl_->robot->setMotionControlMode(MotionControlMode::RtCommand, ec);
  throw_if_follow_error<RealtimeControlException>(ec, "FollowPosition::start setMotionControlMode");
  const auto current_joints = impl_->robot->jointPos(ec);
  throw_if_follow_error<RealtimeStateException>(ec, "FollowPosition::start jointPos");
  rt->startMove(RtControllerMode::jointPosition);
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->desired_joints = current_joints;
    impl_->commanded_joints = current_joints;
    impl_->rt = rt;
  }
  rt->setControlLoop([state = impl_]() {
    std::lock_guard<std::mutex> lock(state->mutex);
    const double scale = std::clamp(state->scale, 0.01, 1.0);
    const double max_step = kFollowPositionMaxStepRadPerCycle * scale;
    for (std::size_t axis = 0; axis < state->commanded_joints.size(); ++axis) {
      const double delta = state->desired_joints[axis] - state->commanded_joints[axis];
      state->commanded_joints[axis] += std::clamp(delta, -max_step, max_step);
    }
    JointPosition command;
    command.joints.assign(state->commanded_joints.begin(), state->commanded_joints.end());
    return command;
  });
  rt->startLoop(false);
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->active = true;
  }
  update(bMe_desire);
}

void FollowPosition<6>::start(const std::array<double, 6> &jnt_desire) {
  if (!impl_->robot || !impl_->model) {
    throw RealtimeControlException("FollowPosition is not initialized");
  }
  if (!impl_->active) {
    Eigen::Transform<double, 3, Eigen::Isometry> identity = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();
    start(identity);
  }
  update(jnt_desire);
}

void FollowPosition<6>::stop() {
  if (impl_->rt) {
    try { impl_->rt->stopLoop(); } catch (...) {}
    try { impl_->rt->stopMove(); } catch (...) {}
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->active = false;
  impl_->rt.reset();
}

void FollowPosition<6>::update(const Eigen::Transform<double, 3, Eigen::Isometry> &bMe_desire) {
  if (!impl_->active || !impl_->model) {
    return;
  }
  const Eigen::Transform<double, 3, Eigen::Isometry> flange_target = bMe_desire * impl_->end_in_flange.inverse();
  std::array<double, 16> cart{};
  const auto matrix = flange_target.matrix();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      cart[static_cast<std::size_t>(row * 4 + col)] = matrix(row, col);
    }
  }
  std::array<double, 6> solved{};
  std::array<double, 6> seed{};
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    seed = impl_->commanded_joints;
  }
  if (impl_->model->getJointPos(cart, 0.0, seed, solved) == 0) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->desired_joints = solved;
  }
}

void FollowPosition<6>::update(const std::array<double, 6> &jnt_desired) {
  if (!impl_->active) {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->desired_joints = jnt_desired;
}

void FollowPosition<6>::setScale(double scale) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->scale = std::clamp(scale, 0.01, 1.0);
}

}  // namespace rokae
