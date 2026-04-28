#include <algorithm>
#include <thread>

#include "rokae/planner.h"
#include "rokae/robot.h"

namespace rokae {
namespace {

constexpr double kFollowPositionMaxStepRadPerCycle = 0.005;

template <typename ExceptionT>
void throw_if_follow_error(const error_code &ec, const char *context) {
  if (ec) {
    throw ExceptionT(std::string(context) + ": " + ec.message(), ec);
  }
}

}  // namespace

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
