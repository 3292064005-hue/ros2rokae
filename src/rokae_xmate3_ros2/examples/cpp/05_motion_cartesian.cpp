/**
 * @file 05_motion_cartesian.cpp
 * @brief 官方 SDK 风格 - MoveJ / MoveL / MoveC
 */

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rokae/robot.h"
#include "rokae_xmate3_ros2/srv/validate_motion.hpp"
#include "example_common.hpp"

using namespace rokae;
using namespace example;

namespace {

using ValidateMotion = rokae_xmate3_ros2::srv::ValidateMotion;

struct PoseDelta {
  double dx{0.0};
  double dy{0.0};
  double dz{0.0};
  double drx{0.0};
  double dry{0.0};
  double drz{0.0};
};

struct CircleDelta {
  PoseDelta aux;
  PoseDelta target;
};

struct ValidatedPoseChoice {
  CartesianPosition target;
  PoseDelta delta;
  ValidateMotion::Response response;
};

struct ValidatedCircleChoice {
  CartesianPosition aux;
  CartesianPosition target;
  CircleDelta delta;
  ValidateMotion::Response response;
};

class MotionValidator {
public:
  MotionValidator()
      : node_(rclcpp::Node::make_shared("xmate3_example_05_validator")),
        client_(node_->create_client<ValidateMotion>("/xmate3/internal/validate_motion")) {}

  bool waitForService(std::chrono::seconds timeout, error_code &ec) {
    if (client_->wait_for_service(timeout)) {
      ec.clear();
      return true;
    }
    ec = std::make_error_code(std::errc::host_unreachable);
    return false;
  }

  bool validatePose(uint8_t motion_kind,
                    const std::array<double, 6> &start_joints,
                    const CartesianPosition &target,
                    int speed,
                    int zone,
                    ValidateMotion::Response &response,
                    error_code &ec) {
    ValidateMotion::Request request;
    request.motion_kind = motion_kind;
    request.use_start_joint_pos = true;
    request.start_joint_pos = start_joints;
    request.target_posture = toArray(target);
    request.speed = speed;
    request.zone = zone;
    request.strict_conf = false;
    request.avoid_singularity = false;
    request.soft_limit_enabled = false;
    request.speed_scale = 1.0;
    return call(request, response, ec);
  }

  bool validateCircle(const std::array<double, 6> &start_joints,
                      const CartesianPosition &aux,
                      const CartesianPosition &target,
                      int speed,
                      int zone,
                      ValidateMotion::Response &response,
                      error_code &ec) {
    ValidateMotion::Request request;
    request.motion_kind = ValidateMotion::Request::MOTION_MOVE_C;
    request.use_start_joint_pos = true;
    request.start_joint_pos = start_joints;
    request.aux_posture = toArray(aux);
    request.target_posture = toArray(target);
    request.speed = speed;
    request.zone = zone;
    request.strict_conf = false;
    request.avoid_singularity = false;
    request.soft_limit_enabled = false;
    request.speed_scale = 1.0;
    return call(request, response, ec);
  }

private:
  static std::array<double, 6> toArray(const CartesianPosition &pose) {
    return {pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz};
  }

  bool call(const ValidateMotion::Request &request,
            ValidateMotion::Response &response,
            error_code &ec) {
    auto request_ptr = std::make_shared<ValidateMotion::Request>(request);
    auto future = client_->async_send_request(request_ptr);
    const auto status = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10));
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      ec = std::make_error_code(status == rclcpp::FutureReturnCode::TIMEOUT ? std::errc::timed_out
                                                                            : std::errc::io_error);
      return false;
    }
    response = *future.get();
    ec.clear();
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ValidateMotion>::SharedPtr client_;
};

void printCartesianPosition(xMateRobot &robot, error_code &ec) {
  const auto pose = robot.cartPosture(CoordinateType::flangeInBase, ec);
  if (!ec) {
    printPose("flange in base", pose);
  }
}

std::string summarizeValidation(const ValidateMotion::Response &response) {
  std::string summary;
  if (!response.selected_branch.empty()) {
    summary += "branch=" + response.selected_branch;
  }
  if (!response.notes.empty()) {
    if (!summary.empty()) {
      summary += " ";
    }
    summary += "note=" + response.notes.front();
  }
  if (summary.empty()) {
    summary = response.message;
  }
  return summary;
}

CartesianPosition offsetPose(const CartesianPosition &base, const PoseDelta &delta) {
  auto pose = base;
  pose.x += delta.dx;
  pose.y += delta.dy;
  pose.z += delta.dz;
  pose.rx += delta.drx;
  pose.ry += delta.dry;
  pose.rz += delta.drz;
  pose.confData.clear();
  return pose;
}

std::vector<PoseDelta> moveJDeltas() {
  return {
      {0.00, 0.00, -0.08, 0.00, 0.00, 0.00},
      {0.04, 0.00, -0.06, 0.00, 0.00, 0.00},
      {-0.04, 0.00, -0.06, 0.00, 0.00, 0.00},
      {0.00, 0.04, -0.06, 0.00, 0.00, 0.00},
      {0.00, -0.04, -0.06, 0.00, 0.00, 0.00},
      {0.04, 0.04, -0.05, 0.00, 0.00, 0.00},
      {0.04, -0.04, -0.05, 0.00, 0.00, 0.00},
      {-0.04, 0.04, -0.05, 0.00, 0.00, 0.00},
      {-0.04, -0.04, -0.05, 0.00, 0.00, 0.00},
      {0.06, 0.00, -0.02, 0.00, 0.12, 0.00},
      {-0.06, 0.00, -0.02, 0.00, -0.12, 0.00},
      {0.00, 0.06, -0.02, 0.00, 0.00, 0.18},
      {0.00, -0.06, -0.02, 0.00, 0.00, -0.18},
      {0.03, 0.03, -0.03, 0.00, 0.12, 0.12},
      {0.03, -0.03, -0.03, 0.00, -0.12, -0.12},
      {-0.03, 0.03, -0.03, 0.00, 0.12, -0.12},
      {-0.03, -0.03, -0.03, 0.00, -0.12, 0.12},
  };
}

std::vector<PoseDelta> moveLDeltas() {
  return {
      {0.03, 0.00, 0.00, 0.00, 0.00, 0.00},
      {0.00, 0.03, 0.00, 0.00, 0.00, 0.00},
      {0.00, -0.03, 0.00, 0.00, 0.00, 0.00},
      {0.02, 0.02, -0.01, 0.00, 0.00, 0.00},
      {0.02, -0.02, -0.01, 0.00, 0.00, 0.00},
      {-0.02, 0.02, 0.01, 0.00, 0.00, 0.00},
      {-0.02, -0.02, 0.01, 0.00, 0.00, 0.00},
      {0.04, 0.00, -0.01, 0.00, 0.00, 0.00},
      {0.00, 0.04, -0.01, 0.00, 0.00, 0.00},
      {0.00, -0.04, -0.01, 0.00, 0.00, 0.00},
  };
}

std::vector<PoseDelta> guidedMoveLDeltas(int step_index) {
  switch (step_index) {
    case 0:
      return {
          {0.02, -0.02, 0.00, 0.00, 0.00, 0.00},
          {0.03, -0.01, -0.01, 0.00, 0.00, 0.00},
          {0.01, -0.03, 0.00, 0.00, 0.00, 0.00},
      };
    case 1:
      return {
          {0.00, -0.04, -0.01, 0.00, 0.00, 0.00},
          {0.02, -0.03, -0.01, 0.00, 0.00, 0.00},
          {-0.02, -0.03, -0.01, 0.00, 0.00, 0.00},
      };
    default:
      return {
          {-0.02, 0.00, 0.01, 0.00, 0.00, 0.00},
          {-0.03, 0.01, 0.01, 0.00, 0.00, 0.00},
          {-0.01, 0.02, 0.01, 0.00, 0.00, 0.00},
      };
  }
}

std::vector<CircleDelta> moveCDeltas() {
  return {
      CircleDelta{PoseDelta{0.02, 0.01, 0.00, 0.00, 0.00, 0.00},
                  PoseDelta{0.04, 0.03, 0.00, 0.00, 0.00, 0.00}},
      CircleDelta{PoseDelta{0.02, -0.01, 0.00, 0.00, 0.00, 0.00},
                  PoseDelta{0.04, -0.03, 0.00, 0.00, 0.00, 0.00}},
      CircleDelta{PoseDelta{0.01, 0.02, -0.01, 0.00, 0.00, 0.00},
                  PoseDelta{0.00, 0.04, -0.01, 0.00, 0.00, 0.00}},
      CircleDelta{PoseDelta{0.01, -0.02, -0.01, 0.00, 0.00, 0.00},
                  PoseDelta{0.00, -0.04, -0.01, 0.00, 0.00, 0.00}},
      CircleDelta{PoseDelta{0.02, 0.00, 0.01, 0.00, 0.00, 0.00},
                  PoseDelta{0.03, 0.02, 0.01, 0.00, 0.00, 0.00}},
  };
}

bool executeMoveAbsJAndWait(xMateRobot &robot,
                            const std::array<double, 6> &target,
                            int speed,
                            int zone,
                            error_code &ec) {
  const std::vector<double> target_joints(target.begin(), target.end());
  robot.executeCommand(std::vector<MoveAbsJCommand>{MoveAbsJCommand(target_joints, speed, zone)}, ec);
  if (ec) {
    return false;
  }
  return waitForCommandResult(robot, "MoveAbsJ", ec);
}

bool executeMoveJAndWait(xMateRobot &robot,
                         const CartesianPosition &target,
                         int speed,
                         int zone,
                         error_code &ec) {
  robot.executeCommand(std::vector<MoveJCommand>{MoveJCommand(target, speed, zone)}, ec);
  if (ec) {
    return false;
  }
  return waitForCommandResult(robot, "MoveJ", ec);
}

bool executeMoveLAndWait(xMateRobot &robot,
                         const CartesianPosition &target,
                         int speed,
                         int zone,
                         error_code &ec) {
  robot.executeCommand(std::vector<MoveLCommand>{MoveLCommand(target, speed, zone)}, ec);
  if (ec) {
    return false;
  }
  return waitForCommandResult(robot, "MoveL", ec);
}

bool executeMoveCAndWait(xMateRobot &robot,
                         const CartesianPosition &aux,
                         const CartesianPosition &target,
                         int speed,
                         int zone,
                         error_code &ec) {
  robot.executeCommand(std::vector<MoveCCommand>{MoveCCommand(target, aux, speed, zone)}, ec);
  if (ec) {
    return false;
  }
  return waitForCommandResult(robot, "MoveC", ec);
}

bool findValidatedPose(xMateRobot &robot,
                       MotionValidator &validator,
                       uint8_t motion_kind,
                       const std::vector<PoseDelta> &deltas,
                       int speed,
                       int zone,
                       ValidatedPoseChoice &choice,
                       error_code &ec) {
  const auto start_joints = robot.jointPos(ec);
  if (ec) {
    return false;
  }
  const auto seed_pose = robot.cartPosture(CoordinateType::endInRef, ec);
  if (ec) {
    return false;
  }

  ValidateMotion::Response last_response;
  for (const auto &delta : deltas) {
    const auto target = offsetPose(seed_pose, delta);
    ValidateMotion::Response response;
    if (!validator.validatePose(motion_kind, start_joints, target, speed, zone, response, ec)) {
      return false;
    }
    last_response = response;
    if (!response.reachable) {
      continue;
    }
    choice.target = target;
    choice.delta = delta;
    choice.response = response;
    ec.clear();
    return true;
  }

  ec = std::make_error_code(std::errc::operation_not_permitted);
  if (!last_response.reject_reason.empty()) {
    os << "validated search rejected: " << last_response.reject_reason
       << " (" << last_response.message << ")" << std::endl;
  }
  return false;
}

bool findValidatedCircle(xMateRobot &robot,
                         MotionValidator &validator,
                         const std::vector<CircleDelta> &deltas,
                         int speed,
                         int zone,
                         ValidatedCircleChoice &choice,
                         error_code &ec) {
  const auto start_joints = robot.jointPos(ec);
  if (ec) {
    return false;
  }
  const auto seed_pose = robot.cartPosture(CoordinateType::endInRef, ec);
  if (ec) {
    return false;
  }

  ValidateMotion::Response last_response;
  for (const auto &delta : deltas) {
    const auto aux = offsetPose(seed_pose, delta.aux);
    const auto target = offsetPose(seed_pose, delta.target);
    ValidateMotion::Response response;
    if (!validator.validateCircle(start_joints, aux, target, speed, zone, response, ec)) {
      return false;
    }
    last_response = response;
    if (!response.reachable) {
      continue;
    }
    choice.aux = aux;
    choice.target = target;
    choice.delta = delta;
    choice.response = response;
    ec.clear();
    return true;
  }

  ec = std::make_error_code(std::errc::operation_not_permitted);
  if (!last_response.reject_reason.empty()) {
    os << "validated circle rejected: " << last_response.reject_reason
       << " (" << last_response.message << ")" << std::endl;
  }
  return false;
}

bool moveToSmokeSafeStart(xMateRobot &robot,
                          MotionValidator &validator,
                          ValidatedPoseChoice &move_j_choice,
                          error_code &ec) {
  const std::array<std::array<double, 6>, 5> presets{{
      kXMate3DragPose,
      {{0.00, 0.40, 0.85, 0.00, 1.15, 0.00}},
      {{0.25, 0.35, 0.90, 0.00, 1.10, 0.20}},
      {{-0.25, 0.35, 0.90, 0.00, 1.10, -0.20}},
      {{0.00, 0.65, 0.60, 0.00, 1.25, 0.00}},
  }};

  for (size_t index = 0; index < presets.size(); ++index) {
    if (!executeMoveAbsJAndWait(robot, presets[index], 180, 0, ec)) {
      return false;
    }
    os << "start preset #" << (index + 1) << " selected for probing" << std::endl;
    printArray("start joints", presets[index]);
    if (findValidatedPose(robot,
                          validator,
                          ValidateMotion::Request::MOTION_MOVE_J,
                          moveJDeltas(),
                          220,
                          5,
                          move_j_choice,
                          ec)) {
      os << "MoveJ probe accepted after preset #" << (index + 1)
         << " -> " << summarizeValidation(move_j_choice.response) << std::endl;
      ec.clear();
      return true;
    }
    ec.clear();
  }

  ec = std::make_error_code(std::errc::operation_not_permitted);
  return false;
}

}  // namespace

int main() {
  printHeader("示例 5: 笛卡尔空间运动", "官方 SDK 风格");

  error_code ec;
  xMateRobot robot;
  if (!connectRobot(robot, ec)) {
    return 1;
  }
  if (!prepareAutomaticNrt(robot, ec, 180, 5)) {
    cleanupRobot(robot);
    return 1;
  }
  MotionValidator validator;
  if (!validator.waitForService(std::chrono::seconds(10), ec) &&
      reportError("waitForService(validate_motion)", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  robot.setDefaultConfOpt(false, ec);
  if (reportError("setDefaultConfOpt", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  robot.setEventWatcher(Event::moveExecution,
                        [](const EventInfo &info) { printMoveEvent(info); },
                        ec);
  if (reportError("setEventWatcher", ec)) {
    cleanupRobot(robot);
    return 1;
  }

  printSection("1 MoveAbsJ 到起始位");
  ValidatedPoseChoice move_j_choice;
  if (!moveToSmokeSafeStart(robot, validator, move_j_choice, ec) &&
      reportError("executeCommand(MoveAbsJ start)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  printCartesianPosition(robot, ec);

  printSection("2 MoveJ 到笛卡尔目标");
  if (!executeMoveJAndWait(robot, move_j_choice.target, 220, 5, ec) &&
      reportError("executeCommand(MoveJ)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "MoveJ safe offset: ["
     << std::fixed << std::setprecision(4)
     << move_j_choice.delta.dx << ", " << move_j_choice.delta.dy << ", " << move_j_choice.delta.dz
     << "] " << summarizeValidation(move_j_choice.response) << std::endl;
  printCartesianPosition(robot, ec);

  printSection("3 MoveL 直线运动");
  ValidatedPoseChoice move_l_choice;
  if (!findValidatedPose(robot,
                         validator,
                         ValidateMotion::Request::MOTION_MOVE_L,
                         moveLDeltas(),
                         180,
                         0,
                         move_l_choice,
                         ec) &&
      reportError("validate MoveL", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  if (!executeMoveLAndWait(robot, move_l_choice.target, 180, 0, ec) &&
      reportError("executeCommand(MoveL)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "MoveL safe offset: ["
     << std::fixed << std::setprecision(4)
     << move_l_choice.delta.dx << ", " << move_l_choice.delta.dy << ", " << move_l_choice.delta.dz
     << "] " << summarizeValidation(move_l_choice.response) << std::endl;
  printCartesianPosition(robot, ec);

  printSection("4 MoveC 圆弧运动");
  ValidatedCircleChoice move_c_choice;
  if (!findValidatedCircle(robot, validator, moveCDeltas(), 160, 0, move_c_choice, ec) &&
      reportError("validate MoveC", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  if (!executeMoveCAndWait(robot, move_c_choice.aux, move_c_choice.target, 160, 0, ec) &&
      reportError("executeCommand(MoveC)", ec)) {
    cleanupRobot(robot);
    return 1;
  }
  os << "MoveC safe offsets: aux=["
     << std::fixed << std::setprecision(4)
     << move_c_choice.delta.aux.dx << ", " << move_c_choice.delta.aux.dy << ", " << move_c_choice.delta.aux.dz
     << "] target=["
     << move_c_choice.delta.target.dx << ", " << move_c_choice.delta.target.dy << ", "
     << move_c_choice.delta.target.dz << "] "
     << summarizeValidation(move_c_choice.response) << std::endl;
  printCartesianPosition(robot, ec);

  printSection("5 多段 MoveL");
  for (int step = 0; step < 3; ++step) {
    ValidatedPoseChoice step_choice;
    if (!findValidatedPose(robot,
                           validator,
                           ValidateMotion::Request::MOTION_MOVE_L,
                           guidedMoveLDeltas(step),
                           180,
                           step == 2 ? 0 : 5,
                           step_choice,
                           ec) &&
        reportError("validate multi MoveL", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    if (!executeMoveLAndWait(robot, step_choice.target, 180, step == 2 ? 0 : 5, ec) &&
        reportError("executeCommand(multi MoveL)", ec)) {
      cleanupRobot(robot);
      return 1;
    }
    os << "multi MoveL step " << (step + 1) << " offset: ["
       << std::fixed << std::setprecision(4)
       << step_choice.delta.dx << ", " << step_choice.delta.dy << ", " << step_choice.delta.dz
       << "] " << summarizeValidation(step_choice.response) << std::endl;
  }
  printCartesianPosition(robot, ec);

  printSection("6 断开连接");
  cleanupRobot(robot);
  os << "robot disconnected" << std::endl;
  return 0;
}
