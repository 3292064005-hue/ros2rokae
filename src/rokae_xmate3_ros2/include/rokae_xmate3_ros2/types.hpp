#ifndef ROKAE_TYPES_H
#define ROKAE_TYPES_H

#include <any>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rokae {

constexpr int USE_DEFAULT = -1;
constexpr int Unknown = -1;

// ==================== Enumerations ====================
enum class OperationState {
  idle = 0,
  jog = 1,
  rtControlling = 2,
  drag = 3,
  rlProgram = 4,
  demo = 5,
  dynamicIdentify = 6,
  frictionIdentify = 7,
  loadIdentify = 8,
  moving = 9,
  jogging = 10,
  unknown = Unknown
};

enum class WorkType {
  industrial = 0,
  collaborative = 1
};

enum class OperateMode {
  manual = 0,
  automatic = 1,
  unknown = Unknown
};

enum class PowerState {
  on = 0,
  off = 1,
  estop = 2,
  gstop = 3,
  unknown = Unknown
};

enum class CoordinateType {
  flangeInBase = 0,
  endInRef = 1
};

enum class MotionControlMode : unsigned {
  Idle = 0,
  NrtCommand = 1,
  NrtRLTask = 2,
  RtCommand = 3,
};

enum class RtControllerMode : unsigned {
  jointPosition = 0,
  cartesianPosition = 1,
  jointImpedance = 2,
  cartesianImpedance = 3,
  torque = 4
};

namespace RtSupportedFields {
constexpr const char *jointPos_m = "q_m";
constexpr const char *jointPos_c = "q_c";
constexpr const char *jointVel_m = "dq_m";
constexpr const char *jointVel_c = "dq_c";
constexpr const char *jointAcc_c = "ddq_c";
constexpr const char *tcpPose_m = "pos_m";
constexpr const char *tcpPoseAbc_m = "pos_abc_m";
constexpr const char *tcpPose_c = "pos_c";
constexpr const char *tcpVel_c = "pos_vel_c";
constexpr const char *tcpAcc_c = "pos_acc_c";
constexpr const char *elbow_m = "psi_m";
constexpr const char *elbow_c = "psi_c";
constexpr const char *elbowVel_c = "psi_vel_c";
constexpr const char *elbowAcc_c = "psi_acc_c";
constexpr const char *tau_m = "tau_m";
constexpr const char *tau_c = "tau_c";
constexpr const char *tauFiltered_m = "tau_filtered_m";
constexpr const char *tauVel_c = "tau_vel_c";
constexpr const char *tauExt_inBase = "tau_ext_base";
constexpr const char *tauExt_inStiff = "tau_ext_stiff";
constexpr const char *theta_m = "theta_m";
constexpr const char *thetaVel_m = "theta_vel_m";
constexpr const char *motorTau = "motor_tau";
constexpr const char *motorTauFiltered = "motor_tau_filtered";
}  // namespace RtSupportedFields

enum class StopLevel {
  stop0 = 0,
  stop1 = 1,
  stop2 = 2,
  suppleStop = 3
};

struct DragParameter {
  enum Space {
    jointSpace = 0,
    cartesianSpace = 1
  };
  enum Type {
    translationOnly = 0,
    rotationOnly = 1,
    freely = 2
  };
};

enum class FrameType {
  world = 0,
  base = 1,
  flange = 2,
  tool = 3,
  wobj = 4,
  path = 5
};

struct JogOpt {
  enum Space {
    world = 0,
    flange = 1,
    baseFrame = 2,
    toolFrame = 3,
    wobjFrame = 4,
    jointSpace = 5,
    singularityAvoidMode = 6,
    baseParallelMode = 7
  };
};

struct xPanelOpt {
  enum Vout {
    off = 0,
    reserve = 1,
    supply12v = 2,
    supply24v = 3,
  };
};

enum class TorqueType {
  full = 0,
  inertia = 1,
  coriolis = 2,
  friction = 3,
  gravity = 4
};

typedef std::unordered_map<std::string, std::any> EventInfo;
typedef std::function<void(const EventInfo &)> EventCallback;

enum class Event {
  moveExecution = 0,
  safety = 1
};

namespace EventInfoKey {
namespace MoveExecution {
constexpr const char *ID = "cmdID";
constexpr const char *ReachTarget = "reachTarget";
constexpr const char *WaypointIndex = "wayPointIndex";
constexpr const char *Error = "error";
constexpr const char *Remark = "remark";
}  // namespace MoveExecution
namespace Safety {
constexpr const char *Collided = "collided";
}  // namespace Safety
}  // namespace EventInfoKey

// ==================== Data Types ====================
struct Info {
  std::string id;
  std::string version;
  std::string type;
  int joint_num = 6;
  std::string sdk_version;
  std::string serial_number;
  std::string model;
};

class Frame {
public:
  std::array<double, 3> trans{};
  std::array<double, 3> rpy{};
  std::array<double, 16> pos{};
  double &x;
  double &y;
  double &z;
  double &rx;
  double &ry;
  double &rz;

  Frame()
      : x(trans[0]), y(trans[1]), z(trans[2]), rx(rpy[0]), ry(rpy[1]), rz(rpy[2]) {
    syncPoseMatrix();
  }

  Frame(const Frame &other)
      : trans(other.trans), rpy(other.rpy), pos(other.pos),
        x(trans[0]), y(trans[1]), z(trans[2]), rx(rpy[0]), ry(rpy[1]), rz(rpy[2]) {}

  Frame &operator=(const Frame &other) {
    if (this != &other) {
      trans = other.trans;
      rpy = other.rpy;
      pos = other.pos;
    }
    return *this;
  }

  Frame(const std::array<double, 3> &translation, const std::array<double, 3> &rotation)
      : trans(translation), rpy(rotation), x(trans[0]), y(trans[1]), z(trans[2]), rx(rpy[0]), ry(rpy[1]), rz(rpy[2]) {
    syncPoseMatrix();
  }

  Frame(const std::array<double, 6> &frame)
      : x(trans[0]), y(trans[1]), z(trans[2]), rx(rpy[0]), ry(rpy[1]), rz(rpy[2]) {
    trans = {frame[0], frame[1], frame[2]};
    rpy = {frame[3], frame[4], frame[5]};
    syncPoseMatrix();
  }

  explicit Frame(const std::array<double, 16> &matrix)
      : pos(matrix), x(trans[0]), y(trans[1]), z(trans[2]), rx(rpy[0]), ry(rpy[1]), rz(rpy[2]) {
    syncComponentsFromMatrix();
  }

  Frame(std::initializer_list<double> values)
      : x(trans[0]), y(trans[1]), z(trans[2]), rx(rpy[0]), ry(rpy[1]), rz(rpy[2]) {
    if (values.size() == 6) {
      auto it = values.begin();
      for (size_t i = 0; i < 3; ++i, ++it) {
        trans[i] = *it;
      }
      for (size_t i = 0; i < 3; ++i, ++it) {
        rpy[i] = *it;
      }
      syncPoseMatrix();
    } else if (values.size() == 16) {
      std::copy(values.begin(), values.end(), pos.begin());
      syncComponentsFromMatrix();
    } else {
      trans = {};
      rpy = {};
      syncPoseMatrix();
    }
  }

  void syncPoseMatrix() {
    const double cx = std::cos(rpy[0]);
    const double sx = std::sin(rpy[0]);
    const double cy = std::cos(rpy[1]);
    const double sy = std::sin(rpy[1]);
    const double cz = std::cos(rpy[2]);
    const double sz = std::sin(rpy[2]);
    pos = {
      cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx, trans[0],
      sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx, trans[1],
      -sy,     cy * sx,                cy * cx,                trans[2],
      0.0,     0.0,                    0.0,                    1.0};
  }

  void syncComponentsFromMatrix() {
    trans = {pos[3], pos[7], pos[11]};
    rpy[1] = std::atan2(-pos[8], std::sqrt(pos[0] * pos[0] + pos[4] * pos[4]));
    if (std::abs(std::cos(rpy[1])) < 1e-9) {
      rpy[0] = 0.0;
      rpy[2] = std::atan2(-pos[1], pos[5]);
    } else {
      rpy[0] = std::atan2(pos[9], pos[10]);
      rpy[2] = std::atan2(pos[4], pos[0]);
    }
  }
};

class Finishable {
public:
  uint8_t isFinished() const { return finished; }
  void setFinished() { finished = 1; }

protected:
  uint8_t finished{0};
};

class CartesianPosition : public Frame, public Finishable {
public:
  struct Offset {
    enum Type {
      none = 0,
      offs = 1,
      relTool = 2,
    };

    Offset() = default;
    Offset(Type offset_type, const Frame &offset_frame) : type(offset_type), frame(offset_frame) {}
    Offset(Type offset_type, const std::array<double, 6> &offset_frame)
        : type(offset_type), frame(offset_frame) {}
    Offset(Type offset_type, std::initializer_list<double> offset_frame)
        : type(offset_type), frame(offset_frame) {}

    Type type{none};
    Frame frame{};
  };

  using Frame::Frame;

  CartesianPosition() = default;

  CartesianPosition(const CartesianPosition &other)
      : Frame(other), Finishable(other), elbow(other.elbow), hasElbow(other.hasElbow),
        confData(other.confData), external(other.external) {}

  CartesianPosition &operator=(const CartesianPosition &other) {
    if (this != &other) {
      Frame::operator=(other);
      finished = other.finished;
      elbow = other.elbow;
      hasElbow = other.hasElbow;
      confData = other.confData;
      external = other.external;
    }
    return *this;
  }

  double elbow{0.0};
  bool hasElbow{false};
  std::vector<int> confData;
  std::vector<double> external;
};

class JointPosition : public Finishable {
public:
  JointPosition() : joints(6, 0.0) {}
  JointPosition(std::initializer_list<double> values) : joints(values) {}
  explicit JointPosition(std::vector<double> values) : joints(std::move(values)) {}
  JointPosition(size_t n, double v = 0.0) : joints(n, v) {}

  std::vector<double> joints;
  std::vector<double> external;
};

class Torque : public Finishable {
public:
  Torque() : tau(6, 0.0) {}
  explicit Torque(std::vector<double> values) : tau(std::move(values)) {}

  std::vector<double> tau;
};

struct Load {
  double mass = 0.0;
  std::array<double, 3> cog{};
  std::array<double, 3> inertia{};
};

struct Toolset {
  Load load{};
  Frame end{};
  Frame ref{};
  std::string tool_name{"tool0"};
  std::string wobj_name{"wobj0"};
  std::array<double, 6> tool_pose{};
  std::array<double, 6> wobj_pose{};
};

struct FrameCalibrationResult {
  Frame frame{};
  std::array<double, 3> errors{};
  bool success = false;
};

struct RLProjectInfo {
  std::string name;
  std::vector<std::string> taskList;
  bool is_running = false;
  double run_rate = 1.0;
  bool loop_mode = false;
};

struct WorkToolInfo {
  std::string name;
  std::string alias;
  bool robotHeld = true;
  Frame pos{};
  Load load{};
};

struct MoveAbsJCommand {
  JointPosition target{};
  int speed = 100;
  int zone = 0;

  MoveAbsJCommand() = default;
  explicit MoveAbsJCommand(std::initializer_list<double> joints) : target(joints) {}
  explicit MoveAbsJCommand(const JointPosition &joint_target) : target(joint_target) {}
  explicit MoveAbsJCommand(const std::vector<double> &joints) : target(joints) {}
  MoveAbsJCommand(std::initializer_list<double> joints, int speed_value, int zone_value)
      : target(joints), speed(speed_value), zone(zone_value) {}
  MoveAbsJCommand(const JointPosition &joint_target, int speed_value, int zone_value)
      : target(joint_target), speed(speed_value), zone(zone_value) {}
  MoveAbsJCommand(const std::vector<double> &joints, int speed_value, int zone_value)
      : target(joints), speed(speed_value), zone(zone_value) {}
};

struct MoveJCommand {
  CartesianPosition target{};
  int speed = 100;
  int zone = 0;
  CartesianPosition::Offset offset{};

  MoveJCommand() = default;
  explicit MoveJCommand(const CartesianPosition &target_pose) : target(target_pose) {}
  explicit MoveJCommand(const std::array<double, 6> &target_pose) : target(target_pose) {}
  MoveJCommand(std::initializer_list<double> target_pose) : target(target_pose) {}
  MoveJCommand(const CartesianPosition &target_pose, int speed_value, int zone_value)
      : target(target_pose), speed(speed_value), zone(zone_value) {}
  MoveJCommand(const std::array<double, 6> &target_pose, int speed_value, int zone_value)
      : target(target_pose), speed(speed_value), zone(zone_value) {}
  MoveJCommand(std::initializer_list<double> target_pose, int speed_value, int zone_value)
      : target(target_pose), speed(speed_value), zone(zone_value) {}
};

struct MoveLCommand {
  CartesianPosition target{};
  int speed = 100;
  int zone = 0;
  CartesianPosition::Offset offset{};

  MoveLCommand() = default;
  explicit MoveLCommand(const CartesianPosition &target_pose) : target(target_pose) {}
  explicit MoveLCommand(const std::array<double, 6> &target_pose) : target(target_pose) {}
  MoveLCommand(std::initializer_list<double> target_pose) : target(target_pose) {}
  MoveLCommand(const CartesianPosition &target_pose, int speed_value, int zone_value)
      : target(target_pose), speed(speed_value), zone(zone_value) {}
  MoveLCommand(const std::array<double, 6> &target_pose, int speed_value, int zone_value)
      : target(target_pose), speed(speed_value), zone(zone_value) {}
  MoveLCommand(std::initializer_list<double> target_pose, int speed_value, int zone_value)
      : target(target_pose), speed(speed_value), zone(zone_value) {}
};

struct MoveCCommand {
  CartesianPosition target{};
  CartesianPosition aux{};
  int speed = 100;
  int zone = 0;
  CartesianPosition::Offset targetOffset{};
  CartesianPosition::Offset auxOffset{};

  MoveCCommand() = default;
  MoveCCommand(const CartesianPosition &target_pose, const CartesianPosition &aux_pose)
      : target(target_pose), aux(aux_pose) {}
  MoveCCommand(const CartesianPosition &target_pose, const CartesianPosition &aux_pose,
               int speed_value, int zone_value)
      : target(target_pose), aux(aux_pose), speed(speed_value), zone(zone_value) {}
};

struct MoveCFCommand {
  CartesianPosition target{};
  CartesianPosition aux{};
  int speed = 100;
  int zone = 0;
  double angle = 0.0;
  CartesianPosition::Offset targetOffset{};
  CartesianPosition::Offset auxOffset{};

  MoveCFCommand() = default;
  MoveCFCommand(const CartesianPosition &target_pose, const CartesianPosition &aux_pose)
      : target(target_pose), aux(aux_pose) {}
  MoveCFCommand(const CartesianPosition &target_pose, const CartesianPosition &aux_pose,
                double move_angle, int speed_value, int zone_value)
      : target(target_pose), aux(aux_pose), speed(speed_value), zone(zone_value), angle(move_angle) {}
};

struct MoveSPCommand {
  CartesianPosition target{};
  double radius = 0.0;
  double radius_step = 0.0;
  double angle = 0.0;
  bool direction = true;
  int speed = 100;
  int zone = 0;

  MoveSPCommand() = default;
  explicit MoveSPCommand(const CartesianPosition &target_pose) : target(target_pose) {}
  MoveSPCommand(const CartesianPosition &target_pose,
                double initial_radius,
                double radius_delta,
                double total_angle,
                bool rotate_direction,
                int speed_value = 100,
                int zone_value = 0)
      : target(target_pose), radius(initial_radius), radius_step(radius_delta), angle(total_angle),
        direction(rotate_direction), speed(speed_value), zone(zone_value) {}
};

struct LogInfo {
  enum class Level : uint8_t {
    debug = 0,
    info = 1,
    warning = 2,
    error = 3,
    fatal = 4,
  };

  std::string timestamp;
  std::string content;
  std::string repair;
  int level = static_cast<int>(Level::info);
};

}  // namespace rokae

#endif  // ROKAE_TYPES_H
