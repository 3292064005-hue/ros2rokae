#ifndef ROKAE_TYPES_H
#define ROKAE_TYPES_H

#include <cstdint>
#include <array>
#include <vector>
#include <string>
#include <chrono>

namespace rokae {

// ==================== 8.1 枚举类型定义（严格对齐手册） ====================
/**
 * @brief 机器人工作状态（手册8.1.1）
 */
enum class OperationState : uint8_t {
  idle = 0,                ///< 空闲
  jog = 1,                 ///< 点动
  rtControlling = 2,       ///< 实时控制
  drag = 3,                ///< 拖动示教
  rlProgram = 4,           ///< 机器人语言程序运行
  moving = 5,              ///< 运动中
  jogging = 6,             ///< 连续点动
  unknown = 255            ///< 未知状态
};

/**
 * @brief 机型类别（手册8.1.2）
 */
enum class WorkType : uint8_t {
  collaborative = 0,       ///< 协作机器人
  industrial = 1,          ///< 工业机器人
  unknown = 255            ///< 未知机型
};

/**
 * @brief 机器人操作模式（手册8.1.3）
 */
enum class OperateMode : uint8_t {
  manual = 0,              ///< 手动模式
  automatic = 1,           ///< 自动模式
  unknown = 255            ///< 未知模式
};

/**
 * @brief 机器人上电及急停状态（手册8.1.4）
 */
enum class PowerState : uint8_t {
  off = 0,                 ///< 下电
  on = 1,                  ///< 上电
  estop = 2,               ///< 急停被按下
  gstop = 3,               ///< 安全门打开
  unknown = 255            ///< 未知状态
};

/**
 * @brief 位姿坐标系类型（手册8.1.5）
 */
enum class CoordinateType : uint8_t {
  flangeInBase = 0,        ///< 法兰相对于基坐标系
  endInRef = 1             ///< 末端相对于外部参考坐标系
};

/**
 * @brief 运动控制模式（手册8.1.6）
 */
enum class MotionControlMode : uint8_t {
  NrtCommand = 0,          ///< 非实时指令模式
  RtCommand = 1,           ///< 实时指令模式
  RLProgram = 2            ///< RL工程模式
};

/**
 * @brief 控制器实时控制模式（手册8.1.7）
 */
enum class RtControllerMode : uint8_t {
  jointPosition = 0,       ///< 轴空间位置控制
  cartesianPosition = 1,   ///< 笛卡尔空间位置控制
  jointImpedance = 2,      ///< 轴空间阻抗控制
  cartesianImpedance = 3,  ///< 笛卡尔空间阻抗控制
  jointTorque = 4          ///< 直接力矩控制
};

/**
 * @brief 机器人停止运动等级（手册8.1.8）
 */
enum class StopLevel : uint8_t {
  stop0 = 0,               ///< 快速停止后断电
  stop1 = 1,               ///< 规划停止后断电
  stop2 = 2,               ///< 规划停止后不断电
  suppleStop = 3           ///< 柔顺停止（仅协作机型）
};

/**
 * @brief 机器人拖动模式参数（手册8.1.9）
 */
namespace DragParameter {
  enum class Space : uint8_t {
    jointSpace = 0,        ///< 轴空间拖动
    cartesianSpace = 1     ///< 笛卡尔空间拖动
  };
  enum class Type : uint8_t {
    translationOnly = 0,   ///< 仅平移
    rotationOnly = 1,      ///< 仅旋转
    freely = 2             ///< 自由拖拽
  };
}

/**
 * @brief 坐标系类型（手册8.1.10）
 */
enum class FrameType : uint8_t {
  world = 0,               ///< 世界坐标系
  base = 1,                ///< 基坐标系
  flange = 2,              ///< 法兰坐标系
  tool = 3,                ///< 工具坐标系
  wobj = 4,                ///< 工件坐标系
  path = 5                 ///< 路径坐标系
};

/**
 * @brief Jog选项-坐标系（手册8.1.11）
 */
namespace JogOpt {
  enum class Space : uint8_t {
    world = 0,
    flange = 1,
    baseFrame = 2,
    toolFrame = 3,
    wobjFrame = 4,
    jointSpace = 5,
    singularityAvoidMode = 6,
    baseParallelMode = 7
  };
}

/**
 * @brief 力矩类型（手册8.1.13）
 */
enum class TorqueType : uint8_t {
  full = 0,                ///< 全量关节力矩
  inertia = 1,             ///< 惯性力
  coriolis = 2,            ///< 科氏力
  friction = 3,            ///< 摩擦力
  gravity = 4              ///< 重力
};

/**
 * @brief 事件类型（手册8.1.14）
 */
enum class Event : uint8_t {
  moveExecution = 0,       ///< 非实时运动指令执行信息
  safety = 1,              ///< 安全事件（碰撞等）
  unknown = 255
};

/**
 * @brief 错误码类型
 */
enum class ErrorCode : int32_t {
  success = 0,
  networkError = 1,
  connectionFailed = 2,
  serviceUnavailable = 3,
  invalidParameter = 4,
  motionError = 5,
  permissionDenied = 6,
  robotBusy = 7,
  hardwareError = 10,
  unknownError = 255
};

// ==================== 8.2 数据结构体（严格对齐手册） ====================
/**
 * @brief 机器人基本信息（手册8.2.1）
 */
struct Info {
  std::string id;                  ///< 机器人唯一ID
  std::string version;             ///< 控制器固件版本
  std::string type;                ///< 机器人机型名称
  int joint_num = 6;               ///< 关节轴数
  std::string sdk_version;         ///< SDK版本
  std::string serial_number;       ///< 机器人序列号
  std::string model;               ///< 机器人型号
};

/**
 * @brief 坐标系（手册8.2.2）
 */
/**
 * @brief 坐标系（手册8.2.2）
 */
struct Frame {
  std::array<double, 3> trans;     ///< 平移量 [X,Y,Z] 单位:米
  std::array<double, 3> rpy;       ///< XYZ欧拉角 [A,B,C] 单位:弧度
  std::array<double, 16> pos;      ///< 行优先4x4齐次变换矩阵
  FrameType type = FrameType::base;
  std::string name = "default";
};
// 原错误：enum class Frame { World, Tool, User }; 重名了
// 修正后：改名避免冲突
enum class FrameId { World, Tool, User };


struct CartesianPosition {
  std::array<double, 3> trans;     ///< 平移量 [x,y,z] 单位:米
  std::array<double, 3> rpy;       ///< 旋转量 [r,p,y] 单位:弧度
  double elbow = 0.0;               ///< 臂角（7轴机器人专用）
  bool hasElbow = false;            ///< 是否包含臂角
  std::vector<int> confData;       ///< 轴配置数据
  std::vector<double> external;     ///< 外部关节角度

  // 保留非const引用（兼容原有直接访问方式，无重复声明）
  double& x = trans[0];
  double& y = trans[1];
  double& z = trans[2];
  double& rx = rpy[0];
  double& ry = rpy[1];
  double& rz = rpy[2];

  // 提供const版本的访问接口（替代重复的const引用声明）
  const double& get_x() const { return trans[0]; }
  const double& get_y() const { return trans[1]; }
  const double& get_z() const { return trans[2]; }
  const double& get_rx() const { return rpy[0]; }
  const double& get_ry() const { return rpy[1]; }
  const double& get_rz() const { return rpy[2]; }

  /**
   * @brief 笛卡尔点位偏移量（手册8.2.4）
   */
  struct Offset {
    enum class Type { Offs, RelTool } type;
    Frame frame;
  };
};

/**
 * @brief 关节点位（手册8.2.5）
 */
struct JointPosition {
  std::vector<double> joints;       ///< 关节角度 单位:弧度
  std::vector<double> external;     ///< 外部关节角度

  // 构造函数：默认初始化6轴空间，确保joints至少有6个元素
  JointPosition() : joints(6, 0.0), external(0) {}
  JointPosition(size_t dof) : joints((dof >= 6) ? dof : 6, 0.0), external(0) {}

  // 保留非const引用（兼容原有直接访问方式），且避免越界
  double& j1 = joints[0];
  double& j2 = joints[1];
  double& j3 = joints[2];
  double& j4 = joints[3];
  double& j5 = joints[4];
  double& j6 = joints[5];

  // 提供const版本的访问接口（替代重复的const引用声明）
  const double& get_j1() const { return joints[0]; }
  const double& get_j2() const { return joints[1]; }
  const double& get_j3() const { return joints[2]; }
  const double& get_j4() const { return joints[3]; }
  const double& get_j5() const { return joints[4]; }
  const double& get_j6() const { return joints[5]; }
};
/**
 * @brief 关节扭矩（手册8.2.6）
 */
struct Torque {
  std::vector<double> joint_torques;    ///< 关节扭矩 单位:Nm
  std::vector<double> external_torques; ///< 外部轴扭矩
  TorqueType type = TorqueType::full;

  // 构造函数：默认初始化6轴空间
  Torque() : joint_torques(6, 0.0), external_torques(0) {}
  Torque(size_t dof) : joint_torques(dof, 0.0), external_torques(0) {}
};

/**
 * @brief 负载信息（手册8.2.7）
 */
struct Load {
  double mass = 0.0;                    ///< 负载质量 单位:千克
  std::array<double, 3> cog;            ///< 质心 [x,y,z] 单位:米
  std::array<double, 3> inertia;        ///< 惯量 [ix,iy,iz] 单位:千克·平方米
};

/**
 * @brief 工具工件组信息（手册8.2.8）
 */
struct Toolset {
  Load load;                             ///< 末端手持负载
  Frame end;                             ///< 末端坐标系相对法兰转换
  Frame ref;                             ///< 参考坐标系相对世界转换
  // 兼容代码逻辑扩展字段
  std::string tool_name;
  std::string wobj_name;
  std::array<double, 6> tool_pose;      ///< 工具位姿 [X,Y,Z,Rx,Ry,Rz]
  std::array<double, 6> wobj_pose;      ///< 工件位姿 [X,Y,Z,Rx,Ry,Rz]
};

/**
 * @brief 坐标系标定结果（手册8.2.9）
 */
struct FrameCalibrationResult {
  Frame frame;
  std::array<double, 3> errors;         ///< 标定偏差 [最小值,平均值,最大值]
  bool success = false;
};

/**
 * @brief RL工程信息（手册8.2.10）
 */
struct RLProjectInfo {
  std::string name;
  std::vector<std::string> taskList;
  bool is_running = false;
  double run_rate = 1.0;
  bool loop_mode = false;
};

/**
 * @brief 工具/工件信息（手册8.2.11）
 */
struct WorkToolInfo {
  std::string name;
  std::string alias;
  bool robotHeld = true;
  Frame pos;
  Load load;
};

/**
 * @brief 运动指令MoveAbsJ（手册8.2.12）
 */
struct MoveAbsJCommand {
  JointPosition target;
  int speed = 100;
  int zone = 0;
};

/**
 * @brief 运动指令MoveJ（手册8.2.13）
 */
struct MoveJCommand {
  CartesianPosition target;
  int speed = 100;
  int zone = 0;
  CartesianPosition::Offset offset;
};

/**
 * @brief 运动指令MoveL（手册8.2.14）
 */
struct MoveLCommand {
  CartesianPosition target;
  int speed = 100;
  int zone = 0;
  CartesianPosition::Offset offset;
};

/**
 * @brief 运动指令MoveC（手册8.2.15）
 */
struct MoveCCommand {
  CartesianPosition target;
  CartesianPosition aux;
  int speed = 100;
  int zone = 0;
  CartesianPosition::Offset targetOffset;
  CartesianPosition::Offset auxOffset;
};

/**
 * @brief 运动指令MoveCF（手册8.2.16）
 */
struct MoveCFCommand {
  CartesianPosition target;
  CartesianPosition aux;
  int speed = 100;
  int zone = 0;
  double angle = 0.0;
  CartesianPosition::Offset targetOffset;
  CartesianPosition::Offset auxOffset;
};

/**
 * @brief 运动指令MoveSP（手册8.2.17）
 */
struct MoveSPCommand {
  CartesianPosition target;
  double radius = 0.0;
  double radius_step = 0.0;
  double angle = 0.0;
  bool direction = true;
  int speed = 100;
  int zone = 0;
};

/**
 * @brief 控制器日志信息（手册8.2.18）
 */
struct LogInfo {
  std::string timestamp;
  std::string content;
  std::string repair;
  int level = 0;
};

} // namespace rokae

#endif // ROKAE_TYPES_H

