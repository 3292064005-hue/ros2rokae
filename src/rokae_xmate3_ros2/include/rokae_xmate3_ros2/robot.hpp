#ifndef ROKAE_XMATE3_ROS2_ROBOT_H
#define ROKAE_XMATE3_ROS2_ROBOT_H

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <cfloat>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include "rokae_xmate3_ros2/types.hpp"
#include "rokae_xmate3_ros2/sdk_catalog_policy.hpp"
#include "rokae_xmate3_ros2/runtime/rt_fast_command.hpp"

namespace rclcpp {
class Context;
class Executor;
class Node;
}  // namespace rclcpp


namespace rokae::ros2 {

/**
 * @brief Optional ROS integration hooks for callers that already own the ROS graph.
 *
 * The default constructors keep the historical self-managed behaviour. When one
 * of these handles is supplied, the SDK wrapper reuses the external resource
 * instead of creating its own node/context/executor.
 */
struct RosClientOptions {
    std::string node_name{"xmate3_robot"};
    std::string remote_ip{"192.168.0.160"};
    std::string local_ip;
    std::shared_ptr<rclcpp::Context> context;
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<rclcpp::Executor> executor;
    bool attach_to_executor = true;
    std::optional<SdkCatalogConsistencyPolicy> catalog_policy;
};

/**
 * @brief xCore SDK-compatible xMate3 robot facade.
 *
 * The public interface intentionally stays stable while the ROS clients,
 * runtime state caches, and action plumbing remain hidden behind a private
 * implementation to keep header coupling under control.
 */
class xMateRobot {
public:
    explicit xMateRobot(const std::string& node_name = "xmate3_robot");
    xMateRobot(const std::string& remote_ip, const std::string& local_ip = "");
    explicit xMateRobot(const RosClientOptions& options);
    ~xMateRobot();

    static std::string sdkVersion();
    bool supportsIo() const noexcept;
    bool supportsRl() const noexcept;
    bool supportsCalibration() const noexcept;
    void rememberCompatError(const std::error_code& ec) noexcept;
    void connectToRobot(std::error_code& ec);
    void disconnectFromRobot(std::error_code& ec);
    rokae::Info robotInfo(std::error_code& ec);
    rokae::PowerState powerState(std::error_code& ec);
    void setPowerState(bool on, std::error_code& ec);
    rokae::OperateMode operateMode(std::error_code& ec);
    void setOperateMode(rokae::OperateMode mode, std::error_code& ec);
    rokae::OperationState operationState(std::error_code& ec);
    std::vector<rokae::LogInfo> queryControllerLog(unsigned int count, std::error_code& ec);
    void clearServoAlarm(std::error_code& ec);
    bool getProfileCapabilities(std::string& active_profile,
                                std::vector<rokae::RuntimeProfileCapability>& profiles,
                                std::vector<rokae::RuntimeOptionDescriptor>& options,
                                std::error_code& ec);

    std::array<double, 6> jointPos(std::error_code& ec);
    std::array<double, 6> jointVel(std::error_code& ec);
    std::array<double, 6> jointTorques(std::error_code& ec);
    [[deprecated("Use jointTorques() instead")]]
    std::array<double, 6> jointTorque(std::error_code& ec);
    std::array<double, 6> posture(rokae::CoordinateType ct, std::error_code& ec);
    rokae::CartesianPosition cartPosture(rokae::CoordinateType ct, std::error_code& ec);
    std::array<double, 6> baseFrame(std::error_code& ec);
    rokae::JointPosition calcIk(const rokae::CartesianPosition& posture, std::error_code& ec);
    rokae::CartesianPosition calcFk(const rokae::JointPosition& joints, std::error_code& ec);

    rokae::Toolset toolset(std::error_code& ec);
    void setToolset(const rokae::Toolset& toolset, std::error_code& ec);
    void setToolset(const std::string& toolName, const std::string& wobjName, std::error_code& ec);
    void enableCollisionDetection(const std::array<double, 6>& sensitivity, rokae::StopLevel behaviour, double fallback, std::error_code& ec);
    void disableCollisionDetection(std::error_code& ec);
    bool getSoftLimit(std::array<std::array<double,2>,6>& limits, std::error_code& ec);
    void setSoftLimit(bool enable,
                      std::error_code& ec,
                      const std::array<std::array<double,2>,6>& limits = {{{DBL_MAX, DBL_MAX},
                                                                           {DBL_MAX, DBL_MAX},
                                                                           {DBL_MAX, DBL_MAX},
                                                                           {DBL_MAX, DBL_MAX},
                                                                           {DBL_MAX, DBL_MAX},
                                                                           {DBL_MAX, DBL_MAX}}});

    void setMotionControlMode(rokae::MotionControlMode mode, std::error_code& ec);
    void moveReset(std::error_code& ec);
    void moveStart(std::error_code& ec);
    void stop(std::error_code& ec);
    void setDefaultSpeed(int speed, std::error_code& ec);
    void setDefaultZone(int zone, std::error_code& ec);
    void setDefaultConfOpt(bool forced, std::error_code& ec);
    void adjustSpeedOnline(double scale, std::error_code& ec);
    void moveAbsJ(const rokae::MoveAbsJCommand& cmd, std::error_code& ec);
    void moveJ(const rokae::MoveJCommand& cmd, std::error_code& ec);
    void moveL(const rokae::MoveLCommand& cmd, std::error_code& ec);
    void moveC(const rokae::MoveCCommand& cmd, std::error_code& ec);
    void moveCF(const rokae::MoveCFCommand& cmd, std::error_code& ec);
    void moveSP(const rokae::MoveSPCommand& cmd, std::error_code& ec);

    void setRtControlMode(rokae::RtControllerMode mode, std::error_code& ec);
    bool getRtJointData(std::array<double, 6>& position, std::array<double, 6>& velocity, std::array<double, 6>& torque, std::error_code& ec);
    std::string sendCustomData(const std::string& topic, const std::string& payload, std::error_code& ec);
    bool publishRtFastCommand(const rokae_xmate3_ros2::runtime::RtFastCommandFrame& frame,
                              std::uint32_t& queue_depth,
                              std::error_code& ec);
    bool registerDataCallback(const std::string& data_topic, const std::string& callback_id, std::error_code& ec);

    bool getDI(unsigned int board, unsigned int port, std::error_code& ec);
    bool getDO(unsigned int board, unsigned int port, std::error_code& ec);
    void setDI(unsigned int board, unsigned int port, bool state, std::error_code& ec);
    void setDO(unsigned int board, unsigned int port, bool state, std::error_code& ec);
    double getAI(unsigned int board, unsigned int port, std::error_code& ec);
    void setAO(unsigned int board, unsigned int port, double value, std::error_code& ec);
    void setSimulationMode(bool state, std::error_code& ec);
    std::string readRegister(const std::string& key, std::error_code& ec);
    std::string readRegister(const std::string& name, int index, std::error_code& ec);
    void writeRegister(const std::string& key, const std::string& value, std::error_code& ec);
    void writeRegister(const std::string& name, int index, const std::string& value, std::error_code& ec);
    void setxPanelVout(rokae::xPanelOpt::Vout opt, std::error_code& ec);

    std::vector<rokae::RLProjectInfo> projectInfo(std::error_code& ec);
    void ppToMain(std::error_code& ec);
    void runProject(std::error_code& ec);
    void pauseProject(std::error_code& ec);
    void setProjectRunningOpt(double rate, bool loop, std::error_code& ec);
    std::vector<rokae::WorkToolInfo> toolsInfo(std::error_code& ec);
    std::vector<rokae::WorkToolInfo> wobjsInfo(std::error_code& ec);
    bool loadRLProject(const std::string& project_path, std::string& project_name, std::error_code& ec);
    bool startRLProject(const std::string& project_id, int& current_episode, std::error_code& ec);
    bool stopRLProject(const std::string& project_id, int& finished_episode, std::error_code& ec);

    void enableDrag(rokae::DragParameter::Space space, rokae::DragParameter::Type type, std::error_code& ec);
    void disableDrag(std::error_code& ec);
    void startJog(rokae::JogOpt::Space space, double rate, double step, unsigned int index, bool direction, std::error_code& ec);
    void startJog(rokae::JogOpt::Space space, unsigned int index, bool direction, double step, std::error_code& ec);
    void setAvoidSingularity(bool enable, std::error_code& ec);
    bool getAvoidSingularity(std::error_code& ec);
    std::array<double, 6> getEndEffectorTorque(std::error_code& ec);
    [[deprecated("Use getEndEffectorTorque() instead")]]
    std::array<double, 6> getEndTorque(std::error_code& ec);
    void getEndTorque(rokae::FrameType ref_type,
                      std::array<double, 6>& joint_torque_measured,
                      std::array<double, 6>& external_torque_measured,
                      std::array<double, 3>& cart_torque,
                      std::array<double, 3>& cart_force,
                      std::error_code& ec);
    bool calcJointTorque(const std::array<double, 6>& joint_pos,
                         const std::array<double, 6>& joint_vel,
                         const std::array<double, 6>& joint_acc,
                         const std::array<double, 6>& external_force,
                         std::array<double, 6>& joint_torque,
                         std::array<double, 6>& gravity_torque,
                         std::array<double, 6>& coriolis_torque,
                         std::error_code& ec);
    bool generateSTrajectory(const std::array<double, 6>& start_joint_pos,
                             const std::array<double, 6>& target_joint_pos,
                             std::vector<std::array<double, 6>>& trajectory_points,
                             double& total_time,
                             std::error_code& ec);
    bool mapCartesianToJointTorque(const std::array<double, 6>& cart_force,
                                   const std::array<double, 6>& joint_pos,
                                   std::array<double, 6>& joint_torque,
                                   std::error_code& ec);
    void startRecordPath(std::chrono::seconds duration, std::error_code& ec);
    void stopRecordPath(std::error_code& ec);
    void cancelRecordPath(std::error_code& ec);
    void saveRecordPath(const std::string& name, std::error_code& ec, const std::string& saveAs = "");
    void replayPath(const std::string& name, double rate, std::error_code& ec);
    void removePath(const std::string& name, std::error_code& ec, bool removeAll = false);
    std::vector<std::string> queryPathLists(std::error_code& ec);

    void setEventWatcher(rokae::Event eventType, const rokae::EventCallback& callback, std::error_code& ec);
    rokae::EventInfo queryEventInfo(rokae::Event eventType, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveAbsJCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveJCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveLCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveCCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveCFCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void moveAppend(const std::vector<rokae::MoveSPCommand>& cmds, std::string& cmdID, std::error_code& ec);
    void executeCommand(const std::vector<rokae::MoveAbsJCommand>& cmds, std::error_code& ec);
    void executeCommand(const std::vector<rokae::MoveJCommand>& cmds, std::error_code& ec);
    void executeCommand(const std::vector<rokae::MoveLCommand>& cmds, std::error_code& ec);
    void executeCommand(const std::vector<rokae::MoveCCommand>& cmds, std::error_code& ec);
    void setMaxCacheSize(int number, std::error_code& ec);
    std::error_code lastErrorCode() noexcept;
    void startReceiveRobotState(std::chrono::steady_clock::duration interval, const std::vector<std::string>& fields);
    void stopReceiveRobotState() noexcept;
    unsigned updateRobotState(std::chrono::steady_clock::duration timeout);
    int getStateDataArray6(const std::string& fieldName, std::array<double, 6>& data);
    int getStateDataArray3(const std::string& fieldName, std::array<double, 3>& data);
    int getStateDataMatrix16(const std::string& fieldName, std::array<double, 16>& data);
    int getStateDataScalarDouble(const std::string& fieldName, double& data);
    int getStateDataBool(const std::string& fieldName, bool& data);
    std::shared_ptr<void> model();
    std::weak_ptr<void> getRtMotionController();
    rokae::FrameCalibrationResult calibrateFrame(rokae::FrameType type,
                                                  const std::vector<std::array<double, 6>>& points,
                                                  bool is_held,
                                                  std::error_code& ec,
                                                  const std::array<double, 3>& base_aux = {});
    bool getSoftLimit(std::array<double[2], 6>& limits, std::error_code& ec);
    void setSoftLimit(bool enable, std::error_code& ec, const std::array<double[2], 6>& limits = {{DBL_MAX, DBL_MAX}});

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
};

} // namespace rokae::ros2

#endif // ROKAE_XMATE3_ROS2_ROBOT_H
