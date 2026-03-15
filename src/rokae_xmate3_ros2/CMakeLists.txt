cmake_minimum_required(VERSION 3.16)
project(rokae_xmate3_ros2)
cmake_policy(SET CMP0076 NEW)
# 抑制Gazebo CMake配置的dev警告
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)

# 生成msg/srv/action接口
rosidl_generate_interfaces(${PROJECT_NAME}
  # Msg
  msg/OperationState.msg
  msg/PowerState.msg
  msg/OperateMode.msg
  msg/CartesianPosition.msg
  msg/JointPosition.msg
  msg/Toolset.msg
  msg/MoveAbsJCommand.msg
  msg/MoveJCommand.msg
  msg/MoveLCommand.msg
  msg/MoveCCommand.msg
  msg/MoveCFCommand.msg
  msg/MoveSPCommand.msg
  msg/Info.msg
  msg/JointPos6.msg
  msg/LogInfo.msg
  # Srv
  srv/Connect.srv
  srv/Disconnect.srv
  srv/GetInfo.srv
  srv/GetPowerState.srv
  srv/SetPowerState.srv
  srv/GetOperateMode.srv
  srv/SetOperateMode.srv
  srv/GetJointPos.srv
  srv/GetJointVel.srv
  srv/GetJointTorque.srv
  srv/GetPosture.srv
  srv/GetCartPosture.srv
  srv/GetBaseFrame.srv
  srv/GetToolset.srv
  srv/SetToolset.srv
  srv/SetToolsetByName.srv
  srv/CalcIk.srv
  srv/CalcFk.srv
  srv/EnableCollisionDetection.srv
  srv/DisableCollisionDetection.srv
  srv/GetSoftLimit.srv
  srv/SetSoftLimit.srv
  srv/SetMotionControlMode.srv
  srv/MoveReset.srv
  srv/MoveStart.srv
  srv/Stop.srv
  srv/SetDefaultSpeed.srv
  srv/SetDefaultZone.srv
  srv/SetDefaultConfOpt.srv
  srv/AdjustSpeedOnline.srv
  srv/GetDI.srv
  srv/GetDO.srv
  srv/SetDI.srv
  srv/SetDO.srv
  srv/GetAI.srv
  srv/SetAO.srv
  srv/SetSimulationMode.srv
  srv/EnableDrag.srv
  srv/DisableDrag.srv
  srv/StartRecordPath.srv
  srv/StopRecordPath.srv
  srv/CancelRecordPath.srv
  srv/SaveRecordPath.srv
  srv/ReplayPath.srv
  srv/RemovePath.srv
  srv/QueryPathLists.srv
  srv/ClearServoAlarm.srv
  srv/QueryControllerLog.srv
  # 未使用但保留的srv（兼容性）
  srv/Start.srv
  srv/Reset.srv
  srv/ConnectToRobot.srv
  srv/DisconnectFromRobot.srv
  srv/GetRobotInfo.srv
  srv/GetLogInfo.srv
  srv/SetControlMode.srv
  srv/SetRtControlMode.srv
  srv/GetRtJointData.srv
  srv/SetDragParam.srv
  srv/SetCollisionDetectionParam.srv
  srv/SendCustomData.srv
  srv/RegisterDataCallback.srv
  srv/CalcJointTorque.srv
  srv/GenerateSTrajectory.srv
  srv/MapCartesianToJointTorque.srv
  srv/LoadRLProject.srv
  srv/StartRLProject.srv
  srv/StopRLProject.srv
  # Action
  action/MoveAppend.action
  action/Jog.action
  action/ReplayPath.action
  # 依赖
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces
)

# 包含目录
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# 获取接口类型支持目标
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 1. SDK库
add_library(${PROJECT_NAME}_sdk SHARED src/robot.cpp)
ament_target_dependencies(${PROJECT_NAME}_sdk
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs gazebo_ros
)
target_link_libraries(${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
  ${EIGEN3_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_compile_features(${PROJECT_NAME}_sdk PUBLIC cxx_std_17)

# 2. Gazebo插件
add_library(xcore_controller_gazebo_plugin SHARED
  src/gazebo/xcore_controller_gazebo_plugin.cpp
  src/gazebo/kinematics.cpp
  src/gazebo/trajectory_planner.cpp
)
ament_target_dependencies(xcore_controller_gazebo_plugin
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs tf2_ros gazebo_ros
)
target_link_libraries(xcore_controller_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
target_compile_features(xcore_controller_gazebo_plugin PUBLIC cxx_std_17)

# 3. 示例程序
set(EXAMPLES
  01_basic_connect
  02_joint_cartesian_read
  03_kinematics
  04_motion_basic
  05_motion_cartesian
  06_io_control
  07_safety_collision
  08_path_record_replay
  99_complete_demo
)

foreach(example IN LISTS EXAMPLES)
  add_executable(example_${example} examples/cpp/${example}.cpp)
  ament_target_dependencies(example_${example}
    rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
  )
  target_link_libraries(example_${example}
    ${PROJECT_NAME}_sdk "${cpp_typesupport_target}"
  )
  target_compile_features(example_${example} PUBLIC cxx_std_17)
endforeach()

# 安装规则
install(TARGETS
  ${PROJECT_NAME}_sdk
  xcore_controller_gazebo_plugin
  # 安装所有示例
  example_01_basic_connect
  example_02_joint_cartesian_read
  example_03_kinematics
  example_04_motion_basic
  example_05_motion_cartesian
  example_06_io_control
  example_07_safety_collision
  example_08_path_record_replay
  example_99_complete_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch meshes urdf config worlds examples
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES model.config DESTINATION share/${PROJECT_NAME})

# 导出
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  gazebo_ros
  Eigen3
)
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_sdk
  xcore_controller_gazebo_plugin
)
ament_package()
