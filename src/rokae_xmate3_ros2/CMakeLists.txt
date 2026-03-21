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
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)

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
  srv/ReadRegister.srv
  srv/WriteRegister.srv
  srv/SetAvoidSingularity.srv
  srv/GetAvoidSingularity.srv
  srv/GetEndTorque.srv
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
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# 获取接口类型支持目标
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 1. 内部 runtime 私有分层（不导出私有头）
add_library(${PROJECT_NAME}_runtime_motion_core OBJECT
  src/gazebo/kinematics.cpp
  src/gazebo/trajectory_planner.cpp
  src/runtime/planner_core.cpp
  src/runtime/planning_utils.cpp
  src/runtime/request_adapter.cpp
  src/runtime/request_coordinator.cpp
  src/runtime/executor_core.cpp
  src/runtime/motion_runtime.cpp
)

add_library(${PROJECT_NAME}_runtime_state OBJECT
  src/runtime/runtime_state.cpp
  src/runtime/controller_state.cpp
  src/runtime/runtime_context.cpp
  src/runtime/operation_state_adapter.cpp
)

add_library(${PROJECT_NAME}_runtime_facade OBJECT
  src/runtime/service_facade.cpp
)

add_library(${PROJECT_NAME}_runtime_ros_bridge OBJECT
  src/runtime/ros_bindings.cpp
  src/runtime/runtime_publish_bridge.cpp
)

add_library(${PROJECT_NAME}_runtime_control_bridge OBJECT
  src/runtime/runtime_control_bridge.cpp
)

set(RUNTIME_PRIVATE_TARGETS
  ${PROJECT_NAME}_runtime_motion_core
  ${PROJECT_NAME}_runtime_state
  ${PROJECT_NAME}_runtime_facade
  ${PROJECT_NAME}_runtime_ros_bridge
  ${PROJECT_NAME}_runtime_control_bridge
)

foreach(runtime_target IN LISTS RUNTIME_PRIVATE_TARGETS)
  target_include_directories(${runtime_target}
    PRIVATE
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(${runtime_target}
    rclcpp
    rclcpp_action
    std_msgs
    sensor_msgs
    geometry_msgs
    control_msgs
    trajectory_msgs
    rosidl_runtime_cpp
    unique_identifier_msgs
    action_msgs
    gazebo_ros
  )
  target_compile_features(${runtime_target} PUBLIC cxx_std_17)
  set_target_properties(${runtime_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endforeach()

add_library(${PROJECT_NAME}_runtime_core SHARED
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_motion_core>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_state>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_facade>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_ros_bridge>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_control_bridge>
)
target_include_directories(${PROJECT_NAME}_runtime_core
  PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_runtime_core
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  rosidl_runtime_cpp
  unique_identifier_msgs
  action_msgs
)
target_link_libraries(${PROJECT_NAME}_runtime_core
  ${EIGEN3_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_compile_features(${PROJECT_NAME}_runtime_core PUBLIC cxx_std_17)

# 2. SDK库
add_library(${PROJECT_NAME}_sdk SHARED
  src/robot.cpp
)
ament_target_dependencies(${PROJECT_NAME}_sdk
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs control_msgs trajectory_msgs gazebo_ros
)
target_include_directories(${PROJECT_NAME}_sdk
  PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
target_link_libraries(${PROJECT_NAME}_sdk
  ${PROJECT_NAME}_runtime_core
  "${cpp_typesupport_target}"
  ${EIGEN3_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_compile_features(${PROJECT_NAME}_sdk PUBLIC cxx_std_17)

# 3. Gazebo插件
add_library(xcore_controller_gazebo_plugin SHARED
  src/gazebo/xcore_controller_gazebo_plugin.cpp
)
ament_target_dependencies(xcore_controller_gazebo_plugin
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs control_msgs trajectory_msgs tf2_ros gazebo_ros
)
target_include_directories(xcore_controller_gazebo_plugin
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(xcore_controller_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PROJECT_NAME}_runtime_core
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
  09_advanced_sdk_compat
  10_sdk_workflow_xmate3
  11_move_advanced_xmate3
  12_state_stream_threaded
  13_rl_project_workflow
  14_model_extended
  15_move_queue_and_events
  16_registers_and_runtime_options
  17_state_stream_cache
  18_toolset_and_calibration
  19_diagnostics_and_wrench
  20_rt_joint_position
  21_rt_move_commands
  22_rt_joint_impedance
  23_rt_cartesian_impedance
  24_rt_follow_position
  25_rt_s_line
  26_rt_torque_control
  99_complete_demo
)

set(EXAMPLE_TARGETS)
foreach(example IN LISTS EXAMPLES)
  set(example_target example_${example})
  add_executable(${example_target} examples/cpp/${example}.cpp)
  set_target_properties(${example_target} PROPERTIES
    RULE_LAUNCH_COMPILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/compile_slot.sh"
  )
  ament_target_dependencies(${example_target}
    rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
  )
  target_link_libraries(${example_target}
    ${PROJECT_NAME}_sdk "${cpp_typesupport_target}"
  )
  target_compile_features(${example_target} PUBLIC cxx_std_17)
  list(APPEND EXAMPLE_TARGETS ${example_target})
endforeach()

# 安装规则
install(TARGETS
  ${PROJECT_NAME}_runtime_core
  ${PROJECT_NAME}_sdk
  xcore_controller_gazebo_plugin
  ${EXAMPLE_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch meshes models urdf config worlds examples docs
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

if(BUILD_TESTING)
  option(ROKAE_ENABLE_GAZEBO_STRICT_TESTS "Enable strict Gazebo precision regression tests" OFF)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)

  if(EXISTS "/usr/bin/python3")
    set(ROKAE_LAUNCH_TEST_PYTHON "/usr/bin/python3")
  else()
    set(ROKAE_LAUNCH_TEST_PYTHON "${Python3_EXECUTABLE}")
  endif()

  ament_add_gtest(test_motion_planner_core
    test/test_motion_planner_core.cpp
  )
  if(TARGET test_motion_planner_core)
    target_include_directories(test_motion_planner_core
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_motion_planner_core
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_motion_planner_core PUBLIC cxx_std_17)
  endif()

  ament_add_gtest(test_motion_runtime_state
    test/test_motion_runtime_state.cpp
  )
  if(TARGET test_motion_runtime_state)
    target_include_directories(test_motion_runtime_state
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_motion_runtime_state
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_motion_runtime_state PUBLIC cxx_std_17)
  endif()

  ament_add_gtest(test_runtime_adapters
    test/test_runtime_adapters.cpp
  )
  ament_add_gtest(test_runtime_publish_bridge
    test/test_runtime_publish_bridge.cpp
  )
  ament_add_gtest(test_runtime_control_bridge
    test/test_runtime_control_bridge.cpp
  )
  if(TARGET test_runtime_adapters)
    target_include_directories(test_runtime_adapters
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_adapters
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_runtime_adapters PUBLIC cxx_std_17)
  endif()

  if(TARGET test_runtime_publish_bridge)
    target_include_directories(test_runtime_publish_bridge
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_publish_bridge
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_runtime_publish_bridge PUBLIC cxx_std_17)
  endif()

  if(TARGET test_runtime_control_bridge)
    target_include_directories(test_runtime_control_bridge
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_control_bridge
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_runtime_control_bridge PUBLIC cxx_std_17)
  endif()

  ament_add_gtest(test_controller_state
    test/test_controller_state.cpp
  )

  ament_add_gtest(test_request_coordinator
    test/test_request_coordinator.cpp
  )
  ament_add_gtest(test_runtime_state
    test/test_runtime_state.cpp
  )

  ament_add_gtest(test_service_facade
    test/test_service_facade.cpp
  )
  if(TARGET test_controller_state)
    target_include_directories(test_controller_state
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_controller_state
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_controller_state PUBLIC cxx_std_17)
  endif()

  if(TARGET test_request_coordinator)
    target_include_directories(test_request_coordinator
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_request_coordinator
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_request_coordinator PUBLIC cxx_std_17)
  endif()

  if(TARGET test_runtime_state)
    target_include_directories(test_runtime_state
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_state
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_runtime_state PUBLIC cxx_std_17)
  endif()

  if(TARGET test_service_facade)
    target_include_directories(test_service_facade
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_service_facade
      ${PROJECT_NAME}_runtime_core
    )
    target_compile_features(test_service_facade PUBLIC cxx_std_17)
  endif()

  add_executable(gazebo_sdk_regression_helper
    test/gazebo_sdk_regression_helper.cpp
  )
  target_include_directories(gazebo_sdk_regression_helper
    PRIVATE
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(gazebo_sdk_regression_helper
    rclcpp
    rclcpp_action
    std_msgs
    sensor_msgs
    geometry_msgs
    rosidl_runtime_cpp
    gazebo_ros
  )
  target_link_libraries(gazebo_sdk_regression_helper
    ${PROJECT_NAME}_sdk
    ${PROJECT_NAME}_runtime_core
    "${cpp_typesupport_target}"
    ${EIGEN3_LIBRARIES}
  )
  target_compile_features(gazebo_sdk_regression_helper PUBLIC cxx_std_17)

  set(GAZEBO_TEST_RESULTS_DIR "${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/gazebo")
  file(MAKE_DIRECTORY "${GAZEBO_TEST_RESULTS_DIR}")

  set(GAZEBO_SDK_LAUNCH_TEST "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/gazebo_sdk_regression_launch_test.py")
  string(CONFIGURE [=[
import unittest

import launch
import launch_testing
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable

SIMULATION_LAUNCH = r"@CMAKE_CURRENT_SOURCE_DIR@/launch/simulation.launch.py"
RUNNER = r"$<TARGET_FILE:gazebo_sdk_regression_helper>"
RESULTS_DIR = r"@GAZEBO_TEST_RESULTS_DIR@"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIMULATION_LAUNCH),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "verbose": "false",
            "enable_ros2_control": "false",
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[RUNNER, "--results-dir", RESULTS_DIR, "--mode", "default"],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason="gazebo sdk regression complete"))],
        )
    )
    launch_description = launch.LaunchDescription([
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_SHARE_DIR", PACKAGE_SHARE),
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_LIB_DIR", PACKAGE_LIB_DIR),
        SetEnvironmentVariable("PYTHONPATH", [ROSIDL_PY_PATH, ":", EnvironmentVariable("PYTHONPATH", default_value="")]),
        simulation,
        runner,
        shutdown_handler,
        launch_testing.actions.ReadyToTest(),
    ])
    return launch_description, {"runner": runner}


class TestGazeboSdkRegressionActive(unittest.TestCase):
    def test_runner_finishes(self, proc_info, runner):
        proc_info.assertWaitForShutdown(process=runner, timeout=90)


@launch_testing.post_shutdown_test()
class TestGazeboSdkRegression(unittest.TestCase):
    def test_exit_code(self, proc_info, runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=runner)
]=] GAZEBO_SDK_LAUNCH_TEST_CONTENT @ONLY)
  file(GENERATE OUTPUT "${GAZEBO_SDK_LAUNCH_TEST}" CONTENT "${GAZEBO_SDK_LAUNCH_TEST_CONTENT}")

  add_launch_test("${GAZEBO_SDK_LAUNCH_TEST}"
    TARGET gazebo_sdk_regression
    LABELS gazebo_integration
    TIMEOUT 90
    PYTHON_EXECUTABLE "${ROKAE_LAUNCH_TEST_PYTHON}"
  )
  set_tests_properties(gazebo_sdk_regression PROPERTIES RUN_SERIAL TRUE)

  if(ROKAE_ENABLE_GAZEBO_STRICT_TESTS)
    set(GAZEBO_SDK_PRECISION_LAUNCH_TEST "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/gazebo_sdk_precision_launch_test.py")
    string(CONFIGURE [=[
import unittest

import launch
import launch_testing
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable

SIMULATION_LAUNCH = r"@CMAKE_CURRENT_SOURCE_DIR@/launch/simulation.launch.py"
RUNNER = r"$<TARGET_FILE:gazebo_sdk_regression_helper>"
RESULTS_DIR = r"@GAZEBO_TEST_RESULTS_DIR@/strict"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIMULATION_LAUNCH),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "verbose": "false",
            "enable_ros2_control": "true",
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[RUNNER, "--results-dir", RESULTS_DIR, "--mode", "strict"],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason="gazebo sdk precision complete"))],
        )
    )
    launch_description = launch.LaunchDescription([
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_SHARE_DIR", PACKAGE_SHARE),
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_LIB_DIR", PACKAGE_LIB_DIR),
        SetEnvironmentVariable("PYTHONPATH", [ROSIDL_PY_PATH, ":", EnvironmentVariable("PYTHONPATH", default_value="")]),
        simulation,
        runner,
        shutdown_handler,
        launch_testing.actions.ReadyToTest(),
    ])
    return launch_description, {"runner": runner}


class TestGazeboSdkPrecisionActive(unittest.TestCase):
    def test_runner_finishes(self, proc_info, runner):
        proc_info.assertWaitForShutdown(process=runner, timeout=240)


@launch_testing.post_shutdown_test()
class TestGazeboSdkPrecision(unittest.TestCase):
    def test_exit_code(self, proc_info, runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=runner)
]=] GAZEBO_SDK_PRECISION_LAUNCH_TEST_CONTENT @ONLY)
    file(GENERATE OUTPUT "${GAZEBO_SDK_PRECISION_LAUNCH_TEST}" CONTENT "${GAZEBO_SDK_PRECISION_LAUNCH_TEST_CONTENT}")

    add_launch_test("${GAZEBO_SDK_PRECISION_LAUNCH_TEST}"
      TARGET gazebo_sdk_precision
      LABELS gazebo_integration_strict
      TIMEOUT 240
      PYTHON_EXECUTABLE "${ROKAE_LAUNCH_TEST_PYTHON}"
    )
    set_tests_properties(gazebo_sdk_precision PROPERTIES RUN_SERIAL TRUE)
  endif()

  set(GAZEBO_EXAMPLES_LAUNCH_TEST "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/gazebo_examples_smoke_launch_test.py")
  string(CONFIGURE [=[
import unittest

import launch
import launch_testing
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable

SIMULATION_LAUNCH = r"@CMAKE_CURRENT_SOURCE_DIR@/launch/simulation.launch.py"
PYTHON_BIN = r"@ROKAE_LAUNCH_TEST_PYTHON@"
RUNNER = r"@CMAKE_CURRENT_SOURCE_DIR@/test/gazebo_examples_smoke_runner.py"
EXAMPLE_04 = r"$<TARGET_FILE:example_04_motion_basic>"
EXAMPLE_05 = r"$<TARGET_FILE:example_05_motion_cartesian>"
EXAMPLE_18 = r"$<TARGET_FILE:example_18_toolset_and_calibration>"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIMULATION_LAUNCH),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "verbose": "false",
            "enable_ros2_control": "false",
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[PYTHON_BIN, RUNNER, EXAMPLE_04, EXAMPLE_05, EXAMPLE_18],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason="gazebo examples smoke complete"))],
        )
    )
    launch_description = launch.LaunchDescription([
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_SHARE_DIR", PACKAGE_SHARE),
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_LIB_DIR", PACKAGE_LIB_DIR),
        SetEnvironmentVariable("PYTHONPATH", [ROSIDL_PY_PATH, ":", EnvironmentVariable("PYTHONPATH", default_value="")]),
        simulation,
        runner,
        shutdown_handler,
        launch_testing.actions.ReadyToTest(),
    ])
    return launch_description, {"runner": runner}


class TestGazeboExamplesSmokeActive(unittest.TestCase):
    def test_runner_finishes(self, proc_info, runner):
        proc_info.assertWaitForShutdown(process=runner, timeout=180)


@launch_testing.post_shutdown_test()
class TestGazeboExamplesSmoke(unittest.TestCase):
    def test_exit_code(self, proc_info, runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=runner)
]=] GAZEBO_EXAMPLES_LAUNCH_TEST_CONTENT @ONLY)
  file(GENERATE OUTPUT "${GAZEBO_EXAMPLES_LAUNCH_TEST}" CONTENT "${GAZEBO_EXAMPLES_LAUNCH_TEST_CONTENT}")

  add_launch_test("${GAZEBO_EXAMPLES_LAUNCH_TEST}"
    TARGET gazebo_examples_smoke
    LABELS gazebo_integration
    TIMEOUT 180
    PYTHON_EXECUTABLE "${ROKAE_LAUNCH_TEST_PYTHON}"
  )
  set_tests_properties(gazebo_examples_smoke PROPERTIES RUN_SERIAL TRUE)

  set(GAZEBO_ALIAS_LAUNCH_TEST "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/gazebo_alias_smoke_launch_test.py")
  string(CONFIGURE [=[
import unittest

import launch
import launch_testing
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable

SIMULATION_LAUNCH = r"@CMAKE_CURRENT_SOURCE_DIR@/launch/xmate3_simulation.launch.py"
PYTHON_BIN = r"@ROKAE_LAUNCH_TEST_PYTHON@"
RUNNER = r"@CMAKE_CURRENT_SOURCE_DIR@/test/gazebo_alias_smoke_runner.py"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIMULATION_LAUNCH),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "verbose": "false",
            "enable_ros2_control": "false",
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[PYTHON_BIN, RUNNER],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[EmitEvent(event=Shutdown(reason="gazebo alias smoke complete"))],
        )
    )
    launch_description = launch.LaunchDescription([
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_SHARE_DIR", PACKAGE_SHARE),
        SetEnvironmentVariable("ROKAE_XMATE3_ROS2_LIB_DIR", PACKAGE_LIB_DIR),
        SetEnvironmentVariable("PYTHONPATH", [ROSIDL_PY_PATH, ":", EnvironmentVariable("PYTHONPATH", default_value="")]),
        simulation,
        runner,
        shutdown_handler,
        launch_testing.actions.ReadyToTest(),
    ])
    return launch_description, {"runner": runner}


class TestGazeboAliasSmokeActive(unittest.TestCase):
    def test_runner_finishes(self, proc_info, runner):
        proc_info.assertWaitForShutdown(process=runner, timeout=180)


@launch_testing.post_shutdown_test()
class TestGazeboAliasSmoke(unittest.TestCase):
    def test_exit_code(self, proc_info, runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=runner)
]=] GAZEBO_ALIAS_LAUNCH_TEST_CONTENT @ONLY)
  file(GENERATE OUTPUT "${GAZEBO_ALIAS_LAUNCH_TEST}" CONTENT "${GAZEBO_ALIAS_LAUNCH_TEST_CONTENT}")

  add_launch_test("${GAZEBO_ALIAS_LAUNCH_TEST}"
    TARGET gazebo_alias_smoke
    LABELS gazebo_integration
    TIMEOUT 180
    PYTHON_EXECUTABLE "${ROKAE_LAUNCH_TEST_PYTHON}"
  )
  set_tests_properties(gazebo_alias_smoke PROPERTIES RUN_SERIAL TRUE)
endif()

ament_package()
