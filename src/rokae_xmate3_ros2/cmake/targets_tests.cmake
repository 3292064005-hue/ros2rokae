if(BUILD_TESTING)
  option(ROKAE_ENABLE_GAZEBO_STRICT_TESTS "Enable strict Gazebo precision regression tests" OFF)
  option(ROKAE_ENABLE_GAZEBO_FULL_EXAMPLES_TESTS "Enable full Gazebo examples harness" OFF)
  option(ROKAE_ENABLE_GAZEBO_BACKEND_MODE_TESTS "Enable Gazebo backend mode smoke harnesses" OFF)
  option(ROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS "Enable Gazebo teardown quality contract harness" OFF)
  option(ROKAE_ENABLE_RELEASE_GATE "Enable the heavier release gate bundle (backend modes + teardown + catalog/lifecycle tests)" OFF)
  option(ROKAE_ENABLE_RT_1KHZ_STRESS_TEST "Enable 10-minute RT 1kHz stress gate (slow)" OFF)
  set(ROKAE_RT_1KHZ_STRESS_DURATION_SEC 600 CACHE STRING "Duration (seconds) for rt_1khz_stress gate")
  if(ROKAE_ENABLE_RELEASE_GATE)
    set(ROKAE_ENABLE_GAZEBO_BACKEND_MODE_TESTS ON)
    set(ROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS ON)
    set(ROKAE_ENABLE_GAZEBO_FULL_EXAMPLES_TESTS ON)
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Common test target configuration macro
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/rokae_test_utils.cmake)

  if(NOT Python3_EXECUTABLE)
    message(FATAL_ERROR "Python3_EXECUTABLE must be configured for launch and harness tests")
  endif()
  set(ROKAE_LAUNCH_TEST_PYTHON "${Python3_EXECUTABLE}")
  set(ROKAE_RUNTIME_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}")

  ament_add_gtest(test_motion_planner_core
    test/unit/test_motion_planner_core.cpp
  )
  set_tests_properties(test_motion_planner_core PROPERTIES LABELS "quick_gate")

  if(TARGET test_motion_planner_core)
    target_include_directories(test_motion_planner_core
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_motion_planner_core
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_motion_planner_core PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_motion_planner_core)
  endif()

  ament_add_gtest(test_motion_runtime_state
    test/unit/test_motion_runtime_state.cpp
  )
  set_tests_properties(test_motion_runtime_state PROPERTIES LABELS "quick_gate")

  if(TARGET test_motion_runtime_state)
    target_include_directories(test_motion_runtime_state
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_motion_runtime_state
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_motion_runtime_state PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_motion_runtime_state)
  endif()

  ament_add_gtest(test_runtime_adapters
    test/unit/test_runtime_adapters.cpp
  )
  ament_add_gtest(test_runtime_publish_bridge
    test/unit/test_runtime_publish_bridge.cpp
  )
  ament_add_gtest(test_runtime_control_bridge
    test/unit/test_runtime_control_bridge.cpp
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
      ${PROJECT_NAME}_runtime_test
    )
    if(TARGET xCoreSDK::xCoreSDK_static)
      target_link_libraries(test_runtime_adapters
        xCoreSDK::xCoreSDK_static
      )
    elseif(TARGET xCoreSDK_static)
      target_link_libraries(test_runtime_adapters
        xCoreSDK_static
      )
    endif()
    target_compile_definitions(test_runtime_adapters
      PRIVATE
        ROKAE_TEST_VERIFY_SOURCE_ARCHIVE_PY="${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_source_archive.py"
        ROKAE_TEST_CREATE_VERIFIED_SOURCE_ARCHIVE_PY="${CMAKE_CURRENT_SOURCE_DIR}/cmake/create_verified_source_archive.py"
        ROKAE_TEST_PYTHON_EXECUTABLE="${ROKAE_SYSTEM_PYTHON}"
    )
    target_compile_features(test_runtime_adapters PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_adapters)
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
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_publish_bridge PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_publish_bridge)
  endif()

  set_tests_properties(test_runtime_control_bridge PROPERTIES LABELS "quick_gate;semantic_gate")

  if(TARGET test_runtime_control_bridge)
    target_include_directories(test_runtime_control_bridge
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_control_bridge
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_control_bridge PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_control_bridge)
  endif()

  ament_add_gtest(test_controller_state
    test/unit/test_controller_state.cpp
  )

  ament_add_gtest(test_request_coordinator
    test/unit/test_request_coordinator.cpp
  )
  ament_add_gtest(test_runtime_state
    test/unit/test_runtime_state.cpp
  )

  ament_add_gtest(test_runtime_diagnostics
    test/unit/test_runtime_diagnostics.cpp
  )

  ament_add_gtest(test_runtime_state_machine
    test/unit/test_runtime_state_machine.cpp
  )

  ament_add_gtest(test_planner_preflight
    test/unit/test_planner_preflight.cpp
  )

  ament_add_gtest(test_runtime_catalog_service
    test/unit/test_runtime_catalog_service.cpp
  )

  ament_add_gtest(test_rt_hardening
    test/unit/test_rt_hardening.cpp
  )

  ament_add_gtest(test_compat_state_data
    test/unit/test_compat_state_data.cpp
  )

  ament_add_gtest(test_follow_position
    test/unit/test_follow_position.cpp
  )

  ament_add_gtest(test_contract_surface
    test/unit/test_contract_surface.cpp
  )
  set_tests_properties(test_contract_surface PROPERTIES LABELS "quick_gate;semantic_gate;contract_gate")
  ament_add_gtest(test_rt_pose_comparison
    test/unit/test_rt_pose_comparison.cpp
  )

  ament_add_gtest(test_rt_movec_geometry
    test/unit/test_rt_movec_geometry.cpp
  )

  ament_add_gtest(test_implementation_audit
    test/unit/test_implementation_audit.cpp
  )

  ament_add_gtest(test_service_facade
    test/unit/test_service_facade.cpp
  )
  ament_add_gtest(test_move_queue_semantics
    test/unit/test_move_queue_semantics.cpp
  )
  ament_add_gtest(test_register_semantics
    test/unit/test_register_semantics.cpp
  )
  ament_add_gtest(test_ros_context_owner
    test/unit/test_ros_context_owner.cpp
  )
  ament_add_gtest(test_sdk_catalog_policy
    test/unit/test_sdk_catalog_policy.cpp
  )
  ament_add_gtest(test_service_registry_descriptors
    test/unit/test_service_registry_descriptors.cpp
  )

  foreach(target_name IN ITEMS test_move_queue_semantics test_register_semantics)
    if(TARGET ${target_name})
      target_include_directories(${target_name}
        PRIVATE
          include
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${EIGEN3_INCLUDE_DIRS}
      )
      target_link_libraries(${target_name}
        ${PROJECT_NAME}_runtime_test
      )
      target_compile_features(${target_name} PUBLIC cxx_std_17)
      rokae_add_rosidl_dependency(${target_name})
    endif()
  endforeach()

  set_tests_properties(test_move_queue_semantics PROPERTIES LABELS "quick_gate;semantic_gate")
  set_tests_properties(test_register_semantics PROPERTIES LABELS "quick_gate")

  foreach(target_name IN ITEMS test_ros_context_owner test_sdk_catalog_policy test_service_registry_descriptors)
    if(TARGET ${target_name})
      target_include_directories(${target_name}
        PRIVATE
          include
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${EIGEN3_INCLUDE_DIRS}
      )
      target_compile_features(${target_name} PUBLIC cxx_std_17)
      if("${target_name}" STREQUAL "test_service_registry_descriptors")
        target_link_libraries(${target_name}
          ${PROJECT_NAME}_runtime_test
        )
      endif()
    endif()
  endforeach()


  foreach(target_name IN ITEMS test_rt_pose_comparison test_rt_movec_geometry)
    if(TARGET ${target_name})
      target_include_directories(${target_name}
        PRIVATE
          include
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${EIGEN3_INCLUDE_DIRS}
      )
      target_compile_features(${target_name} PUBLIC cxx_std_17)
    endif()
  endforeach()

  foreach(target_name IN ITEMS test_ros_context_owner test_service_registry_descriptors)
    if(TARGET ${target_name})
      ament_target_dependencies(${target_name}
        rclcpp
        rclcpp_action
      )
      rokae_add_rosidl_dependency(${target_name})
    endif()
  endforeach()

  set_tests_properties(test_ros_context_owner PROPERTIES LABELS "quick_gate")
  set_tests_properties(test_sdk_catalog_policy PROPERTIES LABELS "quick_gate")
  set_tests_properties(test_service_registry_descriptors PROPERTIES LABELS "quick_gate")

add_test(
  NAME repo_contract
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/check_repo_contract.py"
)
set_tests_properties(repo_contract PROPERTIES LABELS "quick_gate")

  if(TARGET test_controller_state)
    target_include_directories(test_controller_state
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_controller_state
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_controller_state PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_controller_state)
  endif()

  set_tests_properties(test_request_coordinator PROPERTIES LABELS "quick_gate")

  if(TARGET test_request_coordinator)
    target_include_directories(test_request_coordinator
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_request_coordinator
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_request_coordinator PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_request_coordinator)
  endif()

  set_tests_properties(test_runtime_diagnostics PROPERTIES LABELS "quick_gate;semantic_gate")

  if(TARGET test_runtime_diagnostics)
    target_include_directories(test_runtime_diagnostics
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_diagnostics
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_diagnostics PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_diagnostics)
  endif()

  foreach(target_name IN ITEMS test_compat_state_data test_follow_position)
    if(TARGET ${target_name})
      target_include_directories(${target_name}
        PRIVATE
          include
          ${CMAKE_CURRENT_BINARY_DIR}
          ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${EIGEN3_INCLUDE_DIRS}
      )
      target_compile_features(${target_name} PUBLIC cxx_std_17)
      if("${target_name}" STREQUAL "test_compat_state_data")
        target_link_libraries(${target_name}
          ${PROJECT_NAME}_runtime_test
        )
        rokae_add_rosidl_dependency(${target_name})
      else()
        target_link_libraries(${target_name}
          xCoreSDK_static
        )
        rokae_add_rosidl_dependency(${target_name})
      endif()
    endif()
  endforeach()

  if(TARGET test_contract_surface)
    target_include_directories(test_contract_surface
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_compile_definitions(test_contract_surface
      PRIVATE
        ROKAE_TEST_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_compile_features(test_contract_surface PUBLIC cxx_std_17)
  endif()

  if(TARGET test_implementation_audit)
    target_include_directories(test_implementation_audit
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_compile_definitions(test_implementation_audit
      PRIVATE
        ROKAE_TEST_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    target_compile_features(test_implementation_audit PUBLIC cxx_std_17)
  endif()

  set_tests_properties(test_runtime_state_machine PROPERTIES LABELS "quick_gate")

  if(TARGET test_runtime_state_machine)
    target_include_directories(test_runtime_state_machine
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_state_machine
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_state_machine PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_state_machine)
  endif()

  set_tests_properties(test_planner_preflight PROPERTIES LABELS "quick_gate")

  if(TARGET test_planner_preflight)
    target_include_directories(test_planner_preflight
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_planner_preflight
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_planner_preflight PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_planner_preflight)
  endif()

  set_tests_properties(test_runtime_catalog_service PROPERTIES LABELS "quick_gate;semantic_gate")

  if(TARGET test_runtime_catalog_service)
    target_include_directories(test_runtime_catalog_service
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_catalog_service
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_catalog_service PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_catalog_service)
  endif()

  set_tests_properties(test_rt_hardening PROPERTIES LABELS "quick_gate;semantic_gate")

  if(TARGET test_rt_hardening)
    target_include_directories(test_rt_hardening
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_rt_hardening
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_rt_hardening PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_rt_hardening)
  endif()

  set_tests_properties(test_runtime_state PROPERTIES LABELS "quick_gate")

  if(TARGET test_runtime_state)
    target_include_directories(test_runtime_state
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_state
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_state PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_state)
  endif()

  set_tests_properties(test_service_facade PROPERTIES LABELS "quick_gate")

  if(TARGET test_service_facade)
    target_include_directories(test_service_facade
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_service_facade
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_service_facade PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_service_facade)
  endif()

  # =====================================================================
  # New unit tests (P0 optimization: expanded test coverage)
  # =====================================================================
  rokae_add_unit_test(test_kinematics_backend
    test/unit/test_kinematics_backend.cpp)

  rokae_add_unit_test(test_trajectory_planner
    test/unit/test_trajectory_planner.cpp)

  rokae_add_unit_test(test_unified_retimer
    test/unit/test_unified_retimer.cpp)

  rokae_add_unit_test(test_model_facade
    test/unit/test_model_facade.cpp)

  set_tests_properties(test_kinematics_backend PROPERTIES LABELS "semantic_gate")
  set_tests_properties(test_model_facade PROPERTIES LABELS "semantic_gate")
  set_tests_properties(test_trajectory_planner PROPERTIES LABELS "quick_gate;semantic_gate")
  set_tests_properties(test_unified_retimer PROPERTIES LABELS "quick_gate;semantic_gate")

  add_executable(gazebo_sdk_regression_helper
    test/strict/gazebo_sdk_regression_helper.cpp
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
    ${PROJECT_NAME}_runtime_test
    "${cpp_typesupport_target}"
    ${EIGEN3_LIBRARIES}
  )
  if(TARGET xCoreSDK::xCoreSDK_static)
    target_link_libraries(gazebo_sdk_regression_helper xCoreSDK::xCoreSDK_static)
  elseif(TARGET xCoreSDK_static)
    target_link_libraries(gazebo_sdk_regression_helper xCoreSDK_static)
  endif()
  target_compile_features(gazebo_sdk_regression_helper PUBLIC cxx_std_17)
  rokae_add_rosidl_dependency(gazebo_sdk_regression_helper)

  set(GAZEBO_TEST_RESULTS_DIR "${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}/gazebo")
  file(MAKE_DIRECTORY "${GAZEBO_TEST_RESULTS_DIR}")
  function(rokae_make_test_runtime_env py_out env_out)
    set(py_lines
      "        SetEnvironmentVariable(\"ROKAE_XMATE3_ROS2_SHARE_DIR\", PACKAGE_SHARE),"
      "        SetEnvironmentVariable(\"ROKAE_XMATE3_ROS2_LIB_DIR\", PACKAGE_LIB_DIR),"
      "        SetEnvironmentVariable(\"PYTHONPATH\", [ROSIDL_PY_PATH, \":\", EnvironmentVariable(\"PYTHONPATH\", default_value=\"\")]),"
      "        SetEnvironmentVariable(\"LD_LIBRARY_PATH\", [RUNTIME_LIB_DIR, \":\", EnvironmentVariable(\"LD_LIBRARY_PATH\", default_value=\"\")]),"
    )
    string(JOIN "\n" py_block ${py_lines})
    set(${py_out} "${py_block}" PARENT_SCOPE)
    set(${env_out}
      "ROKAE_XMATE3_ROS2_SHARE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
      "ROKAE_XMATE3_ROS2_LIB_DIR=$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
      "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:$ENV{PYTHONPATH}"
      "LD_LIBRARY_PATH=${ROKAE_RUNTIME_LIB_DIR}:$ENV{LD_LIBRARY_PATH}"
      PARENT_SCOPE
    )
  endfunction()
  rokae_make_test_runtime_env(ROKAE_TEST_RUNTIME_ENV_PY ROKAE_TEST_RUNTIME_ENV)

  set(GAZEBO_SDK_LAUNCH_TEST "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/gazebo_sdk_regression_launch_test.py")
  string(CONFIGURE [=[
import unittest

import launch
import launch_testing
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
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
RUNTIME_LIB_DIR = r"@ROKAE_RUNTIME_LIB_DIR@"


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIMULATION_LAUNCH),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "verbose": "false",
            "enable_ros2_control": "true",
            "enable_xcore_plugin": "true",
            "backend_mode": "hybrid",
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
        SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11454"),
@ROKAE_TEST_RUNTIME_ENV_PY@
        simulation,
        TimerAction(period=6.0, actions=[runner]),
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
  set_tests_properties(gazebo_sdk_regression PROPERTIES RUN_SERIAL TRUE LABELS "quick_gate")

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
RUNTIME_LIB_DIR = r"@ROKAE_RUNTIME_LIB_DIR@"


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
@ROKAE_TEST_RUNTIME_ENV_PY@
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
RUNNER = r"@CMAKE_CURRENT_SOURCE_DIR@/test/harness/gazebo_examples_smoke_runner.py"
EXAMPLE_04 = r"$<TARGET_FILE:example_04_motion_basic>"
EXAMPLE_05 = r"$<TARGET_FILE:example_05_motion_cartesian>"
EXAMPLE_18 = r"$<TARGET_FILE:example_18_toolset_only>"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"
RUNTIME_LIB_DIR = r"@ROKAE_RUNTIME_LIB_DIR@"


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
        cmd=[PYTHON_BIN, RUNNER, EXAMPLE_04, EXAMPLE_05, EXAMPLE_18],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[
                launch.actions.TimerAction(
                    period=3.0,
                    actions=[EmitEvent(event=Shutdown(reason="gazebo examples smoke complete"))],
                )
            ],
        )
    )
    launch_description = launch.LaunchDescription([
@ROKAE_TEST_RUNTIME_ENV_PY@
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
  set_tests_properties(gazebo_examples_smoke PROPERTIES RUN_SERIAL TRUE LABELS "quick_gate")

  if(ROKAE_ENABLE_GAZEBO_FULL_EXAMPLES_TESTS)
    set(GAZEBO_EXAMPLES_FULL_COMMAND
      "${CMAKE_COMMAND}" "-E" "env"
      ${ROKAE_TEST_RUNTIME_ENV}
      "${ROKAE_LAUNCH_TEST_PYTHON}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/run_gazebo_examples_full_test.py"
      "--simulation-launch" "${CMAKE_CURRENT_SOURCE_DIR}/launch/simulation.launch.py"
      "--launch-runner" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/run_launch_service.py"
      "--runner" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/gazebo_examples_full_runner.py"
      "--package-share" "${CMAKE_CURRENT_SOURCE_DIR}"
      "--package-lib-dir" "$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
      "--rosidl-python-path" "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
      "--runtime-lib-dir" "${ROKAE_RUNTIME_LIB_DIR}"
      "--log-file" "${CMAKE_CURRENT_BINARY_DIR}/launch_test/gazebo_examples_full.txt"
      "--python-bin" "${ROKAE_LAUNCH_TEST_PYTHON}"
    )
    foreach(example_target IN LISTS EXAMPLE_TARGETS)
      list(APPEND GAZEBO_EXAMPLES_FULL_COMMAND "--example" "$<TARGET_FILE:${example_target}>")
    endforeach()

    add_test(
      NAME gazebo_examples_full
      COMMAND ${GAZEBO_EXAMPLES_FULL_COMMAND}
    )
    set_tests_properties(gazebo_examples_full PROPERTIES RUN_SERIAL TRUE)
    set_tests_properties(gazebo_examples_full PROPERTIES
      LABELS "gazebo_integration_examples_full;release_gate"
      TIMEOUT 1800
    )
  endif()

  if(ROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS)
    add_test(
      NAME gazebo_teardown_quality
      COMMAND
        "${CMAKE_COMMAND}" "-E" "env"
        ${ROKAE_TEST_RUNTIME_ENV}
        "${ROKAE_LAUNCH_TEST_PYTHON}"
        "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/gazebo_teardown_quality_runner.py"
        "--simulation-launch" "${CMAKE_CURRENT_SOURCE_DIR}/launch/simulation.launch.py"
        "--launch-runner" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/run_launch_service.py"
        "--package-share" "${CMAKE_CURRENT_SOURCE_DIR}"
        "--package-lib-dir" "$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
        "--rosidl-python-path" "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
        "--runtime-lib-dir" "${ROKAE_RUNTIME_LIB_DIR}"
        "--log-file" "${CMAKE_CURRENT_BINARY_DIR}/launch_test/gazebo_teardown_quality.txt"
        "--python-bin" "${ROKAE_LAUNCH_TEST_PYTHON}"
    )
    set_tests_properties(gazebo_teardown_quality PROPERTIES
      RUN_SERIAL TRUE
      LABELS "gazebo_teardown_quality;release_gate"
      TIMEOUT 240
    )

    add_test(
      NAME gazebo_teardown_quality_repeat
      COMMAND
        "${CMAKE_COMMAND}" "-E" "env"
        ${ROKAE_TEST_RUNTIME_ENV}
        "${ROKAE_LAUNCH_TEST_PYTHON}"
        "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/gazebo_teardown_quality_runner.py"
        "--simulation-launch" "${CMAKE_CURRENT_SOURCE_DIR}/launch/simulation.launch.py"
        "--launch-runner" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/run_launch_service.py"
        "--package-share" "${CMAKE_CURRENT_SOURCE_DIR}"
        "--package-lib-dir" "$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
        "--rosidl-python-path" "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
        "--runtime-lib-dir" "${ROKAE_RUNTIME_LIB_DIR}"
        "--log-file" "${CMAKE_CURRENT_BINARY_DIR}/launch_test/gazebo_teardown_quality_repeat.txt"
        "--python-bin" "${ROKAE_LAUNCH_TEST_PYTHON}"
        "--repeat" "10"
    )
    set_tests_properties(gazebo_teardown_quality_repeat PROPERTIES
      RUN_SERIAL TRUE
      LABELS "gazebo_teardown_quality_repeat"
      TIMEOUT 1800
    )
  endif()

  if(ROKAE_ENABLE_GAZEBO_BACKEND_MODE_TESTS)
    foreach(GAZEBO_BACKEND_MODE IN ITEMS effort jtc hybrid)
      string(TOUPPER "${GAZEBO_BACKEND_MODE}" GAZEBO_BACKEND_MODE_UPPER)
      set(GAZEBO_BACKEND_ENABLE_PLUGIN "true")
      if(GAZEBO_BACKEND_MODE STREQUAL "jtc")
        set(GAZEBO_BACKEND_ENABLE_PLUGIN "false")
      endif()
      set(GAZEBO_BACKEND_MODE_LAUNCH_TEST
          "${CMAKE_CURRENT_BINARY_DIR}/generated_tests/gazebo_backend_mode_${GAZEBO_BACKEND_MODE}_launch_test.py")
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
RUNNER = r"@CMAKE_CURRENT_SOURCE_DIR@/test/harness/gazebo_backend_mode_smoke_runner.py"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"
RUNTIME_LIB_DIR = r"@ROKAE_RUNTIME_LIB_DIR@"


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIMULATION_LAUNCH),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "verbose": "false",
            "enable_ros2_control": "true",
            "enable_xcore_plugin": "@GAZEBO_BACKEND_ENABLE_PLUGIN@",
            "backend_mode": "@GAZEBO_BACKEND_MODE@",
        }.items(),
    )
    runner = ExecuteProcess(
        cmd=[PYTHON_BIN, RUNNER, "@GAZEBO_BACKEND_MODE@"],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[
                launch.actions.TimerAction(
                    period=3.0,
                    actions=[EmitEvent(event=Shutdown(reason="gazebo backend mode smoke complete"))],
                )
            ],
        )
    )
    launch_description = launch.LaunchDescription([
@ROKAE_TEST_RUNTIME_ENV_PY@
        simulation,
        runner,
        shutdown_handler,
        launch_testing.actions.ReadyToTest(),
    ])
    return launch_description, {"runner": runner}


class TestGazeboBackendModeSmokeActive(unittest.TestCase):
    def test_runner_finishes(self, proc_info, runner):
        proc_info.assertWaitForShutdown(process=runner, timeout=120)


@launch_testing.post_shutdown_test()
class TestGazeboBackendModeSmoke(unittest.TestCase):
    def test_exit_code(self, proc_info, runner):
        launch_testing.asserts.assertExitCodes(proc_info, process=runner)
]=] GAZEBO_BACKEND_MODE_LAUNCH_TEST_CONTENT @ONLY)
      file(GENERATE OUTPUT "${GAZEBO_BACKEND_MODE_LAUNCH_TEST}" CONTENT "${GAZEBO_BACKEND_MODE_LAUNCH_TEST_CONTENT}")

      add_launch_test("${GAZEBO_BACKEND_MODE_LAUNCH_TEST}"
        TARGET "gazebo_backend_mode_${GAZEBO_BACKEND_MODE}_smoke"
        LABELS gazebo_integration_modes
        TIMEOUT 120
        PYTHON_EXECUTABLE "${ROKAE_LAUNCH_TEST_PYTHON}"
      )
      set_tests_properties("gazebo_backend_mode_${GAZEBO_BACKEND_MODE}_smoke" PROPERTIES RUN_SERIAL TRUE)
    endforeach()
  endif()

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
RUNNER = r"@CMAKE_CURRENT_SOURCE_DIR@/test/harness/gazebo_alias_smoke_runner.py"
PACKAGE_SHARE = r"@CMAKE_CURRENT_SOURCE_DIR@"
PACKAGE_LIB_DIR = r"$<TARGET_FILE_DIR:xcore_controller_gazebo_plugin>"
ROSIDL_PY_PATH = r"@CMAKE_CURRENT_BINARY_DIR@/rosidl_generator_py"
RUNTIME_LIB_DIR = r"@ROKAE_RUNTIME_LIB_DIR@"


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
        cmd=[PYTHON_BIN, RUNNER],
        output="screen",
    )
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=runner,
            on_exit=[
                launch.actions.TimerAction(
                    period=3.0,
                    actions=[EmitEvent(event=Shutdown(reason="gazebo alias smoke complete"))],
                )
            ],
        )
    )
    launch_description = launch.LaunchDescription([
@ROKAE_TEST_RUNTIME_ENV_PY@
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


if(BUILD_TESTING)
  ament_add_gtest(test_runtime_profile_service
    test/unit/test_runtime_profile_service.cpp
  )

  ament_add_gtest(test_rt_runtime_profile
    test/unit/test_rt_runtime_profile.cpp
  )

  ament_add_gtest(test_runtime_profile_capabilities_contract
    test/unit/test_runtime_profile_capabilities_contract.cpp
  )

  if(TARGET test_runtime_profile_capabilities_contract)
    target_include_directories(test_runtime_profile_capabilities_contract
      PRIVATE
        include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(test_runtime_profile_capabilities_contract
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_profile_capabilities_contract PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_profile_capabilities_contract)
  endif()

  if(TARGET test_rt_runtime_profile)
    target_include_directories(test_rt_runtime_profile PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_rt_runtime_profile
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_rt_runtime_profile PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_rt_runtime_profile)
  endif()

  if(TARGET test_runtime_profile_service)
    target_include_directories(test_runtime_profile_service PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_runtime_profile_service
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_profile_service PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_profile_service)
  endif()

  ament_add_gtest(test_runtime_option_catalog
    test/unit/test_runtime_option_catalog.cpp
  )
  if(TARGET test_runtime_option_catalog)
    target_include_directories(test_runtime_option_catalog PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_runtime_option_catalog
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(test_runtime_option_catalog PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(test_runtime_option_catalog)
  endif()
endif()


if(BUILD_TESTING)
  add_test(
    NAME runtime_diag_gate_tools
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/check_runtime_diag_gate_tools.py"
  )
  set_tests_properties(runtime_diag_gate_tools PROPERTIES LABELS "quick_gate;semantic_gate")

  if(ROKAE_ENABLE_RT_1KHZ_STRESS_TEST AND TARGET example_27_rt_1khz_stress)
    math(EXPR ROKAE_RT_1KHZ_STRESS_TIMEOUT "${ROKAE_RT_1KHZ_STRESS_DURATION_SEC} + 240")
    add_test(
      NAME rt_1khz_stress
      COMMAND "${CMAKE_COMMAND}" -E env
              ${ROKAE_TEST_RUNTIME_ENV}
              bash "${CMAKE_CURRENT_SOURCE_DIR}/tools/run_rt_1khz_stress.sh"
              "${CMAKE_CURRENT_SOURCE_DIR}/../.."
              "${ROKAE_RT_1KHZ_STRESS_DURATION_SEC}"
              "5"
    )
    set_tests_properties(rt_1khz_stress PROPERTIES
      RUN_SERIAL TRUE
      TIMEOUT "${ROKAE_RT_1KHZ_STRESS_TIMEOUT}"
      LABELS "rt_stress;release_gate"
    )
  endif()
endif()

if(BUILD_TESTING AND ROKAE_BUILD_COMPAT_SDK)
  add_test(
    NAME compat_public_abi
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/check_compat_public_abi.py"
  )
  set_tests_properties(compat_public_abi PROPERTIES LABELS "quick_gate;abi_gate")

  add_test(
    NAME xmate6_official_alignment
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/check_xmate6_official_alignment.py"
  )
  set_tests_properties(xmate6_official_alignment PROPERTIES LABELS "quick_gate;abi_gate")

  add_test(
    NAME compat_install_tree_consumer
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/run_install_tree_consumer.py"
            --build-dir "${CMAKE_CURRENT_BINARY_DIR}"
            --consumer-dir "${CMAKE_CURRENT_SOURCE_DIR}/test/compat/install_tree"
            --staging-prefix "${CMAKE_CURRENT_BINARY_DIR}/compat_install_tree_stage"
  )
  set_tests_properties(compat_install_tree_consumer PROPERTIES LABELS "abi_gate;install_tree" TIMEOUT 600)

  add_test(
    NAME compat_no_ros_env_external_consumer
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/run_no_ros_env_consumer.py"
            --build-dir "${CMAKE_CURRENT_BINARY_DIR}"
            --consumer-dir "${CMAKE_CURRENT_SOURCE_DIR}/test/compat/install_tree"
            --staging-prefix "${CMAKE_CURRENT_BINARY_DIR}/compat_no_ros_env_stage"
  )
  set_tests_properties(compat_no_ros_env_external_consumer PROPERTIES LABELS "abi_gate;install_tree" TIMEOUT 600)

  add_test(
    NAME compat_exported_symbols
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/harness/check_exported_symbols.py"
            --library "$<TARGET_FILE:xCoreSDK_shared>"
  )
  set_tests_properties(compat_exported_symbols PROPERTIES LABELS "abi_gate;symbol_gate")
endif()
