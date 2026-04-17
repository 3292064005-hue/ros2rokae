cmake_minimum_required(VERSION 3.16)
project(rokae_xmate3_ros2)
cmake_policy(SET CMP0076 NEW)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN YES)
option(ROKAE_BUILD_COMPAT_SDK "Build official-compatible xCore SDK facade" ON)
option(ROKAE_STRICT_PUBLIC_INSTALL "Restrict the install-facing SDK surface to the public rokae/* ABI only" ON)
option(ROKAE_EXPORT_PRIVATE_SDK_TARGETS "Export native/private SDK targets in the install tree when their ROS2/Gazebo dependencies are available" OFF)
option(ROKAE_INSTALL_BACKEND_DEV_HEADERS "Install backend/developer headers" OFF)
option(ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES "Install internal/backend example binaries into the package runtime path" OFF)
option(ROKAE_PUBLIC_XMATE6_ONLY "Freeze the install-facing compatibility lane to xMate 6-axis only" ON)
option(ROKAE_DAEMONIZED_RUNTIME_DEFAULT "Publish xCoreSDK metadata with daemonized runtime as the default hosting mode" ON)
set(ROKAE_PUBLIC_UNSUPPORTED_MODULES "io;rl;calibration" CACHE STRING "Semicolon-separated public-lane modules intentionally excluded from the xMate6 compatibility contract")
if(ROKAE_DAEMONIZED_RUNTIME_DEFAULT)
  set(ROKAE_PUBLIC_BACKEND_MODE "daemonized_runtime_default")
else()
  set(ROKAE_PUBLIC_BACKEND_MODE "ros_gazebo_backed")
endif()
# 抑制Gazebo CMake配置的dev警告
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
pkg_check_modules(OROCOS_KDL REQUIRED orocos-kdl)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(ROKAE_SYSTEM_PYTHON "${Python3_EXECUTABLE}" CACHE FILEPATH
  "Interpreter used by release/test wrappers; override with -DROKAE_SYSTEM_PYTHON=/path/to/python3 or ROKAE_PYTHON_EXECUTABLE"
)
if(DEFINED ENV{ROKAE_PYTHON_EXECUTABLE} AND NOT "$ENV{ROKAE_PYTHON_EXECUTABLE}" STREQUAL "")
  set(ROKAE_SYSTEM_PYTHON "$ENV{ROKAE_PYTHON_EXECUTABLE}")
endif()
if(NOT EXISTS "${ROKAE_SYSTEM_PYTHON}")
  set(ROKAE_SYSTEM_PYTHON "${Python3_EXECUTABLE}")
endif()
set(ROKAE_CLEAN_ENV_WRAPPER "${CMAKE_CURRENT_SOURCE_DIR}/tools/clean_build_env.sh")
if(DEFINED ENV{CONDA_PREFIX} AND NOT "$ENV{CONDA_PREFIX}" STREQUAL "")
  message(WARNING
    "Conda environment detected at $ENV{CONDA_PREFIX}. "
    "Use ${ROKAE_CLEAN_ENV_WRAPPER} for release build/test/archive commands to avoid shadow-library drift."
  )
endif()

set(ROKAE_GENERATED_URDF_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/urdf")
set(ROKAE_GENERATED_XMATE3_URDF "${ROKAE_GENERATED_URDF_DIR}/xMate3.urdf")
set(ROKAE_GENERATED_XMATE3_URDF_METADATA "${ROKAE_GENERATED_URDF_DIR}/xMate3.description.json")
add_custom_command(
  OUTPUT "${ROKAE_GENERATED_XMATE3_URDF}" "${ROKAE_GENERATED_XMATE3_URDF_METADATA}"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${ROKAE_GENERATED_URDF_DIR}"
  COMMAND ${XACRO_EXECUTABLE}
          "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xMate3.xacro"
          -o "${ROKAE_GENERATED_XMATE3_URDF}"
          mesh_root:=model://rokae_xmate3_ros2/meshes/
          enable_ros2_control:=true
          enable_xcore_plugin:=true
          backend_mode:=jtc
          service_exposure_profile:=public_xmate6_only
  COMMAND "${ROKAE_SYSTEM_PYTHON}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_description_metadata.py"
          --urdf "${ROKAE_GENERATED_XMATE3_URDF}"
          --output "${ROKAE_GENERATED_XMATE3_URDF_METADATA}"
          --source-xacro "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xMate3.xacro"
          --enable-ros2-control true
          --enable-xcore-plugin true
          --backend-mode jtc
          --service-exposure-profile public_xmate6_only
          --source-xacro-package-relative urdf/xMate3.xacro
  DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xMate3.xacro"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xMate3.ros2_control.xacro"
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_generated_urdf ALL
  DEPENDS "${ROKAE_GENERATED_XMATE3_URDF}" "${ROKAE_GENERATED_XMATE3_URDF_METADATA}"
)

set(ROKAE_SOURCE_ARCHIVE "" CACHE FILEPATH
  "Optional source archive (.zip) to verify against the current working tree"
)
get_filename_component(ROKAE_SOURCE_PACKAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
get_filename_component(ROKAE_SOURCE_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(ROKAE_SOURCE_ARCHIVE_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/rokae_source_candidate.zip" CACHE FILEPATH
  "Output path for the verified source archive release candidate"
)
add_custom_target(verify_source_archive
  COMMAND "${CMAKE_COMMAND}" -E env
          "ROKAE_ENFORCE_CLEAN_ENV=1"
          "${ROKAE_CLEAN_ENV_WRAPPER}" "${ROKAE_SYSTEM_PYTHON}"
          "${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_source_archive.py"
          --package-root "${ROKAE_SOURCE_PACKAGE_ROOT}"
          --workspace-root "${ROKAE_SOURCE_WORKSPACE_ROOT}"
          --archive "${ROKAE_SOURCE_ARCHIVE}"
  WORKING_DIRECTORY "${ROKAE_SOURCE_WORKSPACE_ROOT}"
  COMMENT "Verifying packaged source archive matches the package-root release layout"
  VERBATIM
)
add_custom_target(package_verified_source_archive
  COMMAND "${CMAKE_COMMAND}" -E env
          "ROKAE_ENFORCE_CLEAN_ENV=1"
          "${ROKAE_CLEAN_ENV_WRAPPER}" "${ROKAE_SYSTEM_PYTHON}"
          "${CMAKE_CURRENT_SOURCE_DIR}/cmake/create_verified_source_archive.py"
          --package-root "${ROKAE_SOURCE_PACKAGE_ROOT}"
          --workspace-root "${ROKAE_SOURCE_WORKSPACE_ROOT}"
          --archive "${ROKAE_SOURCE_ARCHIVE_OUTPUT}"
  WORKING_DIRECTORY "${ROKAE_SOURCE_WORKSPACE_ROOT}"
  COMMENT "Creating and verifying the single allowed release-candidate source archive"
  VERBATIM
)
add_custom_target(release_candidate
  DEPENDS package_verified_source_archive
  COMMENT "Produce the only distributable verified source archive"
)
if(ROKAE_BUILD_COMPAT_SDK)
  set(ROKAE_SDK_ARTIFACT_STAGE "${CMAKE_CURRENT_BINARY_DIR}/xcoresdk_artifact_stage")
  set(ROKAE_SDK_ARTIFACT_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/xcoresdk_artifact")
  add_custom_target(assemble_sdk_artifact
    COMMAND "${CMAKE_COMMAND}" -E rm -rf "${ROKAE_SDK_ARTIFACT_STAGE}" "${ROKAE_SDK_ARTIFACT_OUTPUT}"
    COMMAND "${CMAKE_COMMAND}" --install "${CMAKE_CURRENT_BINARY_DIR}" --prefix "${ROKAE_SDK_ARTIFACT_STAGE}" --component public_sdk
    COMMAND "${ROKAE_SYSTEM_PYTHON}"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/assemble_sdk_artifact.py"
            --staging-prefix "${ROKAE_SDK_ARTIFACT_STAGE}"
            --output-dir "${ROKAE_SDK_ARTIFACT_OUTPUT}"
    COMMENT "Assembling deterministic xCoreSDK artifact from staged install"
    VERBATIM
  )
endif()
if(ROKAE_SOURCE_ARCHIVE)
  add_test(
    NAME verify_source_archive
    COMMAND "${ROKAE_CLEAN_ENV_WRAPPER}" "${ROKAE_SYSTEM_PYTHON}"
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_source_archive.py"
            --package-root "${ROKAE_SOURCE_PACKAGE_ROOT}"
            --workspace-root "${ROKAE_SOURCE_WORKSPACE_ROOT}"
            --archive "${ROKAE_SOURCE_ARCHIVE}"
  )
  set_tests_properties(verify_source_archive PROPERTIES
    LABELS "archive_quality"
    TIMEOUT 30
  )
endif()

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
  msg/RuntimeDiagnostics.msg
  msg/RtFastCommand.msg
  # Srv
  srv/Connect.srv
  srv/Disconnect.srv
  srv/GetInfo.srv
  srv/GetRuntimeDiagnostics.srv
  srv/GetRuntimeStateSnapshot.srv
  srv/GetProfileCapabilities.srv
  srv/GetPowerState.srv
  srv/SetPowerState.srv
  srv/GetOperateMode.srv
  srv/SetOperateMode.srv
  srv/GetJointPos.srv
  srv/GetJointVel.srv
  srv/GetJointTorques.srv
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
  srv/ValidateMotion.srv
  srv/ReplayPath.srv
  srv/RemovePath.srv
  srv/QueryPathLists.srv
  srv/ClearServoAlarm.srv
  srv/QueryControllerLog.srv
  srv/PrepareShutdown.srv
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
  srv/GetRlProjectInfo.srv
  srv/PauseRLProject.srv
  srv/SetProjectRunningOpt.srv
  srv/ReadRegister.srv
  srv/WriteRegister.srv
  srv/ReadRegisterEx.srv
  srv/WriteRegisterEx.srv
  srv/SetXPanelVout.srv
  srv/SetAvoidSingularity.srv
  srv/GetAvoidSingularity.srv
  srv/GetEndEffectorTorque.srv
  srv/GetEndWrench.srv
  srv/GetToolCatalog.srv
  srv/GetWobjCatalog.srv
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

set(ROKAE_ROSIDL_GENERATOR_TARGET
  ${PROJECT_NAME}__rosidl_generator_cpp
)

function(rokae_add_rosidl_dependency target_name)
  if(TARGET ${target_name} AND TARGET ${ROKAE_ROSIDL_GENERATOR_TARGET})
    add_dependencies(${target_name} ${ROKAE_ROSIDL_GENERATOR_TARGET})
  endif()
endfunction()


include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_runtime.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_sdk_backend.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_sdk_compat.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_plugin.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_examples.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_packaging.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_tests.cmake)

ament_package()
