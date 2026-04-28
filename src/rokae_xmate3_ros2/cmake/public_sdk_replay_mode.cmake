# ROKAE_PUBLIC_SDK_REPLAY_ONLY bootstrap for smoke-level public_sdk install replay
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

function(ament_export_dependencies)
endfunction()
function(ament_export_include_directories)
endfunction()

set(ROKAE_BUILD_COMPAT_SDK ON)
set(ROKAE_STRICT_PUBLIC_INSTALL ON)
set(ROKAE_EXPORT_PRIVATE_SDK_TARGETS OFF)
set(ROKAE_INSTALL_BACKEND_DEV_HEADERS OFF)
set(ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES OFF)
set(ROKAE_ENABLE_INTERNAL_SURFACE OFF)
set(ROKAE_ENABLE_NON_TARGET_INTERNAL_MODULES OFF)
set(ROKAE_PUBLIC_UNSUPPORTED_MODULES "io;rl;calibration")
set(ROKAE_PUBLIC_COMPAT_EXAMPLES "")
set(ROKAE_INTERNAL_BACKEND_EXAMPLES "")
set(PUBLIC_COMPAT_EXAMPLE_TARGETS public_sdk_dummy_example)
set(INTERNAL_BACKEND_EXAMPLE_TARGETS "")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROKAE_DAEMONIZED_RUNTIME_DEFAULT OFF)
if(DEFINED ROKAE_DEFAULT_RUNTIME_HOST AND ROKAE_DEFAULT_RUNTIME_HOST STREQUAL "daemonized_runtime")
  set(ROKAE_DAEMONIZED_RUNTIME_DEFAULT ON)
endif()
set(ROKAE_PUBLIC_DEFAULT_LAUNCH_PROFILE "${ROKAE_DEFAULT_PUBLIC_LAUNCH_PROFILE}")
set(ROKAE_PUBLIC_DEFAULT_RUNTIME_HOST "${ROKAE_DEFAULT_RUNTIME_HOST}")
set(ROKAE_PUBLIC_DEFAULT_RUNTIME_PROFILE "${ROKAE_DEFAULT_RUNTIME_PROFILE}")
set(ROKAE_PUBLIC_DEFAULT_BACKEND_MODE "${ROKAE_DEFAULT_BACKEND_MODE}")
set(ROKAE_PUBLIC_DEFAULT_SERVICE_EXPOSURE_PROFILE "${ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE}")
set(ROKAE_PUBLIC_DEFAULT_ENABLE_ROS2_CONTROL "${ROKAE_DEFAULT_ENABLE_ROS2_CONTROL}")
set(ROKAE_PUBLIC_DEFAULT_ENABLE_XCORE_PLUGIN "${ROKAE_DEFAULT_ENABLE_XCORE_PLUGIN}")
set(ROKAE_PUBLIC_DEFAULT_COMPATIBILITY_ALIAS_POLICY "${ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY}")
set(ROKAE_PUBLIC_PRIMARY_INSTALL_CONSUMER "cxx_sdk_core_consumer")
set(ROKAE_PUBLIC_BACKEND_MODE "${ROKAE_PUBLIC_DEFAULT_BACKEND_MODE}")

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp" "int rokae_dummy() { return 0; }
")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/dummy_main.cpp" "int main() { return 0; }\n")
# Smoke-only replay mode intentionally materializes lightweight dummy targets so the public_sdk
# component shape, shared policy propagation, and install-tree docs/tools can be verified without
# claiming real target ABI/link/install proof.
add_library(${PROJECT_NAME}_runtime_core STATIC "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp")
add_library(${PROJECT_NAME}_sdk_backend STATIC "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp")
add_executable(rokae_sim_runtime "${CMAKE_CURRENT_BINARY_DIR}/dummy_main.cpp")
add_library(xcore_controller_gazebo_plugin MODULE "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp")
add_library(xCoreSDK_core STATIC "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp")
add_library(xCoreSDK::xCoreSDK_core ALIAS xCoreSDK_core)
add_library(xCoreSDK_shared SHARED "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp")
add_library(xCoreSDK_static STATIC "${CMAKE_CURRENT_BINARY_DIR}/dummy_lib.cpp")
add_library(xCoreSDK_ros_bridge INTERFACE)
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
endif()
add_library(xCoreSDK::xCoreSDK_shared ALIAS xCoreSDK_shared)
add_library(xCoreSDK::xCoreSDK_static ALIAS xCoreSDK_static)
add_library(xCoreSDK::xCoreSDK_ros_bridge ALIAS xCoreSDK_ros_bridge)
target_link_libraries(xCoreSDK_core PUBLIC Eigen3::Eigen)
target_link_libraries(xCoreSDK_shared PUBLIC xCoreSDK_core)
target_link_libraries(xCoreSDK_static PUBLIC xCoreSDK_core)
target_link_libraries(xCoreSDK_ros_bridge INTERFACE xCoreSDK_shared)
add_executable(public_sdk_dummy_example "${CMAKE_CURRENT_BINARY_DIR}/dummy_main.cpp")

set(ROKAE_GENERATED_URDF_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/urdf")
set(ROKAE_GENERATED_XMATE_ER3_URDF "${ROKAE_GENERATED_URDF_DIR}/xMateER3.urdf")
set(ROKAE_GENERATED_XMATE_ER3_URDF_METADATA "${ROKAE_GENERATED_URDF_DIR}/xMateER3.description.json")
set(ROKAE_GENERATED_XMATE3_URDF "${ROKAE_GENERATED_URDF_DIR}/xMate3.urdf")
set(ROKAE_GENERATED_XMATE3_URDF_METADATA "${ROKAE_GENERATED_URDF_DIR}/xMate3.description.json")
file(MAKE_DIRECTORY "${ROKAE_GENERATED_URDF_DIR}")
set(_rokae_replay_urdf_template [=[<robot name="@ROKAE_REPLAY_ROBOT_NAME@" backend="@ROKAE_PUBLIC_DEFAULT_BACKEND_MODE@" profile="@ROKAE_PUBLIC_DEFAULT_SERVICE_EXPOSURE_PROFILE@" plugin="@ROKAE_PUBLIC_DEFAULT_ENABLE_XCORE_PLUGIN@" control="@ROKAE_PUBLIC_DEFAULT_ENABLE_ROS2_CONTROL@" runtime_host="@ROKAE_PUBLIC_DEFAULT_RUNTIME_HOST@" runtime_profile="@ROKAE_PUBLIC_DEFAULT_RUNTIME_PROFILE@" launch_profile="@ROKAE_PUBLIC_DEFAULT_LAUNCH_PROFILE@"/>
]=])
set(ROKAE_REPLAY_ROBOT_NAME "xMateER3")
string(CONFIGURE "${_rokae_replay_urdf_template}" _rokae_replay_xmate_er3_urdf @ONLY)
file(WRITE "${ROKAE_GENERATED_XMATE_ER3_URDF}" "${_rokae_replay_xmate_er3_urdf}")
set(ROKAE_REPLAY_ROBOT_NAME "xMate3")
string(CONFIGURE "${_rokae_replay_urdf_template}" _rokae_replay_xmate3_urdf @ONLY)
file(WRITE "${ROKAE_GENERATED_XMATE3_URDF}" "${_rokae_replay_xmate3_urdf}")
unset(_rokae_replay_urdf_template)
unset(_rokae_replay_xmate_er3_urdf)
unset(_rokae_replay_xmate3_urdf)
unset(ROKAE_REPLAY_ROBOT_NAME)

execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_description_metadata.py"
          --urdf "${ROKAE_GENERATED_XMATE_ER3_URDF}"
          --output "${ROKAE_GENERATED_XMATE_ER3_URDF_METADATA}"
          --source-xacro "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xMateER3.xacro"
          --source-xacro-package-relative urdf/xMateER3.xacro
          --enable-ros2-control "${ROKAE_PUBLIC_DEFAULT_ENABLE_ROS2_CONTROL}"
          --enable-xcore-plugin "${ROKAE_PUBLIC_DEFAULT_ENABLE_XCORE_PLUGIN}"
          --backend-mode "${ROKAE_PUBLIC_DEFAULT_BACKEND_MODE}"
          --service-exposure-profile "${ROKAE_PUBLIC_DEFAULT_SERVICE_EXPOSURE_PROFILE}"
          --robot-family xMateER3
          --robot-model xMateER3
          --identity-scope canonical
          --canonical-identity xCoreSDK:xmate_er3
          --canonical-source-xacro urdf/xMateER3.xacro
          --default-runtime-host "${ROKAE_PUBLIC_DEFAULT_RUNTIME_HOST}"
          --default-runtime-profile "${ROKAE_PUBLIC_DEFAULT_RUNTIME_PROFILE}"
          --default-launch-profile "${ROKAE_PUBLIC_DEFAULT_LAUNCH_PROFILE}"
          --compatibility-alias-policy "${ROKAE_PUBLIC_DEFAULT_COMPATIBILITY_ALIAS_POLICY}"
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_description_metadata.py"
          --urdf "${ROKAE_GENERATED_XMATE3_URDF}"
          --output "${ROKAE_GENERATED_XMATE3_URDF_METADATA}"
          --source-xacro "${CMAKE_CURRENT_SOURCE_DIR}/urdf/xMate3.xacro"
          --source-xacro-package-relative urdf/xMate3.xacro
          --enable-ros2-control "${ROKAE_PUBLIC_DEFAULT_ENABLE_ROS2_CONTROL}"
          --enable-xcore-plugin "${ROKAE_PUBLIC_DEFAULT_ENABLE_XCORE_PLUGIN}"
          --backend-mode "${ROKAE_PUBLIC_DEFAULT_BACKEND_MODE}"
          --service-exposure-profile "${ROKAE_PUBLIC_DEFAULT_SERVICE_EXPOSURE_PROFILE}"
          --robot-family xMate3
          --robot-model xMate3
          --identity-scope compatibility_alias
          --canonical-identity xCoreSDK:xmate_er3
          --canonical-source-xacro urdf/xMateER3.xacro
          --default-runtime-host "${ROKAE_PUBLIC_DEFAULT_RUNTIME_HOST}"
          --default-runtime-profile "${ROKAE_PUBLIC_DEFAULT_RUNTIME_PROFILE}"
          --default-launch-profile "${ROKAE_PUBLIC_DEFAULT_LAUNCH_PROFILE}"
          --compatibility-alias-policy "${ROKAE_PUBLIC_DEFAULT_COMPATIBILITY_ALIAS_POLICY}"
  COMMAND_ERROR_IS_FATAL ANY)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/xCoreSDKInstallMetadata.json.in
  ${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKInstallMetadata.json
  @ONLY
)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/targets_packaging.cmake)
