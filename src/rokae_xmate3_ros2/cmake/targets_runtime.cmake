# 1. 内部 runtime 私有分层（不导出私有头）
add_library(${PROJECT_NAME}_runtime_motion_core OBJECT
  src/gazebo/kinematics_backend.cpp
  src/gazebo/kinematics.cpp
  src/gazebo/trajectory_planner.cpp
  src/runtime/joint_retimer.cpp
  src/runtime/unified_retimer.cpp
  src/runtime/planner_core.cpp
  src/runtime/planner_preflight.cpp
  src/runtime/planner_trace.cpp
  src/runtime/planner_pipeline.cpp
  src/runtime/planning_utils.cpp
  src/runtime/request_adapter.cpp
  src/runtime/request_coordinator.cpp
  src/runtime/executor_core.cpp
  src/runtime/motion_runtime.cpp
  src/runtime/runtime_state_machine.cpp
  src/runtime/runtime_status_cache.cpp
  src/runtime/planner_loop.cpp
  src/runtime/runtime_execution_switch.cpp
  src/runtime/trajectory_goal_builder.cpp
)
add_dependencies(${PROJECT_NAME}_runtime_motion_core ${PROJECT_NAME}_generated_urdf)
target_compile_definitions(${PROJECT_NAME}_runtime_motion_core
  PRIVATE ROKAE_XMATE3_GENERATED_URDF_PATH="${ROKAE_GENERATED_XMATE3_URDF}"
)

add_library(${PROJECT_NAME}_runtime_state OBJECT
  src/runtime/session_state.cpp
  src/runtime/motion_options_state.cpp
  src/runtime/tooling_state.cpp
  src/runtime/data_store_state.cpp
  src/runtime/program_state.cpp
  src/runtime/diagnostics_state.cpp
  src/runtime/rt_fast_shm_ring.cpp
  src/runtime/rt_field_registry.cpp
  src/runtime/rt_subscription_plan.cpp
  src/runtime/rt_watchdog.cpp
  src/runtime/rt_prearm_checks.cpp
  src/runtime/runtime_state_utils.cpp
  src/runtime/runtime_catalog_service.cpp
  src/runtime/runtime_profile_service.cpp
  src/runtime/rt_runtime_profile.cpp
  src/runtime/rt_scheduler.cpp
  src/runtime/planning_capability_service.cpp
  src/runtime/controller_state.cpp
  src/runtime/runtime_context.cpp
  src/runtime/operation_state_adapter.cpp
)

add_library(${PROJECT_NAME}_runtime_facade OBJECT
  src/runtime/service_facade_utils.cpp
  src/runtime/control_facade.cpp
  src/runtime/query_facade.cpp
  src/runtime/query_state_service.cpp
  src/runtime/query_kinematics_service.cpp
  src/runtime/query_diagnostics_service.cpp
  src/runtime/query_catalog_service.cpp
  src/runtime/query_profile_service.cpp
  src/runtime/io_program_facade.cpp
  src/runtime/path_facade.cpp
)

add_library(${PROJECT_NAME}_runtime_ros_bridge OBJECT
  src/runtime/ros_bindings.cpp
  src/runtime/ros_service_registry.cpp
  src/runtime/compatibility_alias_registry.cpp
  src/runtime/move_append_action_registry.cpp
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
      ${OROCOS_KDL_INCLUDE_DIRS}
  )
  ament_target_dependencies(${runtime_target}
    ament_index_cpp
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
    kdl_parser
  )
  target_compile_features(${runtime_target} PUBLIC cxx_std_17)
  set_target_properties(${runtime_target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  rokae_add_rosidl_dependency(${runtime_target})
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
  ament_index_cpp
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  rosidl_runtime_cpp
  unique_identifier_msgs
  action_msgs
  kdl_parser
)
target_link_libraries(${PROJECT_NAME}_runtime_core
  ${EIGEN3_LIBRARIES}
  ${OROCOS_KDL_LIBRARIES}
  "${cpp_typesupport_target}"
)
target_compile_features(${PROJECT_NAME}_runtime_core PUBLIC cxx_std_17)
rokae_add_rosidl_dependency(${PROJECT_NAME}_runtime_core)

add_executable(rokae_sim_runtime
  src/runtime/sim_runtime_main.cpp
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_motion_core>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_state>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_facade>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_ros_bridge>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_control_bridge>
)
target_include_directories(rokae_sim_runtime
  PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(rokae_sim_runtime
  ament_index_cpp
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
  kdl_parser
)
target_link_libraries(rokae_sim_runtime
  "${cpp_typesupport_target}"
  ${EIGEN3_LIBRARIES}
  ${OROCOS_KDL_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
target_link_options(rokae_sim_runtime PRIVATE "-Wl,--disable-new-dtags")
target_compile_features(rokae_sim_runtime PUBLIC cxx_std_17)
rokae_add_rosidl_dependency(rokae_sim_runtime)

set(_rokae_runtime_ros_lib "/opt/ros/humble/lib")
if(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
  set(_rokae_runtime_ros_lib "/opt/ros/$ENV{ROS_DISTRO}/lib")
endif()
set(_rokae_runtime_install_lib "${CMAKE_INSTALL_PREFIX}/lib")
set_target_properties(rokae_sim_runtime PROPERTIES
  BUILD_RPATH_USE_ORIGIN ON
  BUILD_RPATH "\$ORIGIN/../lib;${_rokae_runtime_install_lib}"
  INSTALL_RPATH "\$ORIGIN/../lib;${_rokae_runtime_install_lib};${_rokae_runtime_ros_lib}"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)
unset(_rokae_runtime_ros_lib)
unset(_rokae_runtime_install_lib)

if(BUILD_TESTING)
  # Test-only static runtime aggregate. Unit/strict tests link this target instead of the
  # shared runtime_core so they can resolve internal runtime symbols without expanding
  # the public shared-library export surface.
  add_library(${PROJECT_NAME}_runtime_test STATIC
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_motion_core>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_state>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_facade>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_ros_bridge>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_control_bridge>
  )
  target_include_directories(${PROJECT_NAME}_runtime_test
    PUBLIC
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(${PROJECT_NAME}_runtime_test
    ament_index_cpp
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
    kdl_parser
  )
  target_link_libraries(${PROJECT_NAME}_runtime_test
    "${cpp_typesupport_target}"
    ${EIGEN3_LIBRARIES}
    ${OROCOS_KDL_LIBRARIES}
    ${GAZEBO_LIBRARIES}
  )
  target_compile_features(${PROJECT_NAME}_runtime_test PUBLIC cxx_std_17)
  rokae_add_rosidl_dependency(${PROJECT_NAME}_runtime_test)
endif()
