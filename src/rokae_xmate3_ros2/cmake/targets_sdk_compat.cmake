if(ROKAE_BUILD_COMPAT_SDK)
  set(ROKAE_COMPAT_SOURCES
    src/compat/internal/compat_shared.cpp
    src/compat/robot_api.cpp
    src/compat/model_api.cpp
    src/compat/rt_api.cpp
    src/compat/planner_api.cpp
    src/runtime/rt_command_bridge.cpp
  )

  set(ROKAE_COMPAT_PRIVATE_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  )

  add_library(xCoreSDK_static STATIC
    ${ROKAE_COMPAT_SOURCES}
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_motion_core>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_state>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_facade>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_ros_bridge>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_control_bridge>
    $<TARGET_OBJECTS:${PROJECT_NAME}_sdk_backend_objects>
  )
  add_library(xCoreSDK::xCoreSDK_static ALIAS xCoreSDK_static)
  target_include_directories(xCoreSDK_static
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
      ${ROKAE_COMPAT_PRIVATE_INCLUDE_DIRS}
  )
  ament_target_dependencies(xCoreSDK_static
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
    kdl_parser
  )
  target_link_libraries(xCoreSDK_static
    Eigen3::Eigen
    "${cpp_typesupport_target}"
    ${EIGEN3_LIBRARIES}
    ${OROCOS_KDL_LIBRARIES}
  )
  target_compile_features(xCoreSDK_static PUBLIC cxx_std_17)
  set_target_properties(xCoreSDK_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
  rokae_add_rosidl_dependency(xCoreSDK_static)

  add_library(xCoreSDK_shared SHARED
    ${ROKAE_COMPAT_SOURCES}
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_motion_core>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_state>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_facade>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_ros_bridge>
    $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_control_bridge>
    $<TARGET_OBJECTS:${PROJECT_NAME}_sdk_backend_objects>
  )
  add_library(xCoreSDK::xCoreSDK_shared ALIAS xCoreSDK_shared)
  target_include_directories(xCoreSDK_shared
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
      ${ROKAE_COMPAT_PRIVATE_INCLUDE_DIRS}
  )
  ament_target_dependencies(xCoreSDK_shared
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
    kdl_parser
  )
  target_link_libraries(xCoreSDK_shared
    Eigen3::Eigen
    "${cpp_typesupport_target}"
    ${EIGEN3_LIBRARIES}
    ${OROCOS_KDL_LIBRARIES}
  )
  target_compile_features(xCoreSDK_shared PUBLIC cxx_std_17)
  rokae_add_rosidl_dependency(xCoreSDK_shared)
endif()
