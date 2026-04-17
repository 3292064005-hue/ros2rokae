# Backend SDK library that keeps the ROS2/Gazebo implementation details private.
set(ROKAE_SDK_BACKEND_SOURCES
  src/robot.cpp
  src/sdk/robot_clients.cpp
  src/sdk/robot_connection.cpp
  src/sdk/robot_comm.cpp
  src/sdk/robot_project.cpp
  src/sdk/robot_profiles.cpp
  src/sdk/robot_state_cache.cpp
  src/sdk/robot_state_queries.cpp
  src/sdk/robot_state.cpp
  src/sdk/robot_motion.cpp
  src/sdk/robot_io.cpp
  src/sdk/robot_path.cpp
  src/sdk/robot_motion_dispatch.cpp
  src/sdk/robot_rt.cpp
  src/sdk/robot_model.cpp
)

add_library(${PROJECT_NAME}_sdk_backend_objects OBJECT
  ${ROKAE_SDK_BACKEND_SOURCES}
)
ament_target_dependencies(${PROJECT_NAME}_sdk_backend_objects
  ament_index_cpp rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs control_msgs trajectory_msgs kdl_parser
)
target_include_directories(${PROJECT_NAME}_sdk_backend_objects
  PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME}_sdk_backend_objects
  ${PROJECT_NAME}_runtime_core
  "${cpp_typesupport_target}"
  ${EIGEN3_LIBRARIES}
  ${OROCOS_KDL_LIBRARIES}
)
target_compile_features(${PROJECT_NAME}_sdk_backend_objects PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME}_sdk_backend_objects PROPERTIES POSITION_INDEPENDENT_CODE ON)
rokae_add_rosidl_dependency(${PROJECT_NAME}_sdk_backend_objects)

add_library(${PROJECT_NAME}_sdk_backend SHARED
  $<TARGET_OBJECTS:${PROJECT_NAME}_sdk_backend_objects>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_motion_core>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_state>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_facade>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_ros_bridge>
  $<TARGET_OBJECTS:${PROJECT_NAME}_runtime_control_bridge>
)
add_library(${PROJECT_NAME}_sdk ALIAS ${PROJECT_NAME}_sdk_backend)
ament_target_dependencies(${PROJECT_NAME}_sdk_backend
  ament_index_cpp rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs control_msgs trajectory_msgs kdl_parser
)
target_include_directories(${PROJECT_NAME}_sdk_backend
  PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME}_sdk_backend
  "${cpp_typesupport_target}"
  ${EIGEN3_LIBRARIES}
  ${OROCOS_KDL_LIBRARIES}
)
target_compile_features(${PROJECT_NAME}_sdk_backend PUBLIC cxx_std_17)
rokae_add_rosidl_dependency(${PROJECT_NAME}_sdk_backend)
