# 3. Gazebo插件
add_library(xcore_controller_gazebo_plugin SHARED
  src/gazebo/runtime_bootstrap.cpp
  src/gazebo/xcore_controller_gazebo_plugin.cpp
)
ament_target_dependencies(xcore_controller_gazebo_plugin
  ament_index_cpp rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs control_msgs trajectory_msgs std_srvs tf2_ros gazebo_ros kdl_parser
)
target_include_directories(xcore_controller_gazebo_plugin
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(xcore_controller_gazebo_plugin
  ${GAZEBO_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OROCOS_KDL_LIBRARIES}
  ${PROJECT_NAME}_runtime_core
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
target_compile_features(xcore_controller_gazebo_plugin PUBLIC cxx_std_17)
rokae_add_rosidl_dependency(xcore_controller_gazebo_plugin)
