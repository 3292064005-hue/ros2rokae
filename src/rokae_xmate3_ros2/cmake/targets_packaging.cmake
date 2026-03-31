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
install(DIRECTORY launch meshes models urdf config worlds examples docs tools
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES model.config DESTINATION share/${PROJECT_NAME})
install(FILES "${ROKAE_GENERATED_XMATE3_URDF}"
  DESTINATION share/${PROJECT_NAME}/generated
)

# 导出
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  gazebo_ros
  Eigen3
  std_srvs
)
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}_sdk
  xcore_controller_gazebo_plugin
)
