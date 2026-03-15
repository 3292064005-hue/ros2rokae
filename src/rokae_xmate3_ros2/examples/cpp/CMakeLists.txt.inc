# ========================================
# 示例程序构建配置 (包含文件)
# ========================================
# 此文件由主 CMakeLists.txt 包含

# SDK手册第4.3节 - 基础连接与信息查询
add_executable(example_01_basic_connect examples/cpp/01_basic_connect.cpp)
ament_target_dependencies(example_01_basic_connect
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_01_basic_connect
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_01_basic_connect
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_01_basic_connect PUBLIC cxx_std_17)

# SDK手册第4.3节 - 关节与笛卡尔位姿读取
add_executable(example_02_joint_cartesian_read examples/cpp/02_joint_cartesian_read.cpp)
ament_target_dependencies(example_02_joint_cartesian_read
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_02_joint_cartesian_read
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_02_joint_cartesian_read
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_02_joint_cartesian_read PUBLIC cxx_std_17)

# SDK手册第4.3节 - 运动学计算 (FK/IK)
add_executable(example_03_kinematics examples/cpp/03_kinematics.cpp)
ament_target_dependencies(example_03_kinematics
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_03_kinematics
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_03_kinematics
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_03_kinematics PUBLIC cxx_std_17)

# SDK手册第4.4节 - 基础运动控制 (MoveAbsJ)
add_executable(example_04_motion_basic examples/cpp/04_motion_basic.cpp)
ament_target_dependencies(example_04_motion_basic
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_04_motion_basic
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_04_motion_basic
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_04_motion_basic PUBLIC cxx_std_17)

# SDK手册第4.4节 - 笛卡尔空间运动 (MoveJ/MoveL/MoveC)
add_executable(example_05_motion_cartesian examples/cpp/05_motion_cartesian.cpp)
ament_target_dependencies(example_05_motion_cartesian
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_05_motion_cartesian
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_05_motion_cartesian
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_05_motion_cartesian PUBLIC cxx_std_17)

# SDK手册第4.6节 - IO控制
add_executable(example_06_io_control examples/cpp/06_io_control.cpp)
ament_target_dependencies(example_06_io_control
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_06_io_control
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_06_io_control
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_06_io_control PUBLIC cxx_std_17)

# SDK手册第4.3/4.8节 - 安全与碰撞检测
add_executable(example_07_safety_collision examples/cpp/07_safety_collision.cpp)
ament_target_dependencies(example_07_safety_collision
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_07_safety_collision
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_07_safety_collision
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_07_safety_collision PUBLIC cxx_std_17)

# SDK手册第4.8节 - 路径录制与回放
add_executable(example_08_path_record_replay examples/cpp/08_path_record_replay.cpp)
ament_target_dependencies(example_08_path_record_replay
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_08_path_record_replay
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_08_path_record_replay
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_08_path_record_replay PUBLIC cxx_std_17)

# 综合示例
add_executable(example_99_complete_demo examples/cpp/99_complete_demo.cpp)
ament_target_dependencies(example_99_complete_demo
  rclcpp rclcpp_action std_msgs sensor_msgs geometry_msgs
)
target_link_libraries(example_99_complete_demo
  ${PROJECT_NAME}_sdk
  "${cpp_typesupport_target}"
)
rosidl_target_interfaces(example_99_complete_demo
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_features(example_99_complete_demo PUBLIC cxx_std_17)

# 安装所有示例
install(TARGETS
  example_01_basic_connect
  example_02_joint_cartesian_read
  example_03_kinematics
  example_04_motion_basic
  example_05_motion_cartesian
  example_06_io_control
  example_07_safety_collision
  example_08_path_record_replay
  example_99_complete_demo
  DESTINATION lib/${PROJECT_NAME}
)
