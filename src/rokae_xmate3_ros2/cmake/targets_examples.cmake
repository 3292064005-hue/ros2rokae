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
  rokae_add_rosidl_dependency(${example_target})
  list(APPEND EXAMPLE_TARGETS ${example_target})
endforeach()
