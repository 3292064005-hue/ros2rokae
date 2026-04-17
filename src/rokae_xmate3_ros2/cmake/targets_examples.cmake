# 3. 示例程序
option(ROKAE_BUILD_PUBLIC_COMPAT_EXAMPLES "Build install-facing compatibility examples" ON)
option(ROKAE_BUILD_INTERNAL_BACKEND_EXAMPLES "Build backend/internal-only examples" ON)

set(ROKAE_PUBLIC_COMPAT_EXAMPLES
  01_basic_connect
  02_joint_cartesian_read
  03_kinematics
  04_motion_basic
  07_safety_collision
  09_advanced_sdk_compat
  10_sdk_workflow_xmate3
  11_move_advanced_xmate3
  12_state_stream_threaded
  14_model_extended
  15_move_queue_and_events
  17_state_stream_cache
  18_toolset_only
  19_diagnostics_and_wrench
  99_complete_demo
)

set(ROKAE_INTERNAL_BACKEND_EXAMPLES
  05_motion_cartesian
  06_io_control
  08_path_record_replay
  13_rl_project_workflow
  16_registers_and_runtime_options
  20_rt_joint_position
  21_rt_move_commands
  22_rt_joint_impedance
  23_rt_cartesian_impedance
  24_rt_follow_position
  25_rt_s_line
  26_rt_torque_control
  27_rt_1khz_stress
)

set(EXAMPLE_TARGETS)
set(PUBLIC_COMPAT_EXAMPLE_TARGETS)
set(INTERNAL_BACKEND_EXAMPLE_TARGETS)

function(rokae_add_example example_name link_mode)
  set(example_target example_${example_name})
  add_executable(${example_target} examples/cpp/${example_name}.cpp)
  set_target_properties(${example_target} PROPERTIES
    RULE_LAUNCH_COMPILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/compile_slot.sh"
  )
  target_compile_features(${example_target} PUBLIC cxx_std_17)

  if("${link_mode}" STREQUAL "public")
    if(ROKAE_BUILD_COMPAT_SDK)
      target_link_libraries(${example_target}
        xCoreSDK::xCoreSDK_static
      )
    else()
      target_link_libraries(${example_target}
        ${PROJECT_NAME}_sdk_backend
        "${cpp_typesupport_target}"
      )
      ament_target_dependencies(${example_target}
        rclcpp
        rclcpp_action
        std_msgs
        sensor_msgs
        geometry_msgs
      )
      rokae_add_rosidl_dependency(${example_target})
    endif()
    list(APPEND PUBLIC_COMPAT_EXAMPLE_TARGETS ${example_target})
    set(PUBLIC_COMPAT_EXAMPLE_TARGETS "${PUBLIC_COMPAT_EXAMPLE_TARGETS}" PARENT_SCOPE)
  elseif("${link_mode}" STREQUAL "internal")
    if(ROKAE_BUILD_COMPAT_SDK)
      target_link_libraries(${example_target}
        xCoreSDK::xCoreSDK_static
        "${cpp_typesupport_target}"
      )
    else()
      target_link_libraries(${example_target}
        ${PROJECT_NAME}_sdk_backend
        "${cpp_typesupport_target}"
      )
    endif()
    ament_target_dependencies(${example_target}
      rclcpp
      rclcpp_action
      std_msgs
      sensor_msgs
      geometry_msgs
    )
    rokae_add_rosidl_dependency(${example_target})
    list(APPEND INTERNAL_BACKEND_EXAMPLE_TARGETS ${example_target})
    set(INTERNAL_BACKEND_EXAMPLE_TARGETS "${INTERNAL_BACKEND_EXAMPLE_TARGETS}" PARENT_SCOPE)
  else()
    message(FATAL_ERROR "Unknown example link mode: ${link_mode}")
  endif()

  list(APPEND EXAMPLE_TARGETS ${example_target})
  set(EXAMPLE_TARGETS "${EXAMPLE_TARGETS}" PARENT_SCOPE)
endfunction()

if(ROKAE_BUILD_PUBLIC_COMPAT_EXAMPLES)
  foreach(example IN LISTS ROKAE_PUBLIC_COMPAT_EXAMPLES)
    rokae_add_example(${example} public)
  endforeach()
endif()

if(ROKAE_BUILD_INTERNAL_BACKEND_EXAMPLES)
  foreach(example IN LISTS ROKAE_INTERNAL_BACKEND_EXAMPLES)
    rokae_add_example(${example} internal)
  endforeach()
endif()
