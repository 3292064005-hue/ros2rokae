cmake_minimum_required(VERSION 3.16)
project(xcore_sdk_install_tree_consumer LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(xCoreSDK CONFIG REQUIRED)

if(NOT TARGET xCoreSDK::xCoreSDK_shared)
  message(FATAL_ERROR "xCoreSDK::xCoreSDK_shared target is missing from the install package")
endif()
if(NOT TARGET xCoreSDK::xCoreSDK_static)
  message(FATAL_ERROR "xCoreSDK::xCoreSDK_static target is missing from the install package")
endif()
if(NOT DEFINED xCoreSDK_STRICT_PUBLIC_INSTALL)
  message(FATAL_ERROR "xCoreSDK_STRICT_PUBLIC_INSTALL must be exported by the install-facing package")
endif()
if(NOT xCoreSDK_HAS_NATIVE_STATIC_TARGET)
  message(FATAL_ERROR "xCoreSDK::xCoreSDK_static must be a native static target in the install-facing package")
endif()
if(NOT DEFINED xCoreSDK_TARGET_FAMILY OR NOT xCoreSDK_TARGET_FAMILY STREQUAL "xmate6")
  message(FATAL_ERROR "xCoreSDK_TARGET_FAMILY must be exported as xmate6")
endif()
if(NOT DEFINED xCoreSDK_XMATE6_ONLY OR NOT xCoreSDK_XMATE6_ONLY)
  message(FATAL_ERROR "xCoreSDK_XMATE6_ONLY must be TRUE for the install-facing package")
endif()
if(TARGET xCoreSDK::rokae_xmate3_ros2_sdk_backend)
  message(FATAL_ERROR "private backend target must not leak into the install-facing package")
endif()
if(NOT xCoreSDK_STATIC_PROVIDER STREQUAL "native-static")
  message(FATAL_ERROR "xCoreSDK_STATIC_PROVIDER must report native-static")
endif()


get_target_property(_xcore_static_includes xCoreSDK::xCoreSDK_static INTERFACE_INCLUDE_DIRECTORIES)
if(NOT _xcore_static_includes)
  message(FATAL_ERROR "xCoreSDK::xCoreSDK_static must publish install-tree include directories")
endif()
foreach(_inc IN LISTS _xcore_static_includes)
  if(_inc MATCHES "BUILD_INTERFACE" OR _inc MATCHES "CMAKE_CURRENT_SOURCE_DIR" OR _inc MATCHES "CMAKE_CURRENT_BINARY_DIR")
    message(FATAL_ERROR "install-facing static target leaked a build/source include path: ${_inc}")
  endif()
endforeach()

add_executable(minimal_connect minimal_connect.cpp)
target_link_libraries(minimal_connect PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_connect PRIVATE cxx_std_17)

add_executable(minimal_connect_overload minimal_connect_overload.cpp)
target_link_libraries(minimal_connect_overload PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_connect_overload PRIVATE cxx_std_17)

add_executable(minimal_model minimal_model.cpp)
target_link_libraries(minimal_model PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_model PRIVATE cxx_std_17)

add_executable(minimal_robot_t minimal_robot_t.cpp)
target_link_libraries(minimal_robot_t PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_robot_t PRIVATE cxx_std_17)

add_executable(minimal_soft_limit minimal_soft_limit.cpp)
target_link_libraries(minimal_soft_limit PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_soft_limit PRIVATE cxx_std_17)

add_executable(minimal_execute_movec minimal_execute_movec.cpp)
target_link_libraries(minimal_execute_movec PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_execute_movec PRIVATE cxx_std_17)

add_executable(minimal_toolset_by_name minimal_toolset_by_name.cpp)
target_link_libraries(minimal_toolset_by_name PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_toolset_by_name PRIVATE cxx_std_17)

add_executable(minimal_flange_pos minimal_flange_pos.cpp)
target_link_libraries(minimal_flange_pos PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_flange_pos PRIVATE cxx_std_17)

add_executable(minimal_connect_noec_exception minimal_connect_noec_exception.cpp)
target_link_libraries(minimal_connect_noec_exception PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_connect_noec_exception PRIVATE cxx_std_17)

add_executable(minimal_rt_header minimal_rt_header.cpp)
target_link_libraries(minimal_rt_header PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_rt_header PRIVATE cxx_std_17)

add_executable(minimal_planner minimal_planner.cpp)
target_link_libraries(minimal_planner PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_planner PRIVATE cxx_std_17)

add_executable(minimal_state_and_motion minimal_state_and_motion.cpp)
target_link_libraries(minimal_state_and_motion PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_state_and_motion PRIVATE cxx_std_17)


add_executable(minimal_static_link_only minimal_static_link_only.cpp)
target_link_libraries(minimal_static_link_only PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(minimal_static_link_only PRIVATE cxx_std_17)

add_executable(minimal_shared_link_only minimal_shared_link_only.cpp)
target_link_libraries(minimal_shared_link_only PRIVATE xCoreSDK::xCoreSDK_shared)
target_compile_features(minimal_shared_link_only PRIVATE cxx_std_17)

# Official-SDK-style xMate6 subset examples (compile/link contract only).
add_executable(official_sdk_example_xmate6 official_sdk_example_xmate6.cpp)
target_link_libraries(official_sdk_example_xmate6 PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(official_sdk_example_xmate6 PRIVATE cxx_std_17)

add_executable(official_move_example_xmate6 official_move_example_xmate6.cpp)
target_link_libraries(official_move_example_xmate6 PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(official_move_example_xmate6 PRIVATE cxx_std_17)

add_executable(official_read_robot_state_xmate6 official_read_robot_state_xmate6.cpp)
target_link_libraries(official_read_robot_state_xmate6 PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(official_read_robot_state_xmate6 PRIVATE cxx_std_17)

add_executable(official_path_record_xmate6 official_path_record_xmate6.cpp)
target_link_libraries(official_path_record_xmate6 PRIVATE xCoreSDK::xCoreSDK_static)
target_compile_features(official_path_record_xmate6 PRIVATE cxx_std_17)
