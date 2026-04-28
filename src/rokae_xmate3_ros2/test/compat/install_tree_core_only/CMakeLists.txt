cmake_minimum_required(VERSION 3.16)
project(xcore_sdk_install_tree_core_only_consumer LANGUAGES CXX)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)

if(NOT DEFINED xCoreSDK_PRIMARY_INSTALL_CONSUMER OR NOT xCoreSDK_PRIMARY_INSTALL_CONSUMER STREQUAL "cxx_sdk_core_consumer")
  message(FATAL_ERROR "core-only consumer requires xCoreSDK_PRIMARY_INSTALL_CONSUMER=cxx_sdk_core_consumer")
endif()
if(NOT TARGET xCoreSDK::xCoreSDK_core)
  message(FATAL_ERROR "xCoreSDK::xCoreSDK_core target is missing from core-only install consumption")
endif()
if(TARGET xCoreSDK::xCoreSDK_shared OR TARGET xCoreSDK::xCoreSDK_ros_bridge)
  message(FATAL_ERROR "core-only install consumption must not resolve runtime/ROS bridge targets")
endif()

add_executable(minimal_core_only_consumer minimal_core_only_consumer.cpp)
target_link_libraries(minimal_core_only_consumer PRIVATE xCoreSDK::xCoreSDK_core)
target_compile_features(minimal_core_only_consumer PRIVATE cxx_std_17)
