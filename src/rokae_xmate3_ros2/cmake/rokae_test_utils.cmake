# rokae_test_utils.cmake
# Extracted common test target configuration to eliminate CMakeLists.txt boilerplate.

# Standard unit test: gtest + runtime_test link + common include paths.
# Usage:
#   rokae_add_unit_test(test_my_module test/unit/test_my_module.cpp)
#
# For tests needing extra compile definitions, use the DEFINITIONS keyword:
#   rokae_add_unit_test(test_my_module test/unit/test_my_module.cpp
#     DEFINITIONS FOO="bar" BAZ="qux")
function(rokae_add_unit_test test_name test_source)
  cmake_parse_arguments(ROKAE_TEST "" "" "DEFINITIONS" ${ARGN})

  ament_add_gtest(${test_name} ${test_source})

  if(TARGET ${test_name})
    target_include_directories(${test_name}
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIRS}
    )
    target_link_libraries(${test_name}
      ${PROJECT_NAME}_runtime_test
    )
    target_compile_features(${test_name} PUBLIC cxx_std_17)
    rokae_add_rosidl_dependency(${test_name})

    if(ROKAE_TEST_DEFINITIONS)
      target_compile_definitions(${test_name} PRIVATE ${ROKAE_TEST_DEFINITIONS})
    endif()
  endif()
endfunction()
