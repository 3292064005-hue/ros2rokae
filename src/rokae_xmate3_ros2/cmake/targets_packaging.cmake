# Install runtime/backend artifacts used by ROS launch flows. These remain available for the
# ROS/Gazebo package itself, while the install-facing xCoreSDK package is kept on the public
# `rokae/*` ABI lane unless private exports are explicitly enabled.
if(ROKAE_BUILD_COMPAT_SDK)
  install(TARGETS
    ${PROJECT_NAME}_runtime_core
    ${PROJECT_NAME}_sdk_backend
    rokae_sim_runtime
    EXPORT xCoreSDKPrivateTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT internal_runtime
  )
else()
  install(TARGETS
    ${PROJECT_NAME}_runtime_core
    ${PROJECT_NAME}_sdk_backend
    rokae_sim_runtime
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT internal_runtime
  )
endif()

install(TARGETS
  xcore_controller_gazebo_plugin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  COMPONENT internal_runtime
)

install(TARGETS
  ${PUBLIC_COMPAT_EXAMPLE_TARGETS}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  COMPONENT public_sdk
)

if(ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES)
  install(TARGETS
    ${INTERNAL_BACKEND_EXAMPLE_TARGETS}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT internal_runtime
  )
endif()

if(ROKAE_BUILD_COMPAT_SDK)
  install(TARGETS
    xCoreSDK_shared
    xCoreSDK_static
    EXPORT xCoreSDKTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT public_sdk
  )
endif()

install(FILES
  include/rokae/base.h
  include/rokae/data_types.h
  include/rokae/error_category.hpp
  include/rokae/exception.h
  include/rokae/robot.h
  include/rokae/model.h
  include/rokae/motion_control_rt.h
  include/rokae/planner.h
  include/rokae/utility.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rokae
  COMPONENT public_sdk
)

if(ROKAE_INSTALL_BACKEND_DEV_HEADERS OR NOT ROKAE_STRICT_PUBLIC_INSTALL)
  install(DIRECTORY include/rokae_xmate3_ros2 DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT internal_devel)
endif()

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/docs/PUBLIC_SDK_ARTIFACT.md
  DESTINATION share/${PROJECT_NAME}/docs
  RENAME README.md
  COMPONENT public_sdk
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/clean_build_env.sh
  DESTINATION share/${PROJECT_NAME}/tools
  COMPONENT public_sdk
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/simulation.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/xmate6_public.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/rviz_only.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/_simulation_support.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/_launch_profile.py
  DESTINATION share/${PROJECT_NAME}/launch
  COMPONENT public_sdk
)
install(DIRECTORY meshes models urdf config worlds
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  COMPONENT public_sdk
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/render_robot_description.py
  DESTINATION share/${PROJECT_NAME}/tools
  COMPONENT public_sdk
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  COMPONENT internal_runtime
)

set(ROKAE_PUBLIC_COMPAT_EXAMPLE_SOURCE_FILES)
foreach(example_name IN LISTS ROKAE_PUBLIC_COMPAT_EXAMPLES)
  list(APPEND ROKAE_PUBLIC_COMPAT_EXAMPLE_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/cpp/${example_name}.cpp"
  )
endforeach()

set(ROKAE_INTERNAL_BACKEND_EXAMPLE_SOURCE_FILES)
foreach(example_name IN LISTS ROKAE_INTERNAL_BACKEND_EXAMPLES)
  list(APPEND ROKAE_INTERNAL_BACKEND_EXAMPLE_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/cpp/${example_name}.cpp"
  )
endforeach()

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/examples/PUBLIC_SDK_README.md"
  DESTINATION share/${PROJECT_NAME}/examples
  RENAME README.md
  COMPONENT public_sdk
)
if(ROKAE_PUBLIC_COMPAT_EXAMPLE_SOURCE_FILES)
  install(FILES ${ROKAE_PUBLIC_COMPAT_EXAMPLE_SOURCE_FILES}
    DESTINATION share/${PROJECT_NAME}/examples/cpp
    COMPONENT public_sdk
  )
endif()
if(ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES AND ROKAE_INTERNAL_BACKEND_EXAMPLE_SOURCE_FILES)
  install(FILES ${ROKAE_INTERNAL_BACKEND_EXAMPLE_SOURCE_FILES}
    DESTINATION share/${PROJECT_NAME}/examples/internal
    COMPONENT internal_runtime
  )
endif()
install(FILES model.config DESTINATION share/${PROJECT_NAME} COMPONENT internal_runtime)
install(FILES "${ROKAE_GENERATED_XMATE3_URDF}" "${ROKAE_GENERATED_XMATE3_URDF_METADATA}"
  DESTINATION share/${PROJECT_NAME}/generated/urdf
  COMPONENT public_sdk
)

if(ROKAE_BUILD_COMPAT_SDK)
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/xCoreSDKConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK
  )
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKConfigVersion.cmake
    VERSION 2.1.0
    COMPATIBILITY SameMinorVersion
  )
  install(EXPORT xCoreSDKTargets
    NAMESPACE xCoreSDK::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK
    COMPONENT public_sdk
  )
  if(ROKAE_EXPORT_PRIVATE_SDK_TARGETS)
    install(EXPORT xCoreSDKPrivateTargets
      FILE xCoreSDKPrivateTargets.cmake
      DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK
      COMPONENT internal_devel
    )
  endif()
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK
    COMPONENT public_sdk
  )
  # The symlink-based install override used by this package does not always
  # materialize FILE installs into staged prefixes (for example in install-tree
  # consumer tests using `cmake --install --prefix ...`). Mirror the config
  # installation through install(CODE) so staged prefixes always contain
  # find_package(xCoreSDK) entry points.
  install(CODE
    "file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK\")\n\
file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK\" TYPE FILE FILES \"${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKConfig.cmake\" \"${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKConfigVersion.cmake\")"
    COMPONENT public_sdk
  )
  # Likewise mirror public headers so install-tree consumers can resolve the
  # exported include directories even when symlink-install does not materialize
  # FILE installs into staged prefixes.
  install(CODE
    "file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}\")\n\
file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/include/rokae\")\n\
if(EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}\")\n\
endif()"
    COMPONENT public_sdk
  )
  # Mirror runtime share resources required by simulation smoke in staged install prefixes.
  install(CODE
    "file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")\n\
if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/models\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/models\")\n\
endif()\n\
if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/worlds\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/worlds\")\n\
endif()\n\
if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/urdf\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/urdf\")\n\
endif()\n\
if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/launch\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/launch\")\n\
endif()\n\
if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/config\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/config\")\n\
endif()\n\
if(EXISTS \"${CMAKE_CURRENT_SOURCE_DIR}/tools\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_SOURCE_DIR}/tools\")\n\
endif()\n\
if(EXISTS \"${CMAKE_CURRENT_BINARY_DIR}/generated/urdf\")\n\
  file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/generated\")\n\
  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/generated\" TYPE DIRECTORY FILES \"${CMAKE_CURRENT_BINARY_DIR}/generated/urdf\")\n\
endif()"
    COMPONENT internal_runtime
  )
endif()

ament_export_dependencies(
  rosidl_default_runtime
)
ament_export_include_directories(include)
