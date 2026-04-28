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
    xCoreSDK_core
    EXPORT xCoreSDKCoreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT public_sdk
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT public_sdk
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT public_sdk
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(TARGETS
    xCoreSDK_shared
    xCoreSDK_static
    xCoreSDK_ros_bridge
    EXPORT xCoreSDKTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT public_sdk
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT public_sdk
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT public_sdk
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
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
  ${CMAKE_CURRENT_SOURCE_DIR}/docs/public/PUBLIC_SDK_ARTIFACT.md
  DESTINATION share/${PROJECT_NAME}/docs
  RENAME README.md
  COMPONENT public_sdk
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/docs/INDEX.md
  DESTINATION share/${PROJECT_NAME}/docs
  COMPONENT public_sdk
)
install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/docs/public
  ${CMAKE_CURRENT_SOURCE_DIR}/docs/reference
  ${CMAKE_CURRENT_SOURCE_DIR}/docs/release
  DESTINATION share/${PROJECT_NAME}/docs
  COMPONENT public_sdk
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/clean_build_env.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/acceptance_cli_common.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_target_environment.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_runtime_diag_gate.py
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/derive_runtime_diag_gate.py
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/render_robot_description.py
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_acceptance_layers.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_full_task_acceptance.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_launch_smoke.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_loaded_sensor_acceptance.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_main_chain_smoke.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_quick_gate.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_real_dryrun_acceptance.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_release_gate.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_release_gate_portable.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_target_env_acceptance.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_xmate_er3_alignment_behavior_gate.sh
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/write_release_scoreboard.py
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/write_target_env_report.py
  DESTINATION share/${PROJECT_NAME}/tools
  COMPONENT public_sdk
)
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/simulation.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/xmate_er3_public.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/rviz_only.launch.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/_simulation_support.py
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/_launch_profile.py
  DESTINATION share/${PROJECT_NAME}/launch
  COMPONENT public_sdk
)
install(DIRECTORY meshes models urdf config worlds
  DESTINATION share/${PROJECT_NAME}/
  COMPONENT public_sdk
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(ROKAE_ENABLE_INTERNAL_SURFACE)
  install(DIRECTORY internal_interfaces/ DESTINATION share/${PROJECT_NAME}/internal_interfaces COMPONENT internal_runtime)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  COMPONENT internal_runtime
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
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
    "${CMAKE_CURRENT_SOURCE_DIR}/examples/internal/cpp/${example_name}.cpp"
  )
endforeach()

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/examples/README.md"
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
install(FILES
  "${ROKAE_GENERATED_XMATE_ER3_URDF}"
  "${ROKAE_GENERATED_XMATE_ER3_URDF_METADATA}"
  "${ROKAE_GENERATED_XMATE3_URDF}"
  "${ROKAE_GENERATED_XMATE3_URDF_METADATA}"
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
  install(EXPORT xCoreSDKCoreTargets
    FILE xCoreSDKCoreTargets.cmake
    NAMESPACE xCoreSDK::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK
    COMPONENT public_sdk
  )
  install(EXPORT xCoreSDKTargets
    FILE xCoreSDKTargets.cmake
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
    ${CMAKE_CURRENT_BINARY_DIR}/xCoreSDKInstallMetadata.json
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xCoreSDK
    COMPONENT public_sdk
  )
  # The symlink-based install override used by this package does not always
  # materialize FILE installs into staged prefixes (for example in install-tree
  # consumer tests using `cmake --install --prefix ...`). Mirror the config
  # installation through install(CODE) so staged prefixes always contain
  # find_package(xCoreSDK) entry points.
  # Likewise mirror only the public rokae headers so install-tree consumers can resolve the
  # exported include directories even when symlink-install does not materialize
  # FILE installs into staged prefixes. Generated rosidl headers stay out of the
  # public SDK include tree because they would re-expose internal service/message
  # surface area that is intentionally outside the xMateER3 public contract.
  # Mirror runtime share resources required by simulation smoke in staged install prefixes.
endif()

ament_export_dependencies(
  rosidl_default_runtime
)
ament_export_include_directories(include)
