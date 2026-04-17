#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
public_headers = [
    ROOT / "include" / "rokae" / "robot.h",
    ROOT / "include" / "rokae" / "model.h",
    ROOT / "include" / "rokae" / "motion_control_rt.h",
    ROOT / "include" / "rokae" / "planner.h",
    ROOT / "include" / "rokae" / "data_types.h",
    ROOT / "include" / "rokae" / "utility.h",
]
forbidden = ["rokae_xmate3_ros2/", "sdk_shim", "rclcpp", "gazebo", "rosidl"]
failures = []
for path in public_headers:
    if not path.exists():
        failures.append(f"missing public header: {path}")
        continue
    text = path.read_text(encoding="utf-8")
    for needle in forbidden:
        if needle in text:
            failures.append(f"forbidden token '{needle}' found in {path}")

robot_header = (ROOT / "include" / "rokae" / "robot.h").read_text(encoding="utf-8")
if "Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept;" not in robot_header:
    failures.append("rokae/robot.h must expose Toolset-returning setToolset(toolName, wobjName, ec)")
if "std::array<double, 6> flangePos(error_code &ec) const noexcept;" not in robot_header:
    failures.append("rokae/robot.h must keep deprecated flangePos(ec) compatibility alias")
if "[[deprecated(\"Use jointTorque() instead\")]]" not in robot_header:
    failures.append("rokae/robot.h must mark jointTorques() as deprecated compatibility alias")

cmake_root = ROOT / 'CMakeLists.txt'
cmake_text = cmake_root.read_text(encoding='utf-8')
if 'ROKAE_STRICT_PUBLIC_INSTALL' not in cmake_text:
    failures.append('CMakeLists.txt must define ROKAE_STRICT_PUBLIC_INSTALL for install-facing ABI hardening')
if 'ROKAE_EXPORT_PRIVATE_SDK_TARGETS' not in cmake_text:
    failures.append('CMakeLists.txt must define ROKAE_EXPORT_PRIVATE_SDK_TARGETS to gate native/private exports')


compat_sdk = (ROOT / 'cmake' / 'targets_sdk_compat.cmake').read_text(encoding='utf-8')
if '$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>' not in compat_sdk:
    failures.append('targets_sdk_compat.cmake must use BUILD_INTERFACE include paths for install-safe exports')
if '$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>' not in compat_sdk:
    failures.append('targets_sdk_compat.cmake must use INSTALL_INTERFACE include paths for install-safe exports')
if 'src/runtime/rt_command_bridge.cpp' not in compat_sdk:
    failures.append('targets_sdk_compat.cmake must compile the unified RT command bridge into the install-facing SDK targets')

packaging = (ROOT / 'cmake' / 'targets_packaging.cmake').read_text(encoding='utf-8')
if 'ROKAE_INSTALL_BACKEND_DEV_HEADERS OR NOT ROKAE_STRICT_PUBLIC_INSTALL' not in packaging:
    failures.append('targets_packaging.cmake must gate backend header installation behind the strict public install policy')
if 'if(ROKAE_EXPORT_PRIVATE_SDK_TARGETS)' not in packaging:
    failures.append('targets_packaging.cmake must gate private target export behind ROKAE_EXPORT_PRIVATE_SDK_TARGETS')
if 'INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}' not in packaging:
    failures.append('targets_packaging.cmake must export install-safe include destinations for install-facing SDK targets')

config = (ROOT / 'cmake' / 'xCoreSDKConfig.cmake.in').read_text(encoding='utf-8')
if 'set(xCoreSDK_STRICT_PUBLIC_INSTALL TRUE)' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish strict public install mode by default')
if 'set(xCoreSDK_TARGET_FAMILY "xmate6")' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish xCoreSDK_TARGET_FAMILY as xmate6')
if 'set(xCoreSDK_UNSUPPORTED_MODULES' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish xCoreSDK_UNSUPPORTED_MODULES')
if 'xCoreSDK_STATIC_PROVIDER "native-static"' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish the native static provider for the install-facing SDK')
if 'find_dependency(rclcpp REQUIRED CONFIG)' not in config or 'find_dependency(kdl_parser REQUIRED CONFIG)' not in config:
    failures.append('xCoreSDKConfig.cmake.in must resolve the install-facing ROS2/KDL dependency set before loading exported targets')
if 'find_dependency(gazebo_ros REQUIRED CONFIG)' in config:
    failures.append('xCoreSDKConfig.cmake.in must not require gazebo_ros for the install-facing public SDK targets')

compat_targets = ROOT / 'cmake' / 'targets_examples.cmake'
if compat_targets.exists():
    text = compat_targets.read_text(encoding='utf-8')
    if 'ROKAE_PUBLIC_COMPAT_EXAMPLES' not in text:
        failures.append('missing public compat example grouping in cmake/targets_examples.cmake')
    if 'ROKAE_INTERNAL_BACKEND_EXAMPLES' not in text:
        failures.append('missing internal backend example grouping in cmake/targets_examples.cmake')

install_tree = ROOT / 'test' / 'compat' / 'install_tree'
for required in [
    'minimal_state_and_motion.cpp',
    'minimal_static_link_only.cpp',
    'minimal_shared_link_only.cpp',
    'minimal_toolset_by_name.cpp',
    'minimal_flange_pos.cpp',
    'minimal_connect_noec_exception.cpp',
    'official_sdk_example_xmate6.cpp',
    'official_move_example_xmate6.cpp',
    'official_read_robot_state_xmate6.cpp',
    'official_path_record_xmate6.cpp',
]:
    if not (install_tree / required).is_file():
        failures.append(f'missing install-tree consumer coverage source: {required}')

if failures:
    print("compat public ABI check failed:")
    for item in failures:
        print(f"- {item}")
    sys.exit(1)
print("compat public ABI check passed")
