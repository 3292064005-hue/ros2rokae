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
if 'set(xCoreSDK_core_FOUND TRUE)' not in config:
    failures.append('xCoreSDKConfig.cmake.in must mark the core component as found for core-only install consumption')
if 'set(xCoreSDK_TARGET_FAMILY "xmate_er3")' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish xCoreSDK_TARGET_FAMILY as xmate_er3')
if 'set(xCoreSDK_UNSUPPORTED_MODULES' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish xCoreSDK_UNSUPPORTED_MODULES')
if 'set(xCoreSDK_INTERNAL_SURFACE_ENABLED FALSE)' not in config:
    failures.append('xCoreSDKConfig.cmake.in must declare xCoreSDK_INTERNAL_SURFACE_ENABLED FALSE for the public install artifact')
if 'set(xCoreSDK_PUBLIC_ROSIDL_ROOT "srv/")' not in config or 'set(xCoreSDK_INTERNAL_ROSIDL_ROOT "internal_interfaces/srv/")' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish public/internal ROSIDL roots')
if 'xCoreSDK_STATIC_PROVIDER "native-static"' not in config:
    failures.append('xCoreSDKConfig.cmake.in must publish the native static provider for the install-facing SDK')
if 'find_dependency(rclcpp REQUIRED CONFIG)' not in config or 'find_dependency(kdl_parser REQUIRED CONFIG)' not in config:
    failures.append('xCoreSDKConfig.cmake.in must keep ROS2/KDL dependency resolution available for explicit runtime/shared components')
if 'if(_xcoresdk_load_runtime)' not in config or 'set(xCoreSDK_shared_FOUND TRUE)' not in config:
    failures.append('xCoreSDKConfig.cmake.in must gate runtime/shared dependency loading behind explicit requested components')
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
install_tree_cmake = (install_tree / 'CMakeLists.txt').read_text(encoding='utf-8')
if 'xCoreSDK_PRIMARY_INSTALL_CONSUMER' not in install_tree_cmake:
    failures.append('install-tree consumer harness must consume xCoreSDK_PRIMARY_INSTALL_CONSUMER')
if 'if(xCoreSDK_PRIMARY_INSTALL_CONSUMER STREQUAL "cxx_sdk_core_consumer")' not in install_tree_cmake:
    failures.append('install-tree consumer harness must derive the primary install-facing target from xCoreSDK_PRIMARY_INSTALL_CONSUMER')
if 'set(XCORESDK_PRIMARY_INSTALL_LINK_TARGET xCoreSDK::xCoreSDK_core)' not in install_tree_cmake:
    failures.append('install-tree consumer harness must map cxx_sdk_core_consumer to xCoreSDK::xCoreSDK_core')
if 'set(XCORESDK_RUNTIME_LINK_TARGET xCoreSDK::xCoreSDK_shared)' not in install_tree_cmake:
    failures.append('install-tree consumer harness must keep the explicit runtime bridge target on xCoreSDK::xCoreSDK_shared')
if 'target_link_libraries(minimal_connect PRIVATE ${XCORESDK_RUNTIME_LINK_TARGET})' not in install_tree_cmake:
    failures.append('install-tree consumer harness must link runtime consumers through the explicit runtime bridge target')
if 'target_link_libraries(minimal_model PRIVATE ${XCORESDK_CORE_ONLY_LINK_TARGET})' not in install_tree_cmake or 'target_link_libraries(minimal_planner PRIVATE ${XCORESDK_CORE_ONLY_LINK_TARGET})' not in install_tree_cmake:
    failures.append('install-tree consumer harness must keep model/planner coverage on the core-only link target')
if 'find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)' not in (ROOT / 'test' / 'harness' / 'check_public_sdk_packaging_contract.py').read_text(encoding='utf-8'):
    failures.append('public SDK packaging contract must verify the core-only install consumer path with find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)')
runtime_integrity = (ROOT / 'test' / 'harness' / 'check_runtime_source_integrity.py').read_text(encoding='utf-8')
if 'compatibility_alias_policy.hpp failed standalone syntax-check' not in runtime_integrity:
    failures.append('runtime source integrity harness must syntax-check compatibility_alias_policy.hpp')
if 'service_contract_manifest.cpp has extra #endif' not in runtime_integrity:
    failures.append('runtime source integrity harness must guard service_contract_manifest.cpp preprocessor balance')
quickstart = (ROOT / 'docs' / 'public' / 'QUICKSTART.md').read_text(encoding='utf-8')
if 'install-facing 主消费者 `xCoreSDK::xCoreSDK_shared`' in quickstart:
    failures.append('docs/public/QUICKSTART.md must not describe xCoreSDK::xCoreSDK_shared as the install-facing primary consumer')
for required in [
    'minimal_state_and_motion.cpp',
    'minimal_static_link_only.cpp',
    'minimal_shared_link_only.cpp',
    'minimal_toolset_by_name.cpp',
    'minimal_flange_pos.cpp',
    'minimal_connect_noec_exception.cpp',
    'official_sdk_example_xmate_er3.cpp',
    'official_move_example_xmate_er3.cpp',
    'official_read_robot_state_xmate_er3.cpp',
    'official_path_record_xmate_er3.cpp',
]:
    if not (install_tree / required).is_file():
        failures.append(f'missing install-tree consumer coverage source: {required}')

# Split install-tree consumer matrix: keep pure core consumption separate from explicit runtime/ROS components.
core_split = ROOT / 'test' / 'compat' / 'install_tree_core_only'
runtime_split = ROOT / 'test' / 'compat' / 'install_tree_runtime_components'
if not (core_split / 'CMakeLists.txt').is_file() or not (core_split / 'minimal_core_only_consumer.cpp').is_file():
    failures.append('missing split core-only install-tree consumer sample')
else:
    core_split_cmake = (core_split / 'CMakeLists.txt').read_text(encoding='utf-8')
    if 'find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)' not in core_split_cmake:
        failures.append('split core-only consumer must request only the core component')
    if 'target_link_libraries(minimal_core_only_consumer PRIVATE xCoreSDK::xCoreSDK_shared)' in core_split_cmake or 'target_link_libraries(minimal_core_only_consumer PRIVATE xCoreSDK::xCoreSDK_ros_bridge)' in core_split_cmake:
        failures.append('split core-only consumer must not link runtime/ROS bridge targets')
if not (runtime_split / 'CMakeLists.txt').is_file() or not (runtime_split / 'minimal_runtime_components_consumer.cpp').is_file():
    failures.append('missing split runtime-components install-tree consumer sample')
else:
    runtime_split_cmake = (runtime_split / 'CMakeLists.txt').read_text(encoding='utf-8')
    if 'find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)' not in runtime_split_cmake:
        failures.append('split runtime-components consumer must explicitly request shared/static/ros_bridge')
    if 'xCoreSDK::rokae_xmate3_ros2_sdk_backend' not in runtime_split_cmake:
        failures.append('split runtime-components consumer must assert private backend targets do not leak')


public_model = ROOT / 'include' / 'rokae_xmate3_ros2' / 'model.hpp'
public_model_text = public_model.read_text(encoding='utf-8')
for forbidden_public_model_token in [
    'OwnedGazeboProvider',
    'GazeboProvider',
    'rokae_xmate3_ros2/runtime/kinematics_provider.hpp',
    'rokae_xmate3_ros2/gazebo/',
    'gazebo::',
    'gazebo_model::',
]:
    if forbidden_public_model_token in public_model_text:
        failures.append(f'model.hpp must not expose concrete simulation provider detail: {forbidden_public_model_token}')
for required_public_model_token in ['struct Impl', 'std::unique_ptr<Impl>', '~XMateModel()']:
    if required_public_model_token not in public_model_text:
        failures.append(f'model.hpp must hide concrete provider ownership behind an opaque implementation: {required_public_model_token}')
model_impl = ROOT / 'src' / 'sdk' / 'xmate_model.cpp'
if not model_impl.is_file():
    failures.append('missing compiled public model implementation source: src/sdk/xmate_model.cpp')
else:
    model_impl_text = model_impl.read_text(encoding='utf-8')
    if 'runtime/kinematics_provider.hpp' not in model_impl_text:
        failures.append('src/sdk/xmate_model.cpp must include the private concrete provider boundary from src/runtime')
    if 'OwnedGazeboProvider' not in model_impl_text or 'makeModelFacade(provider' not in model_impl_text:
        failures.append('src/sdk/xmate_model.cpp must own the simulation provider privately and feed the public provider facade')

model_facade_path = ROOT / 'include' / 'rokae_xmate3_ros2' / 'model_facade.hpp'
model_facade = model_facade_path.read_text(encoding='utf-8')
if 'using ModelFacade = gazebo_model::ModelFacade' in model_facade:
    failures.append('model_facade.hpp must not re-export gazebo_model::ModelFacade as the public ModelFacade')
if 'using ModelLoadContext = gazebo_model::LoadContext' in model_facade:
    failures.append('model_facade.hpp must not re-export Gazebo load context as the public model load context')
if 'class ModelFacade' not in model_facade or 'kinematics::Provider' not in model_facade:
    failures.append('model_facade.hpp must expose a provider-owned public wrapper')
for forbidden_model_token in [
    'rokae_xmate3_ros2/gazebo/model_facade.hpp',
    'rokae_xmate3_ros2/runtime/kinematics_provider.hpp',
    'gazebo_model::',
    'OwnedGazeboProvider',
    'GazeboProvider',
    'gazebo::',
    'delegate_',
]:
    if forbidden_model_token in model_facade:
        failures.append(f'model_facade.hpp must not expose concrete simulation provider detail: {forbidden_model_token}')
if 'rokae_xmate3_ros2/runtime/kinematics_provider_interface.hpp' not in model_facade:
    failures.append('model_facade.hpp must include only the backend-neutral kinematics provider interface')


gazebo_model_facade_path = ROOT / 'include' / 'rokae_xmate3_ros2' / 'gazebo' / 'model_facade.hpp'
if not gazebo_model_facade_path.is_file():
    failures.append('missing Gazebo model facade header')
else:
    gazebo_model_facade_text = gazebo_model_facade_path.read_text(encoding='utf-8')
    if 'rokae_xmate3_ros2/runtime/kinematics_provider.hpp' in gazebo_model_facade_text:
        failures.append('gazebo/model_facade.hpp must not include the removed public concrete provider path')
    if 'rokae_xmate3_ros2/runtime/kinematics_provider_interface.hpp' not in gazebo_model_facade_text:
        failures.append('gazebo/model_facade.hpp must include the backend-neutral provider interface')

# No C++ source/header may keep including the removed public concrete provider path.
removed_public_provider = 'rokae_xmate3_ros2/runtime/kinematics_provider.hpp'
for source_root in ['include', 'src', 'test']:
    root_path = ROOT / source_root
    if not root_path.exists():
        continue
    for candidate in root_path.rglob('*'):
        if not candidate.is_file() or candidate.suffix not in {'.h', '.hpp', '.hh', '.cpp', '.cc', '.cxx'}:
            continue
        try:
            candidate_text = candidate.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            continue
        if removed_public_provider in candidate_text:
            failures.append(f'removed public concrete provider include remains in {candidate.relative_to(ROOT)}')


public_concrete_provider = ROOT / 'include' / 'rokae_xmate3_ros2' / 'runtime' / 'kinematics_provider.hpp'
if public_concrete_provider.exists():
    failures.append('concrete simulation provider header must not be present under public include/rokae_xmate3_ros2/runtime')
private_concrete_provider = ROOT / 'src' / 'runtime' / 'kinematics_provider.hpp'
if not private_concrete_provider.is_file():
    failures.append('missing private concrete provider boundary: src/runtime/kinematics_provider.hpp')
else:
    private_provider_text = private_concrete_provider.read_text(encoding='utf-8')
    if 'rokae_xmate3_ros2/gazebo/kinematics.hpp' not in private_provider_text or 'OwnedGazeboProvider' not in private_provider_text:
        failures.append('private concrete provider boundary must contain the Gazebo-backed implementation details')

provider_interface = ROOT / 'include' / 'rokae_xmate3_ros2' / 'runtime' / 'kinematics_provider_interface.hpp'
if not provider_interface.is_file():
    failures.append('missing backend-neutral kinematics_provider_interface.hpp')
else:
    provider_interface_text = provider_interface.read_text(encoding='utf-8')
    if 'class Provider' not in provider_interface_text:
        failures.append('kinematics_provider_interface.hpp must declare the Provider abstraction')
    for forbidden_provider_interface_token in ['rokae_xmate3_ros2/gazebo/', 'OwnedGazeboProvider', 'GazeboProvider', 'gazebo::', 'gazebo_model::']:
        if forbidden_provider_interface_token in provider_interface_text:
            failures.append(f'kinematics_provider_interface.hpp must not expose concrete simulation provider detail: {forbidden_provider_interface_token}')

full_gate = ROOT / 'tools' / 'run_full_source_tree_build_gate.sh'
if not full_gate.is_file():
    failures.append('missing tools/run_full_source_tree_build_gate.sh for non-replay full source-tree validation')
else:
    full_gate_text = full_gate.read_text(encoding='utf-8')
    if '-DROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF' not in full_gate_text:
        failures.append('full source-tree build gate must configure with replay-only OFF')
    if 'colcon build' not in full_gate_text or 'check_target_environment.sh' not in full_gate_text:
        failures.append('full source-tree build gate must run through the locked ROS2/Gazebo build environment')

report_verifier = ROOT / 'tools' / 'verify_target_env_acceptance_report.py'
if not report_verifier.is_file():
    failures.append('missing target environment acceptance report verifier')
else:
    verifier_text = report_verifier.read_text(encoding='utf-8')
    for verifier_token in ['full_source_gate', 'environment_check', 'rosdep_install', 'quick_gate', 'success', 'require_evidence_logs', 'require_target_environment', 'FULL_SOURCE_SUCCESS_MARKER']:
        if verifier_token not in verifier_text:
            failures.append(f'target acceptance report verifier missing required token: {verifier_token}')

report_contract = ROOT / 'test' / 'harness' / 'check_target_env_acceptance_report_contract.py'
if not report_contract.is_file():
    failures.append('missing target environment acceptance report contract harness')

acceptance_script = ROOT / 'tools' / 'run_target_env_acceptance.sh'
if acceptance_script.is_file():
    acceptance_text = acceptance_script.read_text(encoding='utf-8')
    if 'verify_target_env_acceptance_report.py' not in acceptance_text:
        failures.append('target environment acceptance must verify success reports before returning success')
    if 'full_source_gate_local.log' not in acceptance_text or 'full_source_gate_container.log' not in acceptance_text:
        failures.append('target environment acceptance must capture dedicated full source gate evidence logs')
else:
    failures.append('missing tools/run_target_env_acceptance.sh')

if failures:
    print("compat public ABI check failed:")
    for item in failures:
        print(f"- {item}")
    sys.exit(1)
print("compat public ABI check passed")

