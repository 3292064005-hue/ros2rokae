#!/usr/bin/env python3
import math
import pathlib
import re
import sys

sys.dont_write_bytecode = True

ROOT = pathlib.Path(__file__).resolve().parents[2]
FAILURES = []


def _parse_spec_array(text: str, name: str) -> list[float]:
    pattern = re.compile(rf"{re.escape(name)}\s*=\s*\{{([^}}]+)\}};", re.MULTILINE | re.DOTALL)
    match = pattern.search(text)
    if not match:
        raise AssertionError(f"unable to locate spec array {name}")
    values = []
    for token in match.group(1).replace("\n", " ").split(','):
        token = token.strip()
        if not token:
            continue
        values.append(float(token))
    return values


def _parse_spec_nested_array(text: str, name: str) -> list[list[float]]:
    pattern = re.compile(rf"{re.escape(name)}\s*=\s*\{{\{{(.*?)\}}\}};", re.MULTILINE | re.DOTALL)
    match = pattern.search(text)
    if not match:
        raise AssertionError(f"unable to locate spec nested array {name}")
    values = []
    for triple in re.findall(r"\{\{([^}]*)\}\}", match.group(1)):
        row = [float(token.strip()) for token in triple.replace("\n", " ").split(',') if token.strip()]
        values.append(row)
    return values


def _parse_spec_string_array(text: str, name: str) -> list[str]:
    pattern = re.compile(rf"{re.escape(name)}\s*=\s*\{{([^}}]+)\}};", re.MULTILINE | re.DOTALL)
    match = pattern.search(text)
    if not match:
        raise AssertionError(f"unable to locate spec string array {name}")
    return re.findall(r'"([^"]+)"', match.group(1))


def _approx_equal(a: float, b: float, tol: float = 1e-6) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)


def _parse_xacro_joint_invocations(text: str) -> list[dict[str, str]]:
    joint_blocks = []
    pattern = re.compile(r'<xacro:robot_joint\s+(.*?)\/>', re.DOTALL)
    attr_pattern = re.compile(r'(\w+)="([^"]+)"')
    for block in pattern.findall(text):
        attrs = {key: value for key, value in attr_pattern.findall(block)}
        if attrs:
            joint_blocks.append(attrs)
    return joint_blocks


for path in ROOT.rglob('*'):
    rel = path.relative_to(ROOT)
    rel_str = str(rel)
    if '__pycache__' in rel.parts:
        FAILURES.append(f'compiled cache should not be committed: {rel_str}')
    if path.is_file() and (rel.name.startswith('REWRITE_') or rel.name.endswith('REWRITE_SUMMARY.md') or rel.name == 'CHATGPT_REWRITE_SUMMARY.md'):
        FAILURES.append(f'rewrite artifact should not be shipped: {rel_str}')

launch_support = (ROOT / 'launch' / '_simulation_support.py').read_text(encoding='utf-8')
if '/usr/share/gazebo-11' in launch_support or '/usr/share/gazebo' in launch_support:
    FAILURES.append('launch/_simulation_support.py still contains hard-coded gazebo resource fallbacks')

clean_env = (ROOT / 'tools' / 'clean_build_env.sh').read_text(encoding='utf-8')
if '/usr/bin/python3' in clean_env:
    FAILURES.append('tools/clean_build_env.sh still contains a hard-coded /usr/bin/python3 fallback')

targets_tests = (ROOT / 'cmake' / 'targets_tests.cmake').read_text(encoding='utf-8')
if '/usr/bin/python3' in targets_tests:
    FAILURES.append('cmake/targets_tests.cmake still contains a hard-coded /usr/bin/python3 fallback')

env_lock = ROOT / 'docs' / 'ENVIRONMENT_LOCK.md'
if not env_lock.is_file():
    FAILURES.append('docs/ENVIRONMENT_LOCK.md is required to pin the supported target environment')

acceptance_script = ROOT / 'tools' / 'run_target_env_acceptance.sh'
if not acceptance_script.is_file():
    FAILURES.append('tools/run_target_env_acceptance.sh is required to execute the locked target-environment acceptance bundle')

launch_smoke_script = ROOT / 'tools' / 'run_launch_smoke.sh'
if not launch_smoke_script.is_file():
    FAILURES.append('tools/run_launch_smoke.sh is required to validate installed-package launch discovery inside the locked target-environment bundle')

acceptance_workflow = ROOT / '.github' / 'workflows' / 'acceptance-humble-gazebo11.yml'
if not acceptance_workflow.is_file():
    FAILURES.append('.github/workflows/acceptance-humble-gazebo11.yml is required for locked target-environment acceptance automation')

acceptance_text = acceptance_script.read_text(encoding='utf-8')
if 'PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"' not in acceptance_text:
    FAILURES.append('run_target_env_acceptance.sh must resolve PKG_ROOT before using local-target-env bundle')
local_branch = acceptance_text.find('if [ "${RUN_LOCAL_TARGET_ENV}" = "1" ]; then')
pkg_root_assignment = acceptance_text.find('PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"')
if local_branch != -1 and pkg_root_assignment != -1 and pkg_root_assignment > local_branch:
    FAILURES.append('run_target_env_acceptance.sh resolves PKG_ROOT after the --local-target-env branch and will fail with set -u')
if 'always emitted into --report-dir' not in acceptance_text or 'emit_report_host' not in acceptance_text:
    FAILURES.append('run_target_env_acceptance.sh must always emit a machine-readable report, including failed acceptance runs')
if 'environment_check_local.log' not in acceptance_text or 'quick_gate_local.log' not in acceptance_text:
    FAILURES.append('run_target_env_acceptance.sh must preserve per-stage acceptance logs for local runs')
if 'acceptance_report_container.json' not in acceptance_text or 'container_bundle_failed_before_report' not in acceptance_text:
    FAILURES.append('run_target_env_acceptance.sh must produce a fallback report when the container bundle fails before writing one')
write_report_text = (ROOT / 'tools' / 'write_target_env_report.py').read_text(encoding='utf-8')
for token in ['--environment-check-status', '--rosdep-install-status', '--quick-gate-status', '--release-gate-status', '--launch-smoke-status', '--image-build-status', "'status': {", "'success': acceptance_status == 'passed'"]:
    if token not in write_report_text:
        FAILURES.append(f'tools/write_target_env_report.py must capture acceptance stage status token: {token}')
env_check_text = (ROOT / 'tools' / 'check_target_environment.sh').read_text(encoding='utf-8')
for required_cmd in ['colcon', 'rosdep', 'ros2', 'xacro']:
    if f"for cmd in colcon rosdep ros2 xacro gazebo; do" not in env_check_text:
        FAILURES.append('tools/check_target_environment.sh must validate colcon, rosdep, ros2, xacro, and gazebo for acceptance paths')
        break

dockerfile = ROOT / 'docker' / 'Dockerfile.humble-gazebo11'
if not dockerfile.is_file():
    FAILURES.append('docker/Dockerfile.humble-gazebo11 is required to provide the locked target-environment baseline')

sdk_shim_core = (ROOT / 'include' / 'rokae' / 'detail' / 'sdk_shim_core.hpp').read_text(encoding='utf-8')
if 'options.catalog_policy = ::rokae::ros2::strictRuntimeCatalogPolicy();' not in sdk_shim_core:
    FAILURES.append('sdk shim no longer defaults compatibility wrappers to strict runtime authority')
if 'options.catalog_policy = ::rokae::ros2::legacySdkCompatibilityCatalogPolicy();' in sdk_shim_core:
    FAILURES.append('sdk shim still hard-codes legacy catalog fallback as the default wrapper policy')
if 'normalize_compatibility_client_options' not in sdk_shim_core:
    FAILURES.append('sdk shim no longer normalizes RosClientOptions to preserve strict runtime-authority defaults')

rt_registry = (ROOT / 'src' / 'runtime' / 'rt_field_registry.cpp').read_text(encoding='utf-8')
if 'synthetic_placeholder' in rt_registry or 'RtSupportedFields::elbow_m' in rt_registry:
    FAILURES.append('xMate3 six-axis RT registry still exposes synthetic elbow/psi placeholders')

rt_cpp = (ROOT / 'src' / 'sdk' / 'robot_rt.cpp').read_text(encoding='utf-8')
if 'RtSupportedFields::elbow_m' in rt_cpp:
    FAILURES.append('native SDK RT cache still handles unsupported elbow/psi placeholder fields')

example_state_stream = (ROOT / 'examples' / 'cpp' / '17_state_stream_cache.cpp').read_text(encoding='utf-8')
if 'RtSupportedFields::elbow_m' in example_state_stream or 'psi_m' in example_state_stream:
    FAILURES.append('state stream example still requests unsupported elbow/psi placeholders on xMate3 six-axis')

robot_internal_sdk = (ROOT / 'src' / 'sdk' / 'robot_internal.hpp').read_text(encoding='utf-8')
if 'class ScopedLastError final' not in robot_internal_sdk or 'remember_last_error' not in robot_internal_sdk:
    FAILURES.append('native SDK facade no longer provides unified lastErrorCode tracking scope for error_code entrypoints')

ros_context_owner = (ROOT / 'include' / 'rokae_xmate3_ros2' / 'runtime' / 'ros_context_owner.hpp').read_text(encoding='utf-8')
if 'not call global rclcpp::shutdown()' not in ros_context_owner:
    FAILURES.append('ros_context_owner contract no longer documents wrapper shutdown ownership explicitly')

runtime_bootstrap = (ROOT / 'src' / 'gazebo' / 'runtime_bootstrap.cpp').read_text(encoding='utf-8')
if '/xmate3/internal/prepare_shutdown' not in runtime_bootstrap:
    FAILURES.append('runtime_bootstrap.cpp no longer exposes the prepare_shutdown contract service')
if 'prepare_shutdown contract accepted=' not in runtime_bootstrap:
    FAILURES.append('runtime_bootstrap.cpp no longer logs the prepare_shutdown contract summary')

sdk_sources = (ROOT / 'src' / 'sdk').glob('*.cpp')
for source in sdk_sources:
    source_text = source.read_text(encoding='utf-8')
    if 'rclcpp::shutdown(' in source_text:
        FAILURES.append(f'sdk source must not call global rclcpp::shutdown(): {source.relative_to(ROOT)}')

robot_clients = (ROOT / 'src' / 'sdk' / 'robot_clients.cpp').read_text(encoding='utf-8')
if 'catalog_policy_ = strictRuntimeCatalogPolicy();' not in robot_clients:
    FAILURES.append('native ROS facade no longer initializes strict runtime catalog policy by default')

robot_rt = (ROOT / 'src' / 'sdk' / 'robot_rt.cpp').read_text(encoding='utf-8')
if 'fetch_joint_state_fallback' in robot_rt:
    FAILURES.append('robot_rt.cpp still contains the legacy RT->NRT joint-state fallback path')
if 'strict_rejected;' not in robot_rt:
    FAILURES.append('robot_rt.cpp no longer records strict RT subscription-plan rejection semantics')
if 'xmate3_robot_get_end_effector_torque_client_' in robot_rt:
    FAILURES.append('robot_rt.cpp references a non-existent end-torque client member and will not compile')
if 'requires_pose_fields' not in robot_rt or 'requires_end_torque_fields' not in robot_rt:
    FAILURES.append('robot_rt.cpp no longer guards supplemental RT state fetches by the requested field set')

readme = (ROOT / 'README.md').read_text(encoding='utf-8')
if '│   ├── xMate3.urdf' in readme or '├── generated/' in readme:
    FAILURES.append('README.md still claims committed source-tree derived URDF artifacts')
if '<build>/generated/urdf/xMate3.urdf' not in readme:
    FAILURES.append('README.md must describe the build-generated URDF path explicitly')
if 'xMate3.description.json' not in readme:
    FAILURES.append('README.md must document the generated description metadata artifact')
if '/xmate3/internal/get_runtime_state_snapshot' not in readme:
    FAILURES.append('README.md must document the aggregated runtime snapshot query surface')
if 'service_contract_manifest.hpp' not in readme:
    FAILURES.append('README.md must document the single-source service contract manifest')

manifest = (ROOT / 'src' / 'runtime' / 'service_contract_manifest.hpp').read_text(encoding='utf-8')
if 'ROKAE_PRIMARY_SERVICE_CONTRACTS' not in manifest or 'ROKAE_COMPATIBILITY_ALIAS_CONTRACTS' not in manifest:
    FAILURES.append('service contract manifest must centralize primary and compatibility descriptors')
if '/xmate3/internal/get_runtime_state_snapshot' not in manifest:
    FAILURES.append('service contract manifest must include the aggregated runtime snapshot query surface')

for launch_file in ['launch/xmate3_simulation.launch.py', 'launch/xmate3_gazebo.launch.py', 'launch/rviz_only.launch.py']:
    launch_text = (ROOT / launch_file).read_text(encoding='utf-8')
    if 'allow_noncanonical_model' not in launch_text:
        FAILURES.append(f'{launch_file} must expose allow_noncanonical_model to gate developer-only model overrides')

render_helper = (ROOT / 'tools' / 'render_robot_description.py').read_text(encoding='utf-8')
if 'non-canonical model override is disabled by default' not in render_helper:
    FAILURES.append('render_robot_description.py must reject non-canonical model overrides unless developer mode is enabled')

launch_smoke_text = (ROOT / 'tools' / 'run_launch_smoke.sh').read_text(encoding='utf-8')
if 'allow-noncanonical-model false' not in launch_smoke_text or 'allow-noncanonical-model true' not in launch_smoke_text:
    FAILURES.append('run_launch_smoke.sh must verify both rejection and explicit opt-in paths for non-canonical model overrides')
if not (ROOT / 'srv' / 'GetRuntimeStateSnapshot.srv').is_file():
    FAILURES.append('srv/GetRuntimeStateSnapshot.srv is required for the aggregated runtime snapshot surface')

service_facade_header = (ROOT / 'src' / 'runtime' / 'service_facade.hpp').read_text(encoding='utf-8')
if 'handleGetRuntimeStateSnapshot' not in service_facade_header:
    FAILURES.append('QueryFacade must declare handleGetRuntimeStateSnapshot in service_facade.hpp')

if 'get_runtime_state_snapshot.hpp' not in robot_internal_sdk:
    FAILURES.append('sdk robot_internal.hpp must include get_runtime_state_snapshot.hpp for the aggregated snapshot client type')


spec_header = (ROOT / 'include' / 'rokae_xmate3_ros2' / 'spec' / 'xmate3_spec.hpp').read_text(encoding='utf-8')
xacro_text = (ROOT / 'urdf' / 'xMate3.xacro').read_text(encoding='utf-8')

spec_joint_names = _parse_spec_string_array(spec_header, 'kJointNames')
spec_joint_effort = _parse_spec_array(spec_header, 'kJointEffortLimit')
spec_joint_lower = _parse_spec_array(spec_header, 'kJointLimitMin')
spec_joint_upper = _parse_spec_array(spec_header, 'kJointLimitMax')
spec_joint_velocity = _parse_spec_array(spec_header, 'kJointVelocityLimit')
spec_joint_axis = _parse_spec_nested_array(spec_header, 'kJointAxis')
spec_joint_origin = _parse_spec_nested_array(spec_header, 'kJointOrigin')

xacro_joints = _parse_xacro_joint_invocations(xacro_text)
if len(xacro_joints) != len(spec_joint_names):
    FAILURES.append('xMate3.xacro joint count no longer matches xmate3_spec.hpp joint count')
else:
    for index, joint in enumerate(xacro_joints):
        if joint.get('name') != spec_joint_names[index]:
            FAILURES.append(f"xMate3.xacro joint order/name drift at index {index}: {joint.get('name')} != {spec_joint_names[index]}")
            continue
        axis = [float(joint[key]) for key in ('axis_x', 'axis_y', 'axis_z')]
        origin = [float(joint[key]) for key in ('ox', 'oy', 'oz')]
        checks = [
            ('effort', float(joint['limit_effort']), spec_joint_effort[index]),
            ('lower', float(joint['limit_lower']), spec_joint_lower[index]),
            ('upper', float(joint['limit_upper']), spec_joint_upper[index]),
            ('velocity', float(joint['limit_vel']), spec_joint_velocity[index]),
        ]
        for label, actual, expected in checks:
            if not _approx_equal(actual, expected):
                FAILURES.append(f'xMate3.xacro {joint["name"]} {label} drift: {actual} != {expected}')
        for axis_idx, (actual, expected) in enumerate(zip(axis, spec_joint_axis[index])):
            if not _approx_equal(actual, expected):
                FAILURES.append(f'xMate3.xacro {joint["name"]} axis[{axis_idx}] drift: {actual} != {expected}')
        for origin_idx, (actual, expected) in enumerate(zip(origin, spec_joint_origin[index])):
            if not _approx_equal(actual, expected):
                FAILURES.append(f'xMate3.xacro {joint["name"]} origin[{origin_idx}] drift: {actual} != {expected}')

env_lock_text = env_lock.read_text(encoding='utf-8')
if 'Ubuntu 22.04' not in env_lock_text or 'ROS 2 Humble' not in env_lock_text or 'Gazebo 11' not in env_lock_text:
    FAILURES.append('docs/ENVIRONMENT_LOCK.md no longer pins Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 explicitly')

acceptance_workflow_text = acceptance_workflow.read_text(encoding='utf-8')
if './tools/run_target_env_acceptance.sh' not in acceptance_workflow_text:
    FAILURES.append('acceptance workflow no longer executes tools/run_target_env_acceptance.sh')
if 'upload-artifact@v4' not in acceptance_workflow_text or 'acceptance-humble-gazebo11-report' not in acceptance_workflow_text:
    FAILURES.append('acceptance workflow must upload the target-environment acceptance report artifact')
if 'pull_request:' not in acceptance_workflow_text:
    FAILURES.append('acceptance workflow must run on pull_request so locked target-environment regressions are visible before merge')
if 'github.event_name }}" != "workflow_dispatch"' not in acceptance_workflow_text or 'github.event.inputs.launch_smoke }}" = "true"' not in acceptance_workflow_text:
    FAILURES.append('acceptance workflow must enable --launch-smoke by default on push/pull_request and only allow manual opt-out on workflow_dispatch')

dockerfile_text = dockerfile.read_text(encoding='utf-8')
if 'FROM ros:humble-ros-base' not in dockerfile_text or 'gazebo' not in dockerfile_text or 'ros-humble-gazebo-ros-pkgs' not in dockerfile_text:
    FAILURES.append('docker/Dockerfile.humble-gazebo11 no longer matches the locked Humble/Gazebo11 baseline')

acceptance_script_text = acceptance_script.read_text(encoding='utf-8')
if 'Dockerfile.humble-gazebo11' not in acceptance_script_text or 'run_quick_gate.sh' not in acceptance_script_text:
    FAILURES.append('tools/run_target_env_acceptance.sh no longer builds the locked image and executes the quick gate bundle')
if 'run_launch_smoke.sh' not in acceptance_script_text:
    FAILURES.append('tools/run_target_env_acceptance.sh must route --launch-smoke through tools/run_launch_smoke.sh against the built workspace')
if 'write_target_env_report.py' not in acceptance_script_text or 'acceptance_report_' not in acceptance_script_text:
    FAILURES.append('tools/run_target_env_acceptance.sh must emit a machine-readable acceptance report for both local and container paths')

package_xml = (ROOT / 'package.xml').read_text(encoding='utf-8')
if '<depend>Eigen3</depend>' in package_xml:
    FAILURES.append('package.xml still declares Eigen3 instead of the rosdep-resolvable key eigen')
if '<depend>eigen</depend>' not in package_xml:
    FAILURES.append('package.xml must declare the rosdep-resolvable eigen dependency')

if FAILURES:
    for failure in FAILURES:
        print(failure, file=sys.stderr)
    sys.exit(1)


from pathlib import Path
root = Path(__file__).resolve().parents[2]
shim = (root / "include" / "rokae" / "detail" / "sdk_shim_core.hpp").read_text()
robot_h = (root / "include" / "rokae_xmate3_ros2" / "robot.hpp").read_text()
assert "void startReceiveRobotState(std::chrono::steady_clock::duration interval," in shim
assert "session_->robot->startReceiveRobotState(interval, fields);" in shim
assert "session_->robot->updateRobotState(timeout);" in shim
assert "getStateDataMatrix16" in robot_h


quick_gate = (ROOT / "tools" / "run_quick_gate.sh").read_text(encoding="utf-8")
release_gate = (ROOT / "tools" / "run_release_gate.sh").read_text(encoding="utf-8")
target_acceptance = (ROOT / "tools" / "run_target_env_acceptance.sh").read_text(encoding="utf-8")
env_lock = (ROOT / "docs" / "ENVIRONMENT_LOCK.md").read_text(encoding="utf-8")
if 'check_target_environment.sh" --quiet' not in quick_gate:
    raise SystemExit('quick gate must run target environment preflight')
if 'check_target_environment.sh" --quiet' not in release_gate:
    raise SystemExit('release gate must run target environment preflight')
if 'check_target_environment.sh --quiet' not in target_acceptance and 'check_target_environment.sh" --quiet' not in target_acceptance:
    raise SystemExit('target acceptance must run target environment preflight inside container')
if 'tools/check_target_environment.sh' not in env_lock:
    raise SystemExit('environment lock doc must mention the preflight script')
if 'write_target_env_report.py' not in env_lock or 'artifacts/target_env_acceptance/' not in env_lock:
    raise SystemExit('environment lock doc must mention the generated acceptance report path')
