#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []


def read(rel: str) -> str:
    path = ROOT / rel
    if not path.is_file():
        FAILURES.append(f'missing file: {rel}')
        return ''
    return path.read_text(encoding='utf-8')


try:
    matrix = json.loads(read('config/xmate_er3_capability_matrix.json'))
except Exception as exc:
    print(f'capability matrix contract check failed: invalid JSON: {exc}')
    sys.exit(1)

profiles = matrix.get('launch_profiles', {})
backend_modes = matrix.get('backend_modes', {})
service_profiles = matrix.get('service_exposure_profiles', {})

if matrix.get('default_launch_profile') != 'public_xmate_er3_jtc':
    FAILURES.append('default_launch_profile must be public_xmate_er3_jtc')
if matrix.get('default_service_exposure_profile') != 'public_xmate_er3_only':
    FAILURES.append('default_service_exposure_profile must be public_xmate_er3_only')
if matrix.get('default_compatibility_alias_policy') != 'canonical_only':
    FAILURES.append('default_compatibility_alias_policy must be canonical_only')
for name in ['public_xmate_er3_jtc', 'public_xmate_er3_headless_sdk_smoke', 'public_xmate_er3_experimental_rt', 'public_xmate_er3_sdk']:
    if name not in profiles:
        FAILURES.append(f'capability matrix missing launch profile: {name}')
for name, spec in profiles.items():
    if not isinstance(spec, dict):
        FAILURES.append(f'launch profile {name} must be an object')
        continue
    backend = spec.get('backend_mode')
    if backend not in backend_modes:
        FAILURES.append(f'launch profile {name} references undeclared backend_mode={backend}')
    if spec.get('service_exposure_profile') not in service_profiles:
        FAILURES.append(f'launch profile {name} references undeclared service exposure profile')
    required = spec.get('required_capabilities', [])
    if not isinstance(required, list) or not required:
        FAILURES.append(f'launch profile {name} must declare non-empty required_capabilities')
    backend_required = set(backend_modes.get(backend, {}).get('required_capabilities', []))
    if backend_required and not backend_required.issubset(set(required)):
        FAILURES.append(f'launch profile {name} must include backend required capabilities {sorted(backend_required)}')

public_disabled = set(service_profiles.get('public_xmate_er3_only', {}).get('disabled_modules', []))
if 'move_sp_motion_extension' not in public_disabled:
    FAILURES.append('default public service exposure must disable move_sp_motion_extension')
experimental_enabled = set(service_profiles.get('public_xmate_er3_experimental', {}).get('enabled_modules', []))
if 'move_sp_motion_extension' not in experimental_enabled:
    FAILURES.append('experimental service exposure must enable move_sp_motion_extension')

launch_profile_py = read('launch/_launch_profile.py')
for token in ['xmate_er3_capability_matrix.json', 'capability_matrix()', 'backend_mode_requirements', 'service_exposure_profiles']:
    if token not in launch_profile_py:
        FAILURES.append(f'launch/_launch_profile.py missing matrix source token: {token}')
simulation_support_py = read('launch/_simulation_support.py')
for token in ['capability_matrix()', '_capabilities_for_resolved_profile', 'required = set(profile.required_capabilities)', 'backend_mode_requirements(backend_mode)']:
    if token not in simulation_support_py:
        FAILURES.append(f'launch/_simulation_support.py missing matrix enforcement token: {token}')

gazebo_backend = read('src/gazebo/gazebo_runtime_backend.hpp')
for token in ['action_server_is_ready()', 'wait_for_action_server(std::chrono::seconds(3))', 'joint_trajectory_controller action server is unavailable']:
    if token not in gazebo_backend:
        FAILURES.append(f'Gazebo backend missing JTC readiness token: {token}')

host_builder = read('src/runtime/runtime_host_builder.cpp')
for token in ['setExperimentalMotionExtensionsEnabled', 'includesExperimentalServices(service_exposure_profile)', 'to_string(service_exposure_profile)']:
    if token not in host_builder:
        FAILURES.append(f'RuntimeHostBuilder missing service-exposure motion-extension gate token: {token}')
request_adapter = read('src/runtime/request_adapter.cpp')
for token in ['MoveSP is an experimental motion extension', 'recorded path replay is an experimental motion extension', 'context.experimental_motion_extensions_enabled']:
    if token not in request_adapter:
        FAILURES.append(f'request_adapter missing default-public extension rejection token: {token}')

for rel in ['urdf/xMateER3.xacro', 'urdf/xMate3.xacro']:
    if '<xacro:arg name="compatibility_alias_policy" default="canonical_only"/>' not in read(rel):
        FAILURES.append(f'{rel} must default compatibility_alias_policy to canonical_only')

renderer = read('tools/render_robot_description.py')
noncanonical_guard = renderer.find('non-canonical model override is disabled by default')
xacro_early_return = renderer.find("if model_path.suffix.lower() == '.xacro':")
if noncanonical_guard < 0 or xacro_early_return < 0 or noncanonical_guard > xacro_early_return:
    FAILURES.append('render_robot_description.py must enforce non-canonical guard before any .xacro rendering branch')

if FAILURES:
    print('capability matrix contract check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('capability matrix contract check passed')
