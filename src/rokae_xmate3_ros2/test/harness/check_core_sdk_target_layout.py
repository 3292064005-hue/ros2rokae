#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TARGETS = (ROOT / 'cmake' / 'targets_sdk_compat.cmake').read_text(encoding='utf-8')
CONFIG = (ROOT / 'cmake' / 'xCoreSDKConfig.cmake.in').read_text(encoding='utf-8')
FAILURES: list[str] = []

required_target_tokens = [
    'add_library(xCoreSDK_core STATIC',
    'src/compat/model_api_core.cpp',
    'src/compat/planner_api_core.cpp',
    'target_link_libraries(xCoreSDK_static',
    'PUBLIC\n      xCoreSDK_core' if False else 'xCoreSDK_core',
    'add_library(xCoreSDK_ros_bridge INTERFACE)',
]
for token in required_target_tokens:
    if token not in TARGETS:
        FAILURES.append(f'targets_sdk_compat.cmake missing token: {token}')

if 'add_library(xCoreSDK_core INTERFACE' in TARGETS:
    FAILURES.append('xCoreSDK_core must not remain an INTERFACE-only empty target')

for token in [
    'include("${CMAKE_CURRENT_LIST_DIR}/xCoreSDKCoreTargets.cmake")',
    'if(_xcoresdk_core_only)',
    'set(xCoreSDK_STATIC_PROVIDER "core-only")',
    'include("${CMAKE_CURRENT_LIST_DIR}/xCoreSDKTargets.cmake")',
]:
    if token not in CONFIG:
        FAILURES.append(f'xCoreSDKConfig.cmake.in missing token: {token}')

ros_dep_index = CONFIG.find('find_dependency(rclcpp REQUIRED CONFIG)')
core_guard_index = CONFIG.find('if(_xcoresdk_core_only)')
if ros_dep_index != -1 and core_guard_index != -1 and ros_dep_index < core_guard_index:
    FAILURES.append('core-only branch must be evaluated before ROS bridge dependencies are required')

if FAILURES:
    print('core SDK target layout check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('core SDK target layout check passed')
