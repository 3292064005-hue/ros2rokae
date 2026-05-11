#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PACKAGING = (ROOT / 'cmake' / 'targets_packaging.cmake').read_text(encoding='utf-8')
PUBLIC_DOC = (ROOT / 'docs' / 'public' / 'PUBLIC_SDK_ARTIFACT.md').read_text(encoding='utf-8')
FAILURES: list[str] = []


def load_runtime_host_policy() -> dict[str, str]:
    policy: dict[str, str] = {}
    for raw in (ROOT / 'config' / 'default_runtime_host_policy.env').read_text(encoding='utf-8').splitlines():
        line = raw.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        policy[key.strip()] = value.strip()
    return policy


POLICY = load_runtime_host_policy()
DEFAULT_RUNTIME_HOST = POLICY.get('ROKAE_DEFAULT_RUNTIME_HOST', 'gazebo_plugin')
DEFAULT_BACKEND_MODE = POLICY.get('ROKAE_DEFAULT_BACKEND_MODE', 'jtc')
DEFAULT_SERVICE_PROFILE = POLICY.get('ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE', 'public_xmate_er3_only')
DEFAULT_ENABLE_ROS2_CONTROL = POLICY.get('ROKAE_DEFAULT_ENABLE_ROS2_CONTROL', 'true')
DEFAULT_ENABLE_XCORE_PLUGIN = POLICY.get('ROKAE_DEFAULT_ENABLE_XCORE_PLUGIN', 'true')
DEFAULT_COMPATIBILITY_ALIAS_POLICY = POLICY.get('ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY', 'canonical_only')

block_match = re.search(r'install\(FILES(?P<body>.*?)DESTINATION share/\$\{PROJECT_NAME\}/generated/urdf\s+COMPONENT public_sdk\s*\)', PACKAGING, re.S)
if not block_match:
    FAILURES.append('public_sdk generated/urdf install block not found in targets_packaging.cmake')
else:
    body = block_match.group('body')
    for token in [
        'ROKAE_GENERATED_XMATE_ER3_URDF',
        'ROKAE_GENERATED_XMATE_ER3_URDF_METADATA',
        'ROKAE_GENERATED_XMATE3_URDF',
        'ROKAE_GENERATED_XMATE3_URDF_METADATA',
    ]:
        if token not in body:
            FAILURES.append(f'public_sdk generated/urdf install block missing {token}')

replay_mode = ROOT / 'cmake' / 'public_sdk_replay_mode.cmake'
if not replay_mode.is_file():
    FAILURES.append('cmake/public_sdk_replay_mode.cmake is required to smoke-replay the public_sdk install surface')

for token in [
    'generated canonical description',
    'generated/urdf/xMateER3.urdf',
    'generated/urdf/xMate3.urdf',
    'share/rokae_xmate3_ros2/tools/*',
    'smoke-verified by `ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON`',
]:
    if token not in PUBLIC_DOC:
        FAILURES.append(f'PUBLIC_SDK_ARTIFACT.md missing token: {token}')


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def run(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd or ROOT, env=env, check=True, capture_output=True, text=True)


def generate_metadata(repo_root: Path, urdf: Path, output: Path, *, source_xacro: Path, source_xacro_rel: str,
                      robot_family: str, robot_model: str, identity_scope: str,
                      canonical_source_xacro: str = 'urdf/xMateER3.xacro') -> None:
    cmd = [
        sys.executable,
        str(repo_root / 'tools' / 'generate_description_metadata.py'),
        '--urdf', str(urdf),
        '--output', str(output),
        '--source-xacro', str(source_xacro),
        '--source-xacro-package-relative', source_xacro_rel,
        '--enable-ros2-control', 'true',
        '--enable-xcore-plugin', 'true',
        '--backend-mode', 'jtc',
        '--service-exposure-profile', 'public_xmate_er3_only',
        '--robot-family', robot_family,
        '--robot-model', robot_model,
        '--identity-scope', identity_scope,
        '--canonical-identity', 'xCoreSDK:xmate_er3',
        '--canonical-source-xacro', canonical_source_xacro,
    ]
    run(cmd, cwd=repo_root)


def stage_replay_source(source_root: Path) -> None:
    ignore = shutil.ignore_patterns(".git", "build", "log", "Testing", "__pycache__", "*.pyc")
    shutil.copytree(ROOT, source_root, dirs_exist_ok=True, ignore=ignore)


def replay_public_sdk_smoke_install(prefix: Path) -> None:
    with tempfile.TemporaryDirectory(prefix='rokae_public_sdk_replay_') as tmpdir:
        tmp = Path(tmpdir)
        source_root = tmp / 'src'
        build_root = tmp / 'build'
        stage_replay_source(source_root)
        run([
            'cmake',
            '-S', str(source_root),
            '-B', str(build_root),
            '-DROKAE_PUBLIC_SDK_REPLAY_ONLY=ON',
        ], cwd=source_root)
        run(['cmake', '--build', str(build_root), '--config', 'Release'], cwd=source_root)
        run(['cmake', '--install', str(build_root), '--prefix', str(prefix), '--component', 'public_sdk', '--config', 'Release'], cwd=source_root)


def verify_core_only_components_consumer(prefix: Path) -> None:
    with tempfile.TemporaryDirectory(prefix='rokae_core_only_consumer_') as tmpdir:
        tmp = Path(tmpdir)
        source_dir = tmp / 'src'
        build_dir = tmp / 'build'
        fake_prefix = tmp / 'fake_eigen_prefix'
        fake_include = fake_prefix / 'include' / 'Eigen'
        fake_cmake = fake_prefix / 'share' / 'eigen3' / 'cmake'
        source_dir.mkdir(parents=True, exist_ok=True)
        fake_include.mkdir(parents=True, exist_ok=True)
        fake_cmake.mkdir(parents=True, exist_ok=True)
        write_text(fake_include / 'Geometry', """#pragma once
namespace Eigen {
struct Isometry {};
template <typename Scalar, int Dim, typename Mode>
struct Transform {
  static Transform Identity() { return Transform(); }
};
}  // namespace Eigen
""")
        write_text(fake_cmake / 'Eigen3Config.cmake', """if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../../../include")
endif()
set(Eigen3_FOUND TRUE)
""")
        write_text(source_dir / 'CMakeLists.txt', """cmake_minimum_required(VERSION 3.16)
project(xcore_sdk_core_only_consumer LANGUAGES CXX)
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
if(NOT TARGET xCoreSDK::xCoreSDK_core)
  message(FATAL_ERROR "xCoreSDK::xCoreSDK_core target is missing from core-only install consumption")
endif()
if(TARGET xCoreSDK::xCoreSDK_ros_bridge)
  message(FATAL_ERROR "core-only install consumption must not resolve xCoreSDK::xCoreSDK_ros_bridge")
endif()
add_library(core_only_model STATIC main.cpp)
target_link_libraries(core_only_model PRIVATE xCoreSDK::xCoreSDK_core)
target_compile_features(core_only_model PRIVATE cxx_std_17)
""")
        write_text(source_dir / 'main.cpp', """#include <array>
#include <system_error>
#include "rokae/model.h"
#include "rokae/planner.h"

int main() {
  std::error_code ec;
  rokae::xMateModel<6> model;
  (void)model.calcFk(std::array<double, 6>{}, ec);
  rokae::JointMotionGenerator planner(0.2, std::array<double, 6>{});
  planner.calculateSynchronizedValues(std::array<double, 6>{});
  std::array<double, 6> delta{};
  (void)planner.calculateDesiredValues(0.0, delta);
  return 0;
}
""")
        env = os.environ.copy()
        env['CMAKE_PREFIX_PATH'] = f"{fake_prefix}{os.pathsep}{prefix}{os.pathsep}{env.get('CMAKE_PREFIX_PATH', '')}"
        run(['cmake', '-S', str(source_dir), '-B', str(build_dir)], cwd=source_dir, env=env)
        run(['cmake', '--build', str(build_dir)], cwd=source_dir, env=env)


def verify_renderer_against_stage(prefix: Path) -> None:
    pkg_share = prefix / 'share' / 'rokae_xmate3_ros2'
    fake_bin = prefix / 'fake-bin'
    fake_bin.mkdir(parents=True, exist_ok=True)
    fake_xacro = fake_bin / 'xacro'
    write_text(fake_xacro, '''#!/usr/bin/env python3
import sys
from pathlib import Path
pairs = {}
for item in sys.argv[2:]:
    if ':=' in item:
        key, value = item.split(':=', 1)
        pairs[key] = value
print('<robot source="{}" backend="{}" profile="{}" plugin="{}" control="{}"/>'.format(
    Path(sys.argv[1]).name,
    pairs.get('backend_mode', ''),
    pairs.get('service_exposure_profile', ''),
    pairs.get('enable_xcore_plugin', ''),
    pairs.get('enable_ros2_control', '')))
''')
    fake_xacro.chmod(fake_xacro.stat().st_mode | stat.S_IEXEC)
    env = os.environ.copy()
    env['PATH'] = f"{fake_bin}{os.pathsep}{env.get('PATH', '')}"

    canonical_urdf = pkg_share / 'generated' / 'urdf' / 'xMateER3.urdf'
    canonical_metadata = pkg_share / 'generated' / 'urdf' / 'xMateER3.description.json'
    alias_urdf = pkg_share / 'generated' / 'urdf' / 'xMate3.urdf'
    alias_metadata = pkg_share / 'generated' / 'urdf' / 'xMate3.description.json'

    def render(model: Path, metadata: Path, *, allow_noncanonical: str = 'false') -> str:
        cmd = [
            sys.executable,
            str(prefix / 'share' / 'rokae_xmate3_ros2' / 'tools' / 'render_robot_description.py'),
            '--model', str(model),
            '--package-share', str(pkg_share),
            '--mesh-root', 'package://rokae_xmate3_ros2/models/rokae_xmate3_ros2/meshes/',
            '--enable-ros2-control', DEFAULT_ENABLE_ROS2_CONTROL,
            '--enable-xcore-plugin', DEFAULT_ENABLE_XCORE_PLUGIN,
            '--backend-mode', DEFAULT_BACKEND_MODE,
            '--service-exposure-profile', DEFAULT_SERVICE_PROFILE,
        '--compatibility-alias-policy', DEFAULT_COMPATIBILITY_ALIAS_POLICY,
            '--canonical-model', str(canonical_urdf),
            '--canonical-metadata', str(metadata),
            '--allow-noncanonical-model', allow_noncanonical,
        ]
        return run(cmd, cwd=ROOT, env=env).stdout.strip()

    canonical_out = render(canonical_urdf, canonical_metadata)
    if '<robot name="xMateER3"' not in canonical_out:
        FAILURES.append('smoke-replayed public_sdk canonical render did not return canonical xMateER3 URDF')

    alias_out = render(alias_urdf, alias_metadata, allow_noncanonical='true')
    if '<robot name="xMate3"' not in alias_out:
        FAILURES.append('smoke-replayed public_sdk compatibility render did not return xMate3 alias URDF')

    config_text = (prefix / 'lib' / 'cmake' / 'xCoreSDK' / 'xCoreSDKConfig.cmake').read_text(encoding='utf-8')
    expected_tokens = [
        'set(xCoreSDK_PRIMARY_INSTALL_CONSUMER "cxx_sdk_core_consumer")',
        f'set(xCoreSDK_DEFAULT_COMPATIBILITY_ALIAS_POLICY "{POLICY.get("ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY", "canonical_only")}")',
        f'set(xCoreSDK_DEFAULT_BACKEND_MODE "{DEFAULT_BACKEND_MODE}")',
        f'set(xCoreSDK_BACKEND_MODE "{DEFAULT_BACKEND_MODE}")',
    ]
    for token in expected_tokens:
        if token not in config_text:
            FAILURES.append(f'smoke-replayed xCoreSDKConfig.cmake missing token: {token}')


if not FAILURES:
    with tempfile.TemporaryDirectory(prefix='rokae_public_sdk_install_') as tmpdir:
        stage_root = Path(tmpdir) / 'stage'
        out_dir = Path(tmpdir) / 'artifact'
        replay_public_sdk_smoke_install(stage_root)
        verify_renderer_against_stage(stage_root)
        verify_core_only_components_consumer(stage_root)
        run([
            sys.executable,
            str(ROOT / 'cmake' / 'assemble_sdk_artifact.py'),
            '--staging-prefix', str(stage_root),
            '--output-dir', str(out_dir),
        ], cwd=ROOT)
        required_outputs = [
            out_dir / 'share' / 'rokae_xmate3_ros2' / 'generated' / 'urdf' / 'xMateER3.urdf',
            out_dir / 'share' / 'rokae_xmate3_ros2' / 'generated' / 'urdf' / 'xMateER3.description.json',
            out_dir / 'share' / 'rokae_xmate3_ros2' / 'generated' / 'urdf' / 'xMate3.urdf',
            out_dir / 'share' / 'rokae_xmate3_ros2' / 'generated' / 'urdf' / 'xMate3.description.json',
            out_dir / 'share' / 'rokae_xmate3_ros2' / 'tools' / 'render_robot_description.py',
            out_dir / 'artifact_manifest.json',
        ]
        for path in required_outputs:
            if not path.exists():
                FAILURES.append(f'smoke-replayed public_sdk artifact missing required path: {path.relative_to(out_dir)}')
        if not FAILURES:
            manifest = json.loads((out_dir / 'artifact_manifest.json').read_text(encoding='utf-8'))
            paths = {entry['path'] for entry in manifest.get('files', [])}
            for rel in [
                'share/rokae_xmate3_ros2/generated/urdf/xMateER3.urdf',
                'share/rokae_xmate3_ros2/generated/urdf/xMateER3.description.json',
                'share/rokae_xmate3_ros2/generated/urdf/xMate3.urdf',
                'share/rokae_xmate3_ros2/generated/urdf/xMate3.description.json',
            ]:
                if rel not in paths:
                    FAILURES.append(f'artifact_manifest.json missing smoke-replayed public_sdk path: {rel}')

if FAILURES:
    print('public SDK packaging contract check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('public SDK packaging contract check passed')
