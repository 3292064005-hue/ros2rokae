#!/usr/bin/env python3
import pathlib
import shutil
import sys

sys.dont_write_bytecode = True

ROOT = pathlib.Path(__file__).resolve().parents[2]
FAILURES = []

for pycache_dir in ROOT.rglob('__pycache__'):
    if pycache_dir.is_dir():
        shutil.rmtree(pycache_dir, ignore_errors=True)

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

acceptance_workflow = ROOT / '.github' / 'workflows' / 'acceptance-humble-gazebo11.yml'
if not acceptance_workflow.is_file():
    FAILURES.append('.github/workflows/acceptance-humble-gazebo11.yml is required for locked target-environment acceptance automation')

dockerfile = ROOT / 'docker' / 'Dockerfile.humble-gazebo11'
if not dockerfile.is_file():
    FAILURES.append('docker/Dockerfile.humble-gazebo11 is required to provide the locked target-environment baseline')

sdk_shim_core = (ROOT / 'include' / 'rokae' / 'detail' / 'sdk_shim_core.hpp').read_text(encoding='utf-8')
if 'legacySdkCompatibilityCatalogPolicy()' not in sdk_shim_core:
    FAILURES.append('sdk shim no longer defaults compatibility wrappers to the legacy catalog fallback policy')
if 'normalize_compatibility_client_options' not in sdk_shim_core:
    FAILURES.append('sdk shim no longer normalizes RosClientOptions to preserve compatibility defaults')

robot_clients = (ROOT / 'src' / 'sdk' / 'robot_clients.cpp').read_text(encoding='utf-8')
if 'catalog_policy_ = strictRuntimeCatalogPolicy();' not in robot_clients:
    FAILURES.append('native ROS facade no longer initializes strict runtime catalog policy by default')

package_xml = (ROOT / 'package.xml').read_text(encoding='utf-8')
if '<depend>Eigen3</depend>' in package_xml:
    FAILURES.append('package.xml still declares Eigen3 instead of the rosdep-resolvable key eigen')
if '<depend>eigen</depend>' not in package_xml:
    FAILURES.append('package.xml must declare the rosdep-resolvable eigen dependency')

if FAILURES:
    for failure in FAILURES:
        print(failure, file=sys.stderr)
    sys.exit(1)
