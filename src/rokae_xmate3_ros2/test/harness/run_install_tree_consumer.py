#!/usr/bin/env python3
"""Run a staged-install consumer build against the installed xCoreSDK package."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, env=env, check=True)


def _rmtree_onerror(_func, _path, exc_info):
    if isinstance(exc_info[1], FileNotFoundError):
        return
    raise exc_info[1]


def rmtree_idempotent(path: Path) -> None:
    if path.exists() or path.is_symlink():
        shutil.rmtree(path, onerror=_rmtree_onerror)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--build-dir', required=True)
    parser.add_argument('--consumer-dir', required=True)
    parser.add_argument('--staging-prefix', required=True)
    parser.add_argument('--generator', default='Unix Makefiles')
    args = parser.parse_args()

    build_dir = Path(args.build_dir).resolve()
    consumer_dir = Path(args.consumer_dir).resolve()
    staging_prefix = Path(args.staging_prefix).resolve()
    consumer_build_dir = staging_prefix / 'consumer_build'

    rmtree_idempotent(staging_prefix)
    staging_prefix.mkdir(parents=True, exist_ok=True)

    run(['cmake', '--install', str(build_dir), '--prefix', str(staging_prefix)])

    env = os.environ.copy()
    lib_dir = staging_prefix / 'lib'
    if lib_dir.exists():
        ld_paths = [str(lib_dir)]
        if env.get('LD_LIBRARY_PATH'):
            ld_paths.append(env['LD_LIBRARY_PATH'])
        env['LD_LIBRARY_PATH'] = os.pathsep.join(ld_paths)
        if os.name == 'nt':
            path_entries = [str(lib_dir)]
            if env.get('PATH'):
                path_entries.append(env['PATH'])
            env['PATH'] = os.pathsep.join(path_entries)
    prefix_path = str(staging_prefix)
    if env.get('CMAKE_PREFIX_PATH'):
        prefix_path = prefix_path + os.pathsep + env['CMAKE_PREFIX_PATH']
    env['CMAKE_PREFIX_PATH'] = prefix_path

    rmtree_idempotent(consumer_build_dir)
    consumer_build_dir.mkdir(parents=True, exist_ok=True)

    run([
        'cmake',
        '-S', str(consumer_dir),
        '-B', str(consumer_build_dir),
        '-G', args.generator,
        f'-DCMAKE_PREFIX_PATH={prefix_path}',
    ], env=env)
    run(['cmake', '--build', str(consumer_build_dir), '--parallel'], env=env)

    for exe_name in (
        'minimal_connect',
        'minimal_connect_overload',
        'minimal_model',
        'minimal_robot_t',
        'minimal_soft_limit',
        'minimal_execute_movec',
        'minimal_toolset_by_name',
        'minimal_flange_pos',
        'minimal_connect_noec_exception',
        'minimal_rt_header',
        'minimal_planner',
        'minimal_state_and_motion',
        'minimal_static_link_only',
        'minimal_shared_link_only',
        'official_sdk_example_xmate6',
        'official_move_example_xmate6',
        'official_read_robot_state_xmate6',
        'official_path_record_xmate6',
    ):
        exe_path = consumer_build_dir / exe_name
        if os.name == 'nt' and not exe_path.exists():
            exe_path = exe_path.with_suffix('.exe')
        if not exe_path.exists():
            raise FileNotFoundError(f'missing built consumer executable: {exe_path}')
        run([str(exe_path)], env=env)
    return 0


if __name__ == '__main__':
    sys.exit(main())
