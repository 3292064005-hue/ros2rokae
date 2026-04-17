#!/usr/bin/env python3
"""Build and run an install-tree consumer without sourcing ROS shell setup files."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
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


def sanitized_env(staging_prefix: Path) -> dict[str, str]:
    env = os.environ.copy()
    for key in (
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "ROS_PACKAGE_PATH",
        "ROS_DISTRO",
        "ROS_VERSION",
        "ROS_PYTHON_VERSION",
        "RMW_IMPLEMENTATION",
    ):
        env.pop(key, None)

    ros_prefix = Path("/opt/ros/humble")
    prefix_entries = [str(staging_prefix)]
    if ros_prefix.exists():
        prefix_entries.append(str(ros_prefix))
    prefix_path = os.pathsep.join(prefix_entries)
    env["CMAKE_PREFIX_PATH"] = prefix_path
    if ros_prefix.exists():
        # Keep the environment unsourced from workspace overlays while still
        # allowing ament index resource lookups from the system ROS prefix.
        env["AMENT_PREFIX_PATH"] = str(ros_prefix)
    lib_dir = staging_prefix / "lib"
    if lib_dir.exists():
        env["LD_LIBRARY_PATH"] = str(lib_dir) + os.pathsep + env.get("LD_LIBRARY_PATH", "")
    # Keep the environment unsourced from ROS setup files while still allowing CMake's ament macros
    # to import python modules required by dependency package configs.
    ros_python = str(ros_prefix / "lib/python3.10/site-packages")
    if Path(ros_python).exists():
        env["PYTHONPATH"] = ros_python
    return env


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", required=True)
    parser.add_argument("--consumer-dir", required=True)
    parser.add_argument("--staging-prefix", required=True)
    parser.add_argument("--generator", default="Unix Makefiles")
    args = parser.parse_args()

    build_dir = Path(args.build_dir).resolve()
    consumer_dir = Path(args.consumer_dir).resolve()
    staging_prefix = Path(args.staging_prefix).resolve()
    consumer_build_dir = staging_prefix / "consumer_build_no_ros_env"

    rmtree_idempotent(staging_prefix)
    staging_prefix.mkdir(parents=True, exist_ok=True)
    run(["cmake", "--install", str(build_dir), "--prefix", str(staging_prefix)])

    env = sanitized_env(staging_prefix)

    rmtree_idempotent(consumer_build_dir)
    consumer_build_dir.mkdir(parents=True, exist_ok=True)
    run(
        [
            "cmake",
            "-S",
            str(consumer_dir),
            "-B",
            str(consumer_build_dir),
            "-G",
            args.generator,
            f"-DCMAKE_PREFIX_PATH={env['CMAKE_PREFIX_PATH']}",
            "-DPython3_EXECUTABLE=/usr/bin/python3",
        ],
        env=env,
    )
    run(["cmake", "--build", str(consumer_build_dir), "--parallel"], env=env)

    daemon_path = staging_prefix / "bin" / "rokae_sim_runtime"
    if os.name == "nt" and not daemon_path.exists():
        daemon_path = daemon_path.with_suffix(".exe")
    if not daemon_path.exists():
        raise FileNotFoundError(f"missing runtime daemon executable: {daemon_path}")

    daemon_log = staging_prefix / "daemon_no_ros_env.log"
    with daemon_log.open("w", encoding="utf-8") as log_file:
        daemon = subprocess.Popen([str(daemon_path)], env=env, stdout=log_file, stderr=subprocess.STDOUT)
        try:
            exe_path = consumer_build_dir / "minimal_no_ros_env_runtime"
            if os.name == "nt" and not exe_path.exists():
                exe_path = exe_path.with_suffix(".exe")
            if not exe_path.exists():
                raise FileNotFoundError(f"missing consumer executable: {exe_path}")

            last_rc = 1
            for _ in range(20):
                completed = subprocess.run([str(exe_path)], env=env, check=False)
                last_rc = completed.returncode
                if last_rc == 0:
                    return 0
                time.sleep(0.25)
            raise RuntimeError(
                f"no-ROS env runtime consumer failed after retries, rc={last_rc}, see {daemon_log}"
            )
        finally:
            daemon.terminate()
            try:
                daemon.wait(timeout=10)
            except subprocess.TimeoutExpired:
                daemon.kill()
                daemon.wait(timeout=5)


if __name__ == "__main__":
    sys.exit(main())
