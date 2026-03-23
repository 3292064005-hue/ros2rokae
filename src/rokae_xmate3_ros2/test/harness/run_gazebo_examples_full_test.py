#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import List


class TeePump(threading.Thread):
    def __init__(self, stream, sink, mirror, line_buffer):
        super().__init__(daemon=True)
        self._stream = stream
        self._sink = sink
        self._mirror = mirror
        self._line_buffer = line_buffer

    def run(self) -> None:
        try:
            for line in iter(self._stream.readline, ""):
                self._sink.write(line)
                self._sink.flush()
                self._mirror.write(line)
                self._mirror.flush()
                self._line_buffer.append(line)
        finally:
            try:
                self._stream.close()
            except Exception:
                pass


def _wait_for_ready(proc: subprocess.Popen[str], lines: List[str], timeout_sec: float) -> bool:
    ready_markers = (
        "spawn_entity.py 已退出",
        "当前 backend_mode=",
        "Successfully spawned entity [xmate]",
    )
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return False
        snapshot = "".join(lines[-200:])
        if any(marker in snapshot for marker in ready_markers):
            return True
        time.sleep(0.2)
    return False


def _force_stop_process_group(proc: subprocess.Popen[str], timeout_sec: float = 10.0) -> int:
    if proc.poll() is not None:
        return proc.returncode

    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return proc.wait(timeout=timeout_sec)

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return proc.returncode
        time.sleep(0.1)
    raise RuntimeError("Timed out waiting for simulation process group to stop")


def _request_graceful_shutdown(
    proc: subprocess.Popen[str],
    sentinel_path: Path,
    timeout_sec: float = 20.0,
) -> int | None:
    if proc.poll() is not None:
        return proc.returncode

    sentinel_path.write_text("shutdown\n", encoding="utf-8")

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return proc.returncode
        time.sleep(0.1)
    return None


def _contains_forbidden_shutdown(lines: List[str]) -> bool:
    text = "".join(lines[-400:])
    return "exit code -11" in text or "exit code -6" in text


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-launch", required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--launch-runner", required=True)
    parser.add_argument("--package-share", required=True)
    parser.add_argument("--package-lib-dir", required=True)
    parser.add_argument("--rosidl-python-path", required=True)
    parser.add_argument("--log-file", required=True)
    parser.add_argument("--python-bin", default=sys.executable)
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.add_argument("--runner-timeout", type=float, default=1800.0)
    parser.add_argument("--example", action="append", default=[])
    args = parser.parse_args()

    if not args.example:
        sys.stderr.write("No example binaries were provided.\n")
        return 2

    log_path = Path(args.log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    shutdown_sentinel = log_path.with_suffix(".shutdown")
    if shutdown_sentinel.exists():
        shutdown_sentinel.unlink()

    env = os.environ.copy()
    env["ROKAE_XMATE3_ROS2_SHARE_DIR"] = args.package_share
    env["ROKAE_XMATE3_ROS2_LIB_DIR"] = args.package_lib_dir
    env["PYTHONUNBUFFERED"] = "1"
    existing_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = (
        args.rosidl_python_path
        if not existing_pythonpath
        else f"{args.rosidl_python_path}:{existing_pythonpath}"
    )

    simulation_cmd = [
        args.python_bin,
        args.launch_runner,
        "--launch-file",
        args.simulation_launch,
        "--shutdown-sentinel",
        str(shutdown_sentinel),
        "--launch-arg",
        "gui:=false",
        "--launch-arg",
        "rviz:=false",
        "--launch-arg",
        "verbose:=false",
        "--launch-arg",
        "enable_ros2_control:=true",
        "--launch-arg",
        "enable_xcore_plugin:=true",
        "--launch-arg",
        "backend_mode:=hybrid",
    ]
    runner_cmd = [args.python_bin, args.runner, *args.example]

    captured_lines: List[str] = []
    with log_path.open("w", encoding="utf-8") as sink:
        sink.write(f"[harness] simulation command: {' '.join(simulation_cmd)}\n")
        sink.write(f"[harness] runner command: {' '.join(runner_cmd)}\n")
        sink.flush()

        simulation = subprocess.Popen(
            simulation_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
            preexec_fn=os.setsid,
        )

        pump = TeePump(simulation.stdout, sink, sys.stdout, captured_lines)
        pump.start()

        try:
            if not _wait_for_ready(simulation, captured_lines, args.ready_timeout):
                sink.write("[harness] simulation did not become ready before timeout\n")
                sink.flush()
                return 1

            runner = subprocess.run(
                runner_cmd,
                env=env,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=args.runner_timeout,
                check=False,
            )
            sink.write(runner.stdout)
            sink.write(runner.stderr)
            sink.flush()
            sys.stdout.write(runner.stdout)
            sys.stderr.write(runner.stderr)
            sys.stdout.flush()
            sys.stderr.flush()

            if runner.returncode != 0:
                sink.write(f"[harness] runner exited with {runner.returncode}\n")
                sink.flush()
                return runner.returncode

            if simulation.poll() is not None and simulation.returncode not in (0, None):
                sink.write(f"[harness] simulation exited early with {simulation.returncode}\n")
                sink.flush()
                return 1

            if _contains_forbidden_shutdown(captured_lines):
                sink.write("[harness] forbidden Gazebo shutdown marker detected before teardown\n")
                sink.flush()
                return 1

            stop_code = _request_graceful_shutdown(simulation, shutdown_sentinel)
            if stop_code is None:
                stop_code = _force_stop_process_group(simulation)
            sink.write(f"[harness] simulation process group stopped with code {stop_code}\n")
            sink.flush()
            pump.join(timeout=5.0)

            if _contains_forbidden_shutdown(captured_lines):
                sink.write("[harness] forbidden Gazebo shutdown marker detected during teardown\n")
                sink.flush()
                return 1

            return 0
        finally:
            if shutdown_sentinel.exists():
                shutdown_sentinel.unlink()
            if simulation.poll() is None:
                try:
                    _force_stop_process_group(simulation, timeout_sec=5.0)
                except Exception as exc:
                    sink.write(f"[harness] final simulation cleanup failed: {exc}\n")
                    sink.flush()
            pump.join(timeout=2.0)


if __name__ == "__main__":
    sys.exit(main())
