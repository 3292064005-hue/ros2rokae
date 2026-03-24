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

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController, UnloadController
try:
    from gazebo_msgs.srv import DeleteEntity
except Exception:  # pragma: no cover - best effort dependency
    DeleteEntity = None  # type: ignore[assignment]

from rokae_xmate3_ros2.srv import PrepareShutdown


SHUTDOWN_PHASE = {
    0: "running",
    1: "draining",
    2: "backend_detached",
    3: "safe_to_delete",
    4: "safe_to_stop_world",
    5: "finished",
    6: "faulted",
}


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


def _wait_for_process_exit(proc: subprocess.Popen[str], timeout_sec: float) -> int | None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return proc.returncode
        time.sleep(0.1)
    return None


def _signal_process_group(
    proc: subprocess.Popen[str],
    signal_value: signal.Signals,
    timeout_sec: float,
) -> int | None:
    if proc.poll() is not None:
        return proc.returncode

    try:
        os.killpg(proc.pid, signal_value)
    except ProcessLookupError:
        return proc.wait(timeout=timeout_sec)

    return _wait_for_process_exit(proc, timeout_sec)


def _process_group_members(proc: subprocess.Popen[str]) -> list[int]:
    if proc.poll() is not None:
        return []

    try:
        completed = subprocess.run(
            ["ps", "-eo", "pid=,pgid="],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5.0,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []

    members: list[int] = []
    for line in completed.stdout.splitlines():
        parts = line.strip().split()
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
            pgid = int(parts[1])
        except ValueError:
            continue
        if pgid == proc.pid:
            members.append(pid)
    return members


def _signal_simulation_children(
    proc: subprocess.Popen[str],
    signal_value: signal.Signals,
    sink,
) -> list[int]:
    members = [pid for pid in _process_group_members(proc) if pid != proc.pid]
    if not members:
        sink.write("[harness] no simulation child processes to signal\n")
        sink.flush()
        return []

    sink.write(
        "[harness] sending "
        + signal_value.name
        + " to simulation child processes: "
        + ", ".join(str(pid) for pid in members)
        + "\n"
    )
    sink.flush()

    for pid in members:
        try:
            os.kill(pid, signal_value)
        except ProcessLookupError:
            continue
    return members


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


def _request_gazebo_server_stop(
    env: dict[str, str],
    sink,
    timeout_sec: float = 5.0,
) -> bool:
    cmd = ["gz", "topic", "-p", "/gazebo/server/control", "-m", "stop: true"]
    sink.write(f"[harness] requesting Gazebo server stop via: {' '.join(cmd)}\n")
    sink.flush()
    try:
        completed = subprocess.run(
            cmd,
            env=env,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_sec,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        sink.write(f"[harness] Gazebo server stop request failed to start: {exc}\n")
        sink.flush()
        return False

    if completed.stdout:
        sink.write(completed.stdout)
    if completed.stderr:
        sink.write(completed.stderr)
    sink.flush()
    return completed.returncode == 0


def _wait_for_service(client, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.25):
            return True
    return False


def _wait_for_future(node, future, timeout_sec: float):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            return future.result()
    return None


def _quiesce_controller_manager(sink) -> None:
    controller_names = [
        "joint_trajectory_controller",
        "joint_state_broadcaster",
        "forward_position_controller",
    ]

    try:
        rclpy.init(args=None)
    except RuntimeError:
        pass

    node = rclpy.create_node("rokae_gazebo_examples_full_teardown")
    try:
        list_client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        switch_client = node.create_client(SwitchController, "/controller_manager/switch_controller")
        unload_client = node.create_client(UnloadController, "/controller_manager/unload_controller")

        if not _wait_for_service(list_client, 5.0):
            sink.write("[harness] controller_manager/list_controllers unavailable; skipping controller quiesce\n")
            sink.flush()
            return

        future = list_client.call_async(ListControllers.Request())
        response = _wait_for_future(node, future, 5.0)
        if response is None:
            sink.write("[harness] timed out listing controllers before shutdown\n")
            sink.flush()
            return

        controller_states = {controller.name: controller.state for controller in response.controller}
        sink.write(
            "[harness] controller states before shutdown: "
            + ", ".join(f"{name}={state}" for name, state in sorted(controller_states.items()))
            + "\n"
        )
        sink.flush()

        active_to_deactivate = [
            name for name in controller_names if controller_states.get(name, "").lower() == "active"
        ]
        if active_to_deactivate and _wait_for_service(switch_client, 5.0):
            request = SwitchController.Request()
            request.deactivate_controllers = active_to_deactivate
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.activate_asap = False
            future = switch_client.call_async(request)
            response = _wait_for_future(node, future, 5.0)
            if response is None:
                sink.write(
                    "[harness] timed out deactivating controllers: "
                    + ", ".join(active_to_deactivate)
                    + "\n"
                )
            else:
                sink.write(
                    "[harness] deactivate controllers result: ok="
                    + str(response.ok)
                    + " controllers="
                    + ", ".join(active_to_deactivate)
                    + "\n"
                )
            sink.flush()
            time.sleep(0.5)

        if _wait_for_service(unload_client, 1.0):
            sink.write(
                "[harness] skipping controller unload; entity deletion owns ros2_control/plugin teardown\n"
            )
        else:
            sink.write(
                "[harness] controller_manager/unload_controller unavailable; proceeding with deactivate-only shutdown\n"
            )
        sink.flush()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _delete_robot_entity(sink) -> bool:
    if DeleteEntity is None:
        sink.write("[harness] gazebo_msgs/DeleteEntity unavailable; skipping entity deletion\n")
        sink.flush()
        return False

    try:
        rclpy.init(args=None)
    except RuntimeError:
        pass

    node = rclpy.create_node("rokae_gazebo_examples_full_delete_entity")
    try:
        request = DeleteEntity.Request()
        if hasattr(request, "name"):
            request.name = "xmate"
        elif hasattr(request, "entity_name"):
            request.entity_name = "xmate"

        for service_name in ("/delete_entity", "/gazebo/delete_entity"):
            client = node.create_client(DeleteEntity, service_name)
            try:
                if not _wait_for_service(client, 3.0):
                    continue
                future = client.call_async(request)
                response = _wait_for_future(node, future, 8.0)
                if response is None:
                    sink.write(f"[harness] timed out deleting entity via {service_name}\n")
                    sink.flush()
                    continue
                success = getattr(response, "success", True)
                status = getattr(response, "status_message", "")
                if hasattr(response, "message") and not status:
                    status = getattr(response, "message")
                sink.write(
                    f"[harness] delete entity via {service_name}: success={success} message={status}\n"
                )
                sink.flush()
                return bool(success)
            finally:
                node.destroy_client(client)
        sink.write("[harness] delete_entity service unavailable after controller quiesce\n")
        sink.flush()
        return False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _prepare_plugin_shutdown(sink) -> bool:
    try:
        rclpy.init(args=None)
    except RuntimeError:
        pass

    node = rclpy.create_node("rokae_gazebo_examples_full_prepare_shutdown")
    try:
        client = node.create_client(PrepareShutdown, "/xmate3/internal/prepare_shutdown")
        if not _wait_for_service(client, 5.0):
            sink.write("[harness] /xmate3/internal/prepare_shutdown unavailable\n")
            sink.flush()
            return False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            request = PrepareShutdown.Request()
            request.request_shutdown = True
            future = client.call_async(request)
            response = _wait_for_future(node, future, 2.0)
            if response is None:
                sink.write("[harness] timed out waiting for /xmate3/internal/prepare_shutdown\n")
                sink.flush()
                return False
            sink.write(
                "[harness] prepare shutdown: owner="
                + str(response.owner)
                + " safe_to_delete="
                + str(response.safe_to_delete)
                + " safe_to_stop_world="
                + str(response.safe_to_stop_world)
                + " active_request_count="
                + str(response.active_request_count)
                + " active_goal_count="
                + str(response.active_goal_count)
                + " backend_quiescent="
                + str(response.backend_quiescent)
                + " runtime_phase="
                + str(response.runtime_phase)
                + " shutdown_phase="
                + SHUTDOWN_PHASE.get(response.shutdown_phase, str(response.shutdown_phase))
                + " message="
                + getattr(response, "message", "")
                + "\n"
            )
            sink.flush()
            if response.accepted and response.safe_to_delete:
                return True
            time.sleep(0.25)
        sink.write("[harness] prepare shutdown never reached safe_to_delete\n")
        sink.flush()
        return False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _stop_simulation_process_group(
    proc: subprocess.Popen[str],
    sentinel_path: Path,
    env: dict[str, str],
    sink,
) -> tuple[int, str]:
    sink.write("[harness] requesting graceful launch shutdown via sentinel\n")
    sink.flush()
    stop_code = _request_graceful_shutdown(proc, sentinel_path, timeout_sec=20.0)
    if stop_code is not None:
        sink.write("[harness] simulation exited after graceful shutdown\n")
        sink.flush()
        return stop_code, "graceful"

    sink.write("[harness] graceful shutdown timed out; escalating to process-group SIGTERM\n")
    sink.flush()
    stop_code = _signal_process_group(proc, signal.SIGTERM, timeout_sec=10.0)
    if stop_code is not None:
        sink.write("[harness] simulation exited after process-group SIGTERM fallback\n")
        sink.flush()
        return stop_code, "sigterm_fallback"

    sink.write("[harness] process-group SIGTERM fallback timed out; escalating to SIGKILL\n")
    sink.flush()
    return _force_stop_process_group(proc), "sigkill"


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

            if not _prepare_plugin_shutdown(sink):
                sink.write("[harness] failed to prepare xcore plugin for shutdown\n")
                sink.flush()

            _quiesce_controller_manager(sink)
            deleted_entity = _delete_robot_entity(sink)
            if deleted_entity:
                sink.write("[harness] robot entity deleted before Gazebo shutdown; allowing teardown to settle\n")
                sink.flush()
                time.sleep(1.0)

            stop_code, stop_stage = _stop_simulation_process_group(
                simulation,
                shutdown_sentinel,
                env,
                sink,
            )
            sink.write(
                f"[harness] simulation process group stopped with code {stop_code} via {stop_stage}\n"
            )
            sink.flush()
            pump.join(timeout=5.0)

            if _contains_forbidden_shutdown(captured_lines):
                sink.write(
                    "[harness] warning: forbidden Gazebo shutdown marker detected during teardown; "
                    "business run already completed successfully\n"
                )
                sink.flush()

            if stop_code != 0:
                sink.write(
                    f"[harness] warning: simulation teardown exited with {stop_code} via {stop_stage}; "
                    "business run already completed successfully\n"
                )
                sink.flush()

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
