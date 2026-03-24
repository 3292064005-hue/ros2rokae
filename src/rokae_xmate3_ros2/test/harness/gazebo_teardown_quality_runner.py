#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rokae_xmate3_ros2.action import MoveAppend
from rokae_xmate3_ros2.srv import Connect, PrepareShutdown, SetMotionControlMode, SetOperateMode

from common import RuntimeCleanupMixin, RuntimeReadinessMixin, RuntimeTelemetryMixin, ensure_harness_import_path


CONTROL_OWNER = {
    0: "none",
    1: "effort",
    2: "trajectory",
}

RUNTIME_PHASE = {
    0: "idle",
    1: "planning",
    2: "executing",
    3: "faulted",
}

SHUTDOWN_PHASE = {
    0: "running",
    1: "draining",
    2: "backend_detached",
    3: "safe_to_delete",
    4: "safe_to_stop_world",
    5: "finished",
    6: "faulted",
}

CONTRACT_CODE = {
    0: "ok",
    1: "shutdown_requested",
    2: "runtime_draining",
    3: "backend_detached",
    4: "safe_to_delete",
    5: "safe_to_stop_world",
    6: "finished",
    7: "faulted",
}


def _wait_for_process_exit(proc: subprocess.Popen[str], timeout_sec: float) -> int | None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return proc.returncode
        time.sleep(0.1)
    return None


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
    raise RuntimeError("timed out waiting for simulation process group to stop")


def _request_graceful_shutdown(
    proc: subprocess.Popen[str],
    sentinel_path: Path,
    timeout_sec: float = 20.0,
) -> int | None:
    if proc.poll() is not None:
        return proc.returncode

    sentinel_path.write_text("shutdown\n", encoding="utf-8")
    return _wait_for_process_exit(proc, timeout_sec)


class TeardownQualityProbe(Node, RuntimeCleanupMixin, RuntimeReadinessMixin, RuntimeTelemetryMixin):
    def __init__(self) -> None:
        super().__init__("rokae_gazebo_teardown_quality_probe")
        self._connect_client = self.create_client(Connect, "/xmate3/cobot/connect")
        self._set_motion_control_mode_client = self.create_client(
            SetMotionControlMode, "/xmate3/cobot/set_motion_control_mode"
        )
        self._set_operate_mode_client = self.create_client(
            SetOperateMode, "/xmate3/cobot/set_operate_mode"
        )
        self._prepare_shutdown_client = self.create_client(
            PrepareShutdown, "/xmate3/internal/prepare_shutdown"
        )
        self._move_append_client = ActionClient(self, MoveAppend, "/xmate3/cobot/move_append")
        self._init_runtime_cleanup_clients()
        self._init_runtime_readiness_clients()
        self._init_runtime_telemetry()

    def wait_for_runtime_ready(self, timeout_sec: float) -> bool:
        service_specs = [
            (self._connect_client, "/xmate3/cobot/connect"),
            (self._set_motion_control_mode_client, "/xmate3/cobot/set_motion_control_mode"),
            (self._set_operate_mode_client, "/xmate3/cobot/set_operate_mode"),
            (self._prepare_shutdown_client, "/xmate3/internal/prepare_shutdown"),
        ]
        deadline = time.monotonic() + timeout_sec
        for client, service_name in service_specs:
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=0.25):
                    break
            if not client.service_is_ready():
                sys.stderr.write(f"Timed out waiting for {service_name}\n")
                return False

        while time.monotonic() < deadline:
            if self._move_append_client.wait_for_server(timeout_sec=0.25):
                break
        else:
            sys.stderr.write("Timed out waiting for /xmate3/cobot/move_append action server\n")
            return False

        if not self.wait_for_jtc_action(max(0.0, deadline - time.monotonic())):
            sys.stderr.write(
                "Timed out waiting for /joint_trajectory_controller/follow_joint_trajectory action server\n"
            )
            return False
        return True

    def wait_for_heartbeat(self, timeout_sec: float) -> bool:
        return self.wait_for_joint_heartbeat(timeout_sec)

    def prepare_shutdown_until_safe(self, timeout_sec: float) -> tuple[bool, list[str], str]:
        shutdown_phases: list[str] = []
        final_detail = ""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            request = PrepareShutdown.Request()
            request.request_shutdown = True
            future = self._prepare_shutdown_client.call_async(request)
            response = None
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    response = future.result()
                    break
            if response is None:
                return False, shutdown_phases, "timed out waiting for prepare_shutdown response"

            owner_name = CONTROL_OWNER.get(response.owner, str(response.owner))
            runtime_phase_name = RUNTIME_PHASE.get(response.runtime_phase, str(response.runtime_phase))
            shutdown_phase_name = SHUTDOWN_PHASE.get(response.shutdown_phase, str(response.shutdown_phase))
            contract_code_name = CONTRACT_CODE.get(response.code, str(response.code))
            shutdown_phases.append(shutdown_phase_name)
            final_detail = (
                f"accepted={response.accepted} "
                f"contract_version={response.contract_version} "
                f"code={contract_code_name} "
                f"owner={owner_name} "
                f"backend_quiescent={response.backend_quiescent} "
                f"safe_to_delete={response.safe_to_delete} safe_to_stop_world={response.safe_to_stop_world} "
                f"active_request_count={response.active_request_count} "
                f"active_goal_count={response.active_goal_count} "
                f"runtime_phase={runtime_phase_name} "
                f"shutdown_phase={shutdown_phase_name} message={response.message}"
            )
            sys.stdout.write(f"[teardown-quality] {final_detail}\n")
            sys.stdout.flush()

            if (
                response.accepted
                and response.contract_version == 1
                and response.shutdown_phase >= 3
                and response.safe_to_delete
                and response.safe_to_stop_world
            ):
                return True, shutdown_phases, final_detail
            time.sleep(0.25)
        return False, shutdown_phases, final_detail or "timed out waiting for safe_to_stop_world"


def _run_single_iteration(args: argparse.Namespace, iteration: int, total_iterations: int) -> int:
    log_path = Path(args.log_file)
    if total_iterations > 1:
        log_path = log_path.with_name(f"{log_path.stem}.run_{iteration:02d}{log_path.suffix}")
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

    with log_path.open("w", encoding="utf-8") as sink:
        sink.write(
            f"[teardown-quality] iteration {iteration}/{total_iterations} simulation command: "
            f"{' '.join(simulation_cmd)}\n"
        )
        sink.flush()
        simulation = subprocess.Popen(
            simulation_cmd,
            stdout=sink,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
            preexec_fn=os.setsid,
        )

        rclpy.init()
        probe = TeardownQualityProbe()
        try:
            if not probe.wait_for_heartbeat(args.ready_timeout):
                sys.stderr.write("Timed out waiting for /xmate3/joint_states before teardown check\n")
                return 1
            if not probe.wait_for_runtime_ready(args.ready_timeout):
                return 1

            ok, shutdown_phases, detail = probe.prepare_shutdown_until_safe(20.0)
            if not ok:
                sys.stderr.write(f"prepare_shutdown did not reach safe contract state: {detail}\n")
                return 1
            required_phases = {
                "draining",
                "backend_detached",
                "safe_to_delete",
                "safe_to_stop_world",
            }
            missing_phases = sorted(required_phases.difference(shutdown_phases))
            if missing_phases:
                sys.stderr.write(
                    "prepare_shutdown did not expose full phase progression: "
                    + ", ".join(missing_phases)
                    + "\n"
                )
                return 1
            sys.stdout.write(f"[teardown-quality] final contract: {detail}\n")
            sys.stdout.flush()

            deleted = probe.delete_robot_entity()
            sys.stdout.write(f"[teardown-quality] delete_robot_entity={deleted}\n")
            sys.stdout.flush()
            if not deleted:
                sys.stderr.write("delete_robot_entity did not report success during teardown quality run\n")
                return 1

            stop_code = _request_graceful_shutdown(simulation, shutdown_sentinel, timeout_sec=20.0)
            if stop_code is None:
                sys.stderr.write("simulation did not exit after graceful shutdown request\n")
                return 1
            if stop_code != 0:
                sys.stderr.write(f"simulation exited with non-zero code during teardown quality run: {stop_code}\n")
                return 1

            sys.stdout.write("[teardown-quality] graceful shutdown completed with exit code 0\n")
            sys.stdout.flush()
            return 0
        finally:
            try:
                probe.destroy_node()
            except BaseException:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except BaseException:
                pass
            if simulation.poll() is None:
                _force_stop_process_group(simulation)
            if shutdown_sentinel.exists():
                shutdown_sentinel.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation-launch", required=True)
    parser.add_argument("--launch-runner", required=True)
    parser.add_argument("--package-share", required=True)
    parser.add_argument("--package-lib-dir", required=True)
    parser.add_argument("--rosidl-python-path", required=True)
    parser.add_argument("--log-file", required=True)
    parser.add_argument("--python-bin", default=sys.executable)
    parser.add_argument("--ready-timeout", type=float, default=90.0)
    parser.add_argument("--repeat", type=int, default=1)
    args = parser.parse_args()

    ensure_harness_import_path()
    for iteration in range(1, max(1, args.repeat) + 1):
        result = _run_single_iteration(args, iteration, max(1, args.repeat))
        if result != 0:
            return result
    return 0


if __name__ == "__main__":
    sys.exit(main())
