#!/usr/bin/env python3
"""Measure NRT MoveAppend/MoveStart planning latency without changing public wire shapes."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import Log
from rokae_xmate3_ros2.action import MoveAppend
from rokae_xmate3_ros2.msg import JointPosition, MoveAbsJCommand
from rokae_xmate3_ros2.srv import (
    Connect,
    Disconnect,
    MoveReset,
    MoveStart,
    SetMotionControlMode,
    SetOperateMode,
    SetPowerState,
    Stop,
)

from common import RuntimeLogObservation, RuntimeTelemetryMixin


THRESHOLDS_MS = {
    "move_append_accept_ms": 100.0,
    "move_append_result_ms": 150.0,
    "move_start_to_planning_ms": 50.0,
    "planning_to_plan_queued_ms": 100.0,
    "plan_queued_to_execution_started_ms": 50.0,
    "move_start_to_execution_started_ms": 200.0,
}


@dataclass
class LatencyEvents:
    planning_requested: RuntimeLogObservation | None = None
    plan_queued: RuntimeLogObservation | None = None
    execution_started: RuntimeLogObservation | None = None


class NrtPathLatencyProbe(Node, RuntimeTelemetryMixin):
    def __init__(self, namespace: str) -> None:
        super().__init__("nrt_path_latency_probe")
        self.namespace = namespace.rstrip("/")
        self._connect_client = self.create_client(Connect, f"{self.namespace}/cobot/connect")
        self._disconnect_client = self.create_client(Disconnect, f"{self.namespace}/cobot/disconnect")
        self._power_client = self.create_client(SetPowerState, f"{self.namespace}/cobot/set_power_state")
        self._operate_client = self.create_client(SetOperateMode, f"{self.namespace}/cobot/set_operate_mode")
        self._mode_client = self.create_client(SetMotionControlMode, f"{self.namespace}/cobot/set_motion_control_mode")
        self._reset_client = self.create_client(MoveReset, f"{self.namespace}/cobot/move_reset")
        self._start_client = self.create_client(MoveStart, f"{self.namespace}/cobot/move_start")
        self._stop_client = self.create_client(Stop, f"{self.namespace}/cobot/stop")
        self._move_append_client = ActionClient(self, MoveAppend, f"{self.namespace}/cobot/move_append")
        self._jtc_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self._jtc_goal_accept_times: list[float] = []
        self._init_runtime_telemetry()
        self.create_subscription(Log, "/rosout", self._jtc_rosout_callback, 200)

    def _jtc_rosout_callback(self, msg: Log) -> None:
        if msg.name == "joint_trajectory_controller" and "Accepted new action goal" in msg.msg:
            self._jtc_goal_accept_times.append(time.monotonic())
            if len(self._jtc_goal_accept_times) > 32:
                del self._jtc_goal_accept_times[:-32]

    def spin_until(self, predicate, timeout_sec: float, step_sec: float = 0.01) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=step_sec)
            if predicate():
                return True
        return predicate()

    def wait_for_ready(self, timeout_sec: float, *, require_jtc: bool = False) -> None:
        service_specs = [
            (self._connect_client, "connect"),
            (self._disconnect_client, "disconnect"),
            (self._power_client, "set_power_state"),
            (self._operate_client, "set_operate_mode"),
            (self._mode_client, "set_motion_control_mode"),
            (self._reset_client, "move_reset"),
            (self._start_client, "move_start"),
            (self._stop_client, "stop"),
        ]
        deadline = time.monotonic() + timeout_sec
        for client, label in service_specs:
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=0.1):
                    break
                rclpy.spin_once(self, timeout_sec=0.01)
            if not client.service_is_ready():
                raise RuntimeError(f"timed out waiting for {label}")
        while time.monotonic() < deadline:
            if self._move_append_client.wait_for_server(timeout_sec=0.1):
                break
            rclpy.spin_once(self, timeout_sec=0.01)
        else:
            raise RuntimeError("timed out waiting for move_append action server")

        if require_jtc:
            while time.monotonic() < deadline:
                if self._jtc_client.wait_for_server(timeout_sec=0.1):
                    return
                rclpy.spin_once(self, timeout_sec=0.01)
            raise RuntimeError("timed out waiting for joint_trajectory_controller action server")

    def call_service(self, client, request, label: str, timeout_sec: float = 5.0):
        future = client.call_async(request)
        if not self.spin_until(lambda: future.done(), timeout_sec):
            raise RuntimeError(f"timed out waiting for {label}")
        response = future.result()
        if hasattr(response, "success") and not response.success:
            message = getattr(response, "message", "")
            raise RuntimeError(f"{label} failed: {message}")
        return response

    def prepare_robot(self, remote_ip: str, local_ip: str) -> None:
        connect = Connect.Request()
        connect.remote_ip = remote_ip
        connect.local_ip = local_ip
        self.call_service(self._connect_client, connect, "connect")

        operate = SetOperateMode.Request()
        operate.mode = 1
        self.call_service(self._operate_client, operate, "set_operate_mode")

        power = SetPowerState.Request()
        power.on = True
        self.call_service(self._power_client, power, "set_power_state")

        mode = SetMotionControlMode.Request()
        mode.mode = 0
        self.call_service(self._mode_client, mode, "set_motion_control_mode")
        self.call_service(self._reset_client, MoveReset.Request(), "move_reset")

    def cleanup_robot(self) -> None:
        for client, request, label in (
            (self._stop_client, Stop.Request(), "stop"),
            (self._reset_client, MoveReset.Request(), "move_reset"),
        ):
            try:
                self.call_service(client, request, label, timeout_sec=2.0)
            except Exception:
                pass
        try:
            power = SetPowerState.Request()
            power.on = False
            self.call_service(self._power_client, power, "set_power_state(off)", timeout_sec=2.0)
        except Exception:
            pass
        try:
            self.call_service(self._disconnect_client, Disconnect.Request(), "disconnect", timeout_sec=2.0)
        except Exception:
            pass

    def make_goal(self) -> MoveAppend.Goal:
        goal = MoveAppend.Goal()
        command = MoveAbsJCommand()
        command.target = JointPosition()
        command.target.joints = [0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926]
        command.target.external = []
        command.speed = 80
        command.zone = 5
        goal.absj_cmds = [command]
        goal.j_cmds = []
        goal.l_cmds = []
        goal.c_cmds = []
        goal.cf_cmds = []
        goal.sp_cmds = []
        return goal

    def send_move_append(self) -> tuple[str, dict[str, float]]:
        goal = self.make_goal()
        t_send = time.monotonic()
        goal_future = self._move_append_client.send_goal_async(goal)
        if not self.spin_until(lambda: goal_future.done(), 5.0):
            raise RuntimeError("move_append goal response timed out")
        t_accept = time.monotonic()
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("move_append goal rejected")

        result_future = goal_handle.get_result_async()
        if not self.spin_until(lambda: result_future.done(), 5.0):
            raise RuntimeError("move_append queue result timed out")
        t_result = time.monotonic()
        result_wrapper = result_future.result()
        result = getattr(result_wrapper, "result", None)
        if result is None or not result.success:
            message = getattr(result, "message", "")
            raise RuntimeError(f"move_append queue result failed: {message}")
        return result.cmd_id, {
            "move_append_accept_ms": (t_accept - t_send) * 1000.0,
            "move_append_result_ms": (t_result - t_send) * 1000.0,
        }

    def start_and_wait_for_runtime_events(
        self,
        request_id: str,
        *,
        use_jtc_acceptance: bool = False,
    ) -> tuple[dict[str, float], LatencyEvents]:
        t_start = time.monotonic()
        future = self._start_client.call_async(MoveStart.Request())
        if not self.spin_until(lambda: future.done(), 5.0):
            raise RuntimeError("move_start timed out")
        response = future.result()
        if hasattr(response, "success") and not response.success:
            raise RuntimeError(f"move_start failed: {getattr(response, 'message', '')}")

        events = LatencyEvents()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            for entry in self._telemetry_runtime_logs:
                if entry.request_id != request_id or entry.observed_time < t_start:
                    continue
                if events.planning_requested is None and entry.state == "planning":
                    events.planning_requested = entry
                if (
                    events.plan_queued is None
                    and entry.state == "queued"
                    and events.planning_requested is not None
                    and entry.observed_time >= events.planning_requested.observed_time
                ):
                    events.plan_queued = entry
                if events.execution_started is None and entry.state == "executing":
                    events.execution_started = entry
                    if events.plan_queued is None:
                        # In headless NRT the queued state can be shorter than the runtime log publish tick.
                        # Treat planning->executing as a zero-duration queued handoff instead of failing on sampling.
                        events.plan_queued = entry
            if use_jtc_acceptance and events.execution_started is None:
                for accepted_time in self._jtc_goal_accept_times:
                    if accepted_time >= t_start:
                        events.execution_started = RuntimeLogObservation(
                            request_id=request_id,
                            state="executing",
                            backend="jtc",
                            owner="trajectory",
                            message="joint_trajectory_controller accepted goal",
                            observed_time=accepted_time,
                        )
                        if events.plan_queued is None:
                            events.plan_queued = events.execution_started
                        break
            if events.planning_requested and events.plan_queued and events.execution_started:
                break
            time.sleep(0.005)

        if not (events.planning_requested and events.plan_queued and events.execution_started):
            missing = [
                name
                for name, value in (
                    ("planning_requested", events.planning_requested),
                    ("plan_queued", events.plan_queued),
                    ("execution_started", events.execution_started),
                )
                if value is None
            ]
            raise RuntimeError("runtime_event_missing:" + ",".join(missing))

        metrics = {
            "move_start_to_planning_ms": (events.planning_requested.observed_time - t_start) * 1000.0,
            "planning_to_plan_queued_ms": (
                events.plan_queued.observed_time - events.planning_requested.observed_time
            )
            * 1000.0,
            "plan_queued_to_execution_started_ms": (
                events.execution_started.observed_time - events.plan_queued.observed_time
            )
            * 1000.0,
            "move_start_to_execution_started_ms": (events.execution_started.observed_time - t_start) * 1000.0,
        }
        return metrics, events


def classify(metrics: dict[str, float], error: str | None = None) -> str:
    if error is not None:
        if error.startswith("runtime_event_missing"):
            return "runtime_event_missing"
        return "runtime_event_missing"
    checks = [
        ("move_append_accept_ms", "action_accept_slow"),
        ("move_append_result_ms", "queue_result_slow"),
        ("move_start_to_planning_ms", "planning_dispatch_slow"),
        ("planning_to_plan_queued_ms", "planner_latency_regression"),
        ("plan_queued_to_execution_started_ms", "execution_start_slow"),
        ("move_start_to_execution_started_ms", "execution_start_slow"),
    ]
    for key, label in checks:
        if not math.isfinite(metrics.get(key, float("inf"))) or metrics[key] > THRESHOLDS_MS[key]:
            return label
    return "pass"


def print_result(mode: str, request_id: str, metrics: dict[str, float], classification: str, error: str = "") -> None:
    payload = {
        "mode": mode,
        "request_id": request_id,
        "classification": classification,
        "thresholds_ms": THRESHOLDS_MS,
        "error": error,
    }
    payload.update({key: round(float(value), 3) for key, value in metrics.items()})
    print("NRT_PATH_LATENCY_JSON " + json.dumps(payload, sort_keys=True))
    if classification == "pass":
        print(
            "nrt_path_latency: PASS "
            f"mode={mode} request_id={request_id} "
            f"accept_ms={metrics['move_append_accept_ms']:.3f} "
            f"queue_result_ms={metrics['move_append_result_ms']:.3f} "
            f"start_to_exec_ms={metrics['move_start_to_execution_started_ms']:.3f} "
            "classification=pass"
        )
    else:
        print(
            "nrt_path_latency: FAIL "
            f"mode={mode} request_id={request_id or '<unknown>'} "
            f"classification={classification} error={error}",
            file=sys.stderr,
        )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("headless", "gazebo"), required=True)
    parser.add_argument("--namespace", default="/xmate_er3")
    parser.add_argument("--remote-ip", default="127.0.0.1")
    parser.add_argument("--local-ip", default="127.0.0.1")
    args = parser.parse_args(argv)

    rclpy.init()
    probe = NrtPathLatencyProbe(args.namespace)
    request_id = ""
    metrics: dict[str, float] = {}
    error = ""
    try:
        probe.wait_for_ready(45.0, require_jtc=args.mode == "gazebo")
        probe.prepare_robot(args.remote_ip, args.local_ip)
        request_id, metrics = probe.send_move_append()
        runtime_metrics, _ = probe.start_and_wait_for_runtime_events(
            request_id,
            use_jtc_acceptance=args.mode == "gazebo",
        )
        metrics.update(runtime_metrics)
        classification = classify(metrics)
        print_result(args.mode, request_id, metrics, classification)
        return 0 if classification == "pass" else 1
    except Exception as exc:
        error = str(exc)
        classification = classify(metrics, error)
        print_result(args.mode, request_id, metrics, classification, error)
        return 1
    finally:
        probe.cleanup_robot()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
