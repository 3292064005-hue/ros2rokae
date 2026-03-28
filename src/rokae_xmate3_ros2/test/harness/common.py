#!/usr/bin/env python3

from __future__ import annotations

import re
import os
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from rokae_xmate3_ros2.msg import OperationState
from rokae_xmate3_ros2.srv import (
    Disconnect,
    MoveReset,
    SetMotionControlMode,
    SetOperateMode,
    SetPowerState,
    Stop,
)

try:  # pragma: no cover - depends on ROS distro packages
    from gazebo_msgs.srv import DeleteEntity
except Exception:  # pragma: no cover - best-effort runtime dependency
    DeleteEntity = None  # type: ignore[assignment]


@dataclass(frozen=True)
class RuntimeLogObservation:
    request_id: str
    state: str
    backend: str
    owner: str
    message: str
    observed_time: float


def build_runtime_env(
    package_share: str,
    package_lib_dir: str,
    rosidl_python_path: str,
    runtime_lib_dir: str,
    base_env: Optional[dict[str, str]] = None,
) -> dict[str, str]:
    env = os.environ.copy() if base_env is None else base_env.copy()
    if package_share:
        env.setdefault("ROKAE_XMATE3_ROS2_SHARE_DIR", package_share)
    if package_lib_dir:
        env.setdefault("ROKAE_XMATE3_ROS2_LIB_DIR", package_lib_dir)
    env["PYTHONUNBUFFERED"] = "1"

    if rosidl_python_path:
        existing_pythonpath = env.get("PYTHONPATH", "")
        pythonpath_entries = [entry for entry in existing_pythonpath.split(":") if entry]
        if rosidl_python_path not in pythonpath_entries:
            env["PYTHONPATH"] = (
                rosidl_python_path
                if not existing_pythonpath
                else f"{rosidl_python_path}:{existing_pythonpath}"
            )
    if runtime_lib_dir:
        existing_ld_library_path = env.get("LD_LIBRARY_PATH", "")
        ld_library_entries = [entry for entry in existing_ld_library_path.split(":") if entry]
        if runtime_lib_dir not in ld_library_entries:
            env["LD_LIBRARY_PATH"] = (
                runtime_lib_dir
                if not existing_ld_library_path
                else f"{runtime_lib_dir}:{existing_ld_library_path}"
            )
    return env


class RuntimeCleanupMixin:
    def _init_runtime_cleanup_clients(self) -> None:
        assert isinstance(self, Node)
        self._stop_client = self.create_client(Stop, "/xmate3/cobot/stop")
        self._move_reset_client = self.create_client(MoveReset, "/xmate3/cobot/move_reset")
        self._set_motion_control_mode_client = self.create_client(
            SetMotionControlMode, "/xmate3/cobot/set_motion_control_mode"
        )
        self._set_operate_mode_client = self.create_client(
            SetOperateMode, "/xmate3/cobot/set_operate_mode"
        )
        self._power_client = self.create_client(SetPowerState, "/xmate3/cobot/set_power_state")
        self._disconnect_client = self.create_client(Disconnect, "/xmate3/cobot/disconnect")
        self._delete_entity_client = None
        self._delete_entity_alt_client = None
        if DeleteEntity is not None:
            self._delete_entity_client = self.create_client(DeleteEntity, "/delete_entity")
            self._delete_entity_alt_client = self.create_client(DeleteEntity, "/gazebo/delete_entity")

    def _call_service(self, client, request, service_name: str, wait_sec: float) -> None:
        assert isinstance(self, Node)
        if client is None:
            return
        try:
            if not client.wait_for_service(timeout_sec=wait_sec):
                self.get_logger().info(f"[cleanup] service unavailable, skipping {service_name}")
                return

            future = client.call_async(request)
            deadline = time.monotonic() + wait_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if future.done():
                    try:
                        response = future.result()
                    except Exception as exc:  # pragma: no cover - best effort cleanup
                        self.get_logger().warning(f"[cleanup] {service_name} failed: {exc}")
                        return
                    if hasattr(response, "success"):
                        self.get_logger().info(
                            f"[cleanup] {service_name}: success={response.success} "
                            f"message={getattr(response, 'message', '')}"
                        )
                    else:
                        self.get_logger().info(f"[cleanup] {service_name}: completed")
                    return
            self.get_logger().warning(f"[cleanup] timeout waiting for {service_name}")
        except BaseException as exc:  # pragma: no cover - cleanup must not fail the harness
            self.get_logger().warning(f"[cleanup] aborting {service_name}: {exc}")

    def _delete_robot_entity(self) -> bool:
        assert isinstance(self, Node)
        if DeleteEntity is None:
            self.get_logger().info("[cleanup] gazebo_msgs/DeleteEntity unavailable, skipping entity deletion")
            return False

        request = DeleteEntity.Request()
        if hasattr(request, "name"):
            request.name = "xmate"
        elif hasattr(request, "entity_name"):
            request.entity_name = "xmate"

        for service_name, client in (
            ("/delete_entity", self._delete_entity_client),
            ("/gazebo/delete_entity", self._delete_entity_alt_client),
        ):
            if client is None:
                continue
            try:
                if not client.wait_for_service(timeout_sec=1.0):
                    continue
                self._call_service(client, request, service_name, 5.0)
                time.sleep(0.75)
                return True
            except BaseException as exc:  # pragma: no cover - cleanup must not fail the harness
                self.get_logger().warning(f"[cleanup] aborting entity deletion via {service_name}: {exc}")
                return False

        self.get_logger().info("[cleanup] delete_entity service unavailable, keeping spawned model alive")
        return False

    def delete_robot_entity(self) -> bool:
        return self._delete_robot_entity()

    def cleanup_runtime(self, delete_entity: bool = True) -> None:
        try:
            mode_request = SetMotionControlMode.Request()
            mode_request.mode = 0  # NRT
            self._call_service(
                self._set_motion_control_mode_client,
                mode_request,
                "/xmate3/cobot/set_motion_control_mode",
                3.0,
            )
            time.sleep(0.2)

            operate_request = SetOperateMode.Request()
            operate_request.mode = 1  # AUTOMATIC
            self._call_service(
                self._set_operate_mode_client,
                operate_request,
                "/xmate3/cobot/set_operate_mode",
                3.0,
            )
            time.sleep(0.2)

            self._call_service(self._stop_client, Stop.Request(), "/xmate3/cobot/stop", 3.0)
            time.sleep(0.4)

            self._call_service(self._move_reset_client, MoveReset.Request(), "/xmate3/cobot/move_reset", 3.0)
            time.sleep(0.4)

            power_request = SetPowerState.Request()
            power_request.on = False
            self._call_service(self._power_client, power_request, "/xmate3/cobot/set_power_state", 3.0)
            time.sleep(0.4)

            self._call_service(self._disconnect_client, Disconnect.Request(), "/xmate3/cobot/disconnect", 3.0)
            time.sleep(0.5)
            if delete_entity:
                self._delete_robot_entity()
                time.sleep(1.0)
        except BaseException as exc:  # pragma: no cover - cleanup must not fail the harness
            self.get_logger().warning(f"[cleanup] runtime cleanup interrupted: {exc}")

    def drain_callbacks(self, duration_sec: float) -> None:
        assert isinstance(self, Node)
        deadline = time.monotonic() + max(duration_sec, 0.0)
        while time.monotonic() < deadline:
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except BaseException:  # pragma: no cover - shutdown path
                return
            time.sleep(0.05)


class RuntimeReadinessMixin:
    def _init_runtime_readiness_clients(self) -> None:
        assert isinstance(self, Node)
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def wait_for_jtc_action(self, timeout_sec: float) -> bool:
        assert isinstance(self, Node)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._trajectory_client.wait_for_server(timeout_sec=0.25):
                return True
        return False


class RuntimeTelemetryMixin:
    _RUNTIME_LOG_PATTERN = re.compile(
        r"request=(?P<request>\S+)\s+"
        r"state=(?P<state>\S+)\s+"
        r"backend=(?P<backend>\S+)\s+"
        r"owner=(?P<owner>\S+).*"
        r"message=(?P<message>.*)$"
    )
    _ACTIVE_RUNTIME_STATES = {"planning", "queued", "executing", "settling"}

    def _init_runtime_telemetry(self) -> None:
        assert isinstance(self, Node)
        self._telemetry_joint_heartbeat_count = 0
        self._telemetry_operation_state_count = 0
        self._telemetry_latest_joint_state_time = 0.0
        self._telemetry_latest_operation_state_time = 0.0
        self._telemetry_latest_abs_velocity = float("inf")
        self._telemetry_latest_operation_state = OperationState.UNKNOWN
        self._telemetry_runtime_logs: list[RuntimeLogObservation] = []
        self.create_subscription(JointState, "/xmate3/joint_states", self._telemetry_joint_state_callback, 50)
        self.create_subscription(
            OperationState,
            "/xmate3/cobot/operation_state",
            self._telemetry_operation_state_callback,
            50,
        )
        self.create_subscription(Log, "/rosout", self._telemetry_rosout_callback, 200)

    def _telemetry_joint_state_callback(self, msg: JointState) -> None:
        self._telemetry_joint_heartbeat_count += 1
        self._telemetry_latest_joint_state_time = time.monotonic()
        if msg.velocity:
            self._telemetry_latest_abs_velocity = max(abs(value) for value in msg.velocity)
        else:
            self._telemetry_latest_abs_velocity = 0.0

    def _telemetry_operation_state_callback(self, msg: OperationState) -> None:
        self._telemetry_operation_state_count += 1
        self._telemetry_latest_operation_state_time = time.monotonic()
        self._telemetry_latest_operation_state = msg.state

    def _telemetry_rosout_callback(self, msg: Log) -> None:
        if msg.name != "xcore_gazebo_controller" or "[runtime]" not in msg.msg:
            return
        entry = self._parse_runtime_log(msg.msg, time.monotonic())
        if entry is None:
            return
        self._telemetry_runtime_logs.append(entry)
        if len(self._telemetry_runtime_logs) > 256:
            del self._telemetry_runtime_logs[:-256]

    def _parse_runtime_log(self, text: str, observed_time: float) -> Optional[RuntimeLogObservation]:
        match = self._RUNTIME_LOG_PATTERN.search(text)
        if match is None:
            return None
        return RuntimeLogObservation(
            request_id=match.group("request"),
            state=match.group("state"),
            backend=match.group("backend"),
            owner=match.group("owner"),
            message=match.group("message"),
            observed_time=observed_time,
        )

    def wait_for_joint_heartbeat(self, timeout_sec: float) -> bool:
        assert isinstance(self, Node)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._telemetry_joint_heartbeat_count > 0:
                return True
        return False

    def wait_for_fresh_joint_heartbeat(self, timeout_sec: float) -> bool:
        assert isinstance(self, Node)
        previous_count = self._telemetry_joint_heartbeat_count
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._telemetry_joint_heartbeat_count > previous_count:
                return True
        return False

    def wait_for_runtime_quiescent(
        self,
        timeout_sec: float,
        *,
        quiet_window_sec: float = 1.5,
        velocity_epsilon: float = 0.01,
        stable_sec: float = 0.35,
        label: str = "runtime",
    ) -> bool:
        assert isinstance(self, Node)
        deadline = time.monotonic() + timeout_sec
        satisfied_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            quiescent, _ = self._runtime_quiescent_snapshot(
                now,
                quiet_window_sec=quiet_window_sec,
                velocity_epsilon=velocity_epsilon,
            )
            if quiescent:
                if satisfied_since is None:
                    satisfied_since = now
                if now - satisfied_since >= stable_sec:
                    return True
            else:
                satisfied_since = None
            time.sleep(0.02)

        _, detail = self._runtime_quiescent_snapshot(
            time.monotonic(),
            quiet_window_sec=quiet_window_sec,
            velocity_epsilon=velocity_epsilon,
        )
        sys.stderr.write(f"Timed out waiting for {label} to become quiescent: {detail}\n")
        return False

    def _runtime_quiescent_snapshot(
        self,
        now: float,
        *,
        quiet_window_sec: float,
        velocity_epsilon: float,
    ) -> tuple[bool, str]:
        if self._telemetry_joint_heartbeat_count == 0:
            return False, "no /xmate3/joint_states heartbeat observed"
        if self._telemetry_operation_state_count == 0:
            return False, "no /xmate3/cobot/operation_state heartbeat observed"
        if now - self._telemetry_latest_joint_state_time > 1.0:
            return False, "joint state heartbeat is stale"
        if now - self._telemetry_latest_operation_state_time > 1.0:
            return False, "operation state heartbeat is stale"
        if self._telemetry_latest_operation_state != OperationState.IDLE:
            return False, f"operation state={self._telemetry_latest_operation_state}"
        if self._telemetry_latest_abs_velocity > velocity_epsilon:
            return False, f"joint velocity={self._telemetry_latest_abs_velocity:.4f} rad/s"

        recent_active = None
        for entry in reversed(self._telemetry_runtime_logs):
            owner_active = entry.owner not in {"", "none"}
            state_active = entry.state in self._ACTIVE_RUNTIME_STATES
            if owner_active or state_active:
                recent_active = entry
                break

        if recent_active is not None:
            age = now - recent_active.observed_time
            if age < quiet_window_sec:
                return (
                    False,
                    f"recent runtime activity request={recent_active.request_id} "
                    f"state={recent_active.state} owner={recent_active.owner} age={age:.2f}s",
                )

        if self._telemetry_runtime_logs:
            latest = self._telemetry_runtime_logs[-1]
            detail = (
                f"op=IDLE vel={self._telemetry_latest_abs_velocity:.4f} "
                f"last_runtime=request={latest.request_id} state={latest.state} "
                f"owner={latest.owner}"
            )
            return True, detail
        return True, "op=IDLE with no runtime activity logs"


def ensure_harness_import_path() -> None:
    # Scripts are executed directly by launch testing; keep local sibling imports stable.
    from pathlib import Path

    harness_dir = Path(__file__).resolve().parent
    harness_path = str(harness_dir)
    if harness_path not in sys.path:
        sys.path.insert(0, harness_path)
