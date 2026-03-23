#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
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
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            self._call_service(client, request, service_name, 5.0)
            time.sleep(0.75)
            return True

        self.get_logger().info("[cleanup] delete_entity service unavailable, keeping spawned model alive")
        return False

    def cleanup_runtime(self, delete_entity: bool = True) -> None:
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

    def drain_callbacks(self, duration_sec: float) -> None:
        assert isinstance(self, Node)
        deadline = time.monotonic() + max(duration_sec, 0.0)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
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


def ensure_harness_import_path() -> None:
    # Scripts are executed directly by launch testing; keep local sibling imports stable.
    from pathlib import Path

    harness_dir = Path(__file__).resolve().parent
    harness_path = str(harness_dir)
    if harness_path not in sys.path:
        sys.path.insert(0, harness_path)
