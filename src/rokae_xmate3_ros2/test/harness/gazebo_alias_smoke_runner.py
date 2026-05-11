#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rokae_xmate3_ros2.action import MoveAppend
from rokae_xmate3_ros2.srv import Connect

from common import RuntimeCleanupMixin, RuntimeReadinessMixin


class AliasSmokeProbe(Node, RuntimeCleanupMixin, RuntimeReadinessMixin):
    def __init__(self):
        super().__init__("rokae_gazebo_alias_smoke_probe")
        self._connect_client = self.create_client(Connect, "/xmate3/cobot/connect")
        self._init_runtime_cleanup_clients()
        self._init_runtime_readiness_clients()
        self._move_append_client = ActionClient(self, MoveAppend, "/xmate3/cobot/move_append")
        self._heartbeat_count = 0
        self.create_subscription(JointState, "/xmate3/joint_states", self._joint_state_callback, 20)

    def _joint_state_callback(self, _msg):
        self._heartbeat_count += 1

    def wait_for_service(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._connect_client.wait_for_service(timeout_sec=0.25):
                return True
        return False

    def wait_for_action(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._move_append_client.wait_for_server(timeout_sec=0.25):
                break
        else:
            return False
        return self.wait_for_jtc_action(max(0.0, deadline - time.monotonic()))

    def wait_for_heartbeat(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._heartbeat_count > 0:
                return True
        return False

def main():
    rclpy.init()
    probe = AliasSmokeProbe()
    try:
        if not probe.wait_for_service(45.0):
            sys.stderr.write("Timed out waiting for /xmate3/cobot/connect\n")
            return 1
        if not probe.wait_for_action(45.0):
            sys.stderr.write(
                "Timed out waiting for /xmate3/cobot/move_append or "
                "/joint_trajectory_controller/follow_joint_trajectory action server\n"
            )
            return 1
        if not probe.wait_for_heartbeat(45.0):
            sys.stderr.write("Timed out waiting for /xmate3/joint_states heartbeat\n")
            return 1
        return 0
    finally:
        try:
            probe.cleanup_runtime()
        except BaseException as exc:
            sys.stderr.write(f"[cleanup] ignoring runtime cleanup interruption: {exc}\n")
        try:
            probe.destroy_node()
        except BaseException:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    sys.exit(main())
