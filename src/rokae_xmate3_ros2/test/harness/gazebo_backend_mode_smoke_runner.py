#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory
from rokae_xmate3_ros2.action import MoveAppend
from rokae_xmate3_ros2.srv import Connect

from common import RuntimeCleanupMixin


class BackendModeProbe(Node, RuntimeCleanupMixin):
    def __init__(self):
        super().__init__("rokae_gazebo_backend_mode_probe")
        self._sdk_heartbeat_count = 0
        self._ros2_control_heartbeat_count = 0
        self._connect_client = self.create_client(Connect, "/xmate3/cobot/connect")
        self._init_runtime_cleanup_clients()
        self._move_append_client = ActionClient(self, MoveAppend, "/xmate3/cobot/move_append")
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.create_subscription(JointState, "/xmate3/joint_states", self._sdk_joint_state_callback, 20)
        self.create_subscription(JointState, "/joint_states", self._ros2_control_joint_state_callback, 20)

    def _sdk_joint_state_callback(self, _msg):
        self._sdk_heartbeat_count += 1

    def _ros2_control_joint_state_callback(self, _msg):
        self._ros2_control_heartbeat_count += 1

    def wait_for_heartbeat(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._sdk_heartbeat_count > 0 or self._ros2_control_heartbeat_count > 0:
                return True
        return False

    def wait_for_sdk_heartbeat(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._sdk_heartbeat_count > 0:
                return True
        return False

    def wait_for_ros2_control_heartbeat(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._ros2_control_heartbeat_count > 0:
                return True
        return False

    @property
    def has_sdk_heartbeat(self):
        return self._sdk_heartbeat_count > 0

    @property
    def has_ros2_control_heartbeat(self):
        return self._ros2_control_heartbeat_count > 0

    def wait_for_connect(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._connect_client.wait_for_service(timeout_sec=0.25):
                return True
        return False

    def wait_for_move_append(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._move_append_client.wait_for_server(timeout_sec=0.25):
                return True
        return False

    def wait_for_jtc(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._trajectory_client.wait_for_server(timeout_sec=0.25):
                return True
        return False

def main(argv):
    if len(argv) != 2 or argv[1] not in {"effort", "jtc", "hybrid"}:
        sys.stderr.write("Usage: gazebo_backend_mode_smoke_runner.py <effort|jtc|hybrid>\n")
        return 2

    mode = argv[1]
    rclpy.init()
    probe = BackendModeProbe()
    try:
        if not probe.wait_for_heartbeat(45.0):
            sys.stderr.write("Timed out waiting for /xmate3/joint_states or /joint_states heartbeat\n")
            return 1

        has_connect = probe.wait_for_connect(5.0)
        has_move_append = probe.wait_for_move_append(5.0)
        has_jtc = probe.wait_for_jtc(10.0)

        if mode == "effort":
            if not probe.has_sdk_heartbeat and not probe.wait_for_sdk_heartbeat(5.0):
                sys.stderr.write("effort mode expected /xmate3/joint_states heartbeat from xCore plugin\n")
                return 1
            if not has_connect:
                sys.stderr.write("effort mode expected /xmate3/cobot/connect service\n")
                return 1
            if not has_move_append:
                sys.stderr.write("effort mode expected /xmate3/cobot/move_append action\n")
                return 1
            if has_jtc:
                sys.stderr.write("effort mode should not activate joint_trajectory_controller action server\n")
                return 1
            return 0

        if mode == "jtc":
            if not probe.has_ros2_control_heartbeat and not probe.wait_for_ros2_control_heartbeat(10.0):
                sys.stderr.write("jtc mode expected /joint_states heartbeat from joint_state_broadcaster\n")
                return 1
            if probe.has_sdk_heartbeat:
                sys.stderr.write("jtc mode should not expose /xmate3/joint_states when plugin is disabled\n")
                return 1
            if has_connect:
                sys.stderr.write("jtc mode should not expose /xmate3/cobot/connect service when plugin is disabled\n")
                return 1
            if has_move_append:
                sys.stderr.write("jtc mode should not expose /xmate3/cobot/move_append action when plugin is disabled\n")
                return 1
            if not has_jtc:
                sys.stderr.write("jtc mode expected joint_trajectory_controller action server\n")
                return 1
            return 0

        if not probe.has_sdk_heartbeat and not probe.wait_for_sdk_heartbeat(5.0):
            sys.stderr.write("hybrid mode expected /xmate3/joint_states heartbeat from xCore plugin\n")
            return 1
        if not probe.has_ros2_control_heartbeat and not probe.wait_for_ros2_control_heartbeat(10.0):
            sys.stderr.write("hybrid mode expected /joint_states heartbeat from joint_state_broadcaster\n")
            return 1
        if not has_connect:
            sys.stderr.write("hybrid mode expected /xmate3/cobot/connect service\n")
            return 1
        if not has_move_append:
            sys.stderr.write("hybrid mode expected /xmate3/cobot/move_append action\n")
            return 1
        if not has_jtc:
            sys.stderr.write("hybrid mode expected joint_trajectory_controller action server\n")
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
    sys.exit(main(sys.argv))
