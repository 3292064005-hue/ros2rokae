#!/usr/bin/env python3

import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rokae_xmate3_ros2.action import MoveAppend
from rokae_xmate3_ros2.srv import (
    Connect,
    SetMotionControlMode,
    SetOperateMode,
)

from common import RuntimeCleanupMixin


class JointStateProbe(Node, RuntimeCleanupMixin):
    def __init__(self):
        super().__init__("rokae_gazebo_examples_smoke_probe")
        self._heartbeat_count = 0
        self._last_msg_time = None
        self._connect_client = self.create_client(Connect, "/xmate3/cobot/connect")
        self._set_motion_control_mode_client = self.create_client(
            SetMotionControlMode, "/xmate3/cobot/set_motion_control_mode"
        )
        self._set_operate_mode_client = self.create_client(
            SetOperateMode, "/xmate3/cobot/set_operate_mode"
        )
        self._init_runtime_cleanup_clients()
        self._move_append_client = ActionClient(self, MoveAppend, "/xmate3/cobot/move_append")
        self.create_subscription(JointState, "/xmate3/joint_states", self._callback, 20)

    def _callback(self, _msg):
        self._heartbeat_count += 1
        self._last_msg_time = time.monotonic()

    def wait_for_heartbeat(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._heartbeat_count > 0:
                return True
        return False

    def wait_for_fresh_heartbeat(self, timeout_sec):
        previous_count = self._heartbeat_count
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._heartbeat_count > previous_count:
                return True
        return False

    def wait_for_runtime_ready(self, timeout_sec):
        service_specs = [
            (self._connect_client, "/xmate3/cobot/connect"),
            (self._set_motion_control_mode_client, "/xmate3/cobot/set_motion_control_mode"),
            (self._set_operate_mode_client, "/xmate3/cobot/set_operate_mode"),
            (self._power_client, "/xmate3/cobot/set_power_state"),
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
                return True
        sys.stderr.write("Timed out waiting for /xmate3/cobot/move_append action server\n")
        return False

def run_example(binary_path, required_markers):
    completed = subprocess.run(
        [binary_path],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=180,
        check=False,
    )

    success = completed.returncode == 0
    missing_markers = [marker for marker in required_markers if marker not in completed.stdout]
    if missing_markers:
        success = False

    if success:
        return True, completed

    sys.stderr.write(f"{binary_path} exited with {completed.returncode}\n")
    if missing_markers:
        sys.stderr.write(
            f"{binary_path} missing expected markers: {', '.join(missing_markers)}\n"
        )
    sys.stderr.write(completed.stdout)
    sys.stderr.write(completed.stderr)
    return False, completed


def main(argv):
    if len(argv) != 4:
        sys.stderr.write(
            "Usage: gazebo_examples_smoke_runner.py <example_04> <example_05> <example_18>\n"
        )
        return 2

    rclpy.init()
    probe = JointStateProbe()
    try:
        if not probe.wait_for_heartbeat(45.0):
            sys.stderr.write("Timed out waiting for /xmate3/joint_states before running examples\n")
            return 1
        if not probe.wait_for_runtime_ready(45.0):
            return 1
        time.sleep(1.0)

        examples = [
            (argv[1], ["MoveAbsJ", "多段关节路径"]),
            (argv[2], ["MoveJ", "MoveL", "MoveC"]),
            (argv[3], ["toolset / calibrateFrame", "标定"]),
        ]
        for binary_path, markers in examples:
            if not probe.wait_for_runtime_ready(20.0):
                sys.stderr.write(f"Runtime not ready before {binary_path}\n")
                return 1
            time.sleep(1.0)

            success, _ = run_example(binary_path, markers)
            if not success:
                sys.stderr.write(f"Retrying {binary_path} after runtime settle...\n")
                if not probe.wait_for_runtime_ready(20.0):
                    return 1
                time.sleep(2.0)
                success, _ = run_example(binary_path, markers)
                if not success:
                    return 1
            if not probe.wait_for_fresh_heartbeat(10.0):
                sys.stderr.write(f"/xmate3/joint_states stopped updating after {binary_path}\n")
                return 1
            time.sleep(0.5)
        return 0
    finally:
        probe.cleanup_runtime()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
