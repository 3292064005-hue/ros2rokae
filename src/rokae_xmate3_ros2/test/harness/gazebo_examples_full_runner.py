#!/usr/bin/env python3

import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

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

@dataclass(frozen=True)
class ExampleSpec:
    example_id: int
    mode: str
    timeout_sec: float
    required_markers: Tuple[str, ...]
    allow_skip: bool


def build_spec(binary_path: str) -> ExampleSpec:
    name = Path(binary_path).name
    match = re.match(r"example_(\d+)_", name)
    if not match:
        raise ValueError(f"Unrecognized example binary name: {name}")
    example_id = int(match.group(1))

    timeout_sec = 180.0
    if example_id in {8, 18, 24, 25, 26, 99}:
        timeout_sec = 240.0
    if example_id in {13, 20, 21, 22, 23}:
        timeout_sec = 150.0

    if example_id in {20, 21, 22, 23, 24, 25, 26}:
        mode = "experimental_rt"
        allow_skip = True
    elif example_id in {3, 9, 14, 19}:
        mode = "model_approx"
        allow_skip = True
    else:
        mode = "strict"
        allow_skip = False

    return ExampleSpec(
        example_id=example_id,
        mode=mode,
        timeout_sec=timeout_sec,
        required_markers=(f"示例 {example_id}:",),
        allow_skip=allow_skip,
    )


class ExampleProbe(Node, RuntimeCleanupMixin):
    def __init__(self):
        super().__init__("rokae_gazebo_examples_full_probe")
        self._heartbeat_count = 0
        self._connect_client = self.create_client(Connect, "/xmate3/cobot/connect")
        self._set_motion_control_mode_client = self.create_client(
            SetMotionControlMode, "/xmate3/cobot/set_motion_control_mode"
        )
        self._set_operate_mode_client = self.create_client(
            SetOperateMode, "/xmate3/cobot/set_operate_mode"
        )
        self._init_runtime_cleanup_clients()
        self._move_append_client = ActionClient(self, MoveAppend, "/xmate3/cobot/move_append")
        self.create_subscription(JointState, "/xmate3/joint_states", self._on_joint_state, 20)

    def _on_joint_state(self, _msg):
        self._heartbeat_count += 1

    def wait_for_heartbeat(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._heartbeat_count > 0:
                return True
        return False

    def wait_for_fresh_heartbeat(self, timeout_sec: float) -> bool:
        previous_count = self._heartbeat_count
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._heartbeat_count > previous_count:
                return True
        return False

    def wait_for_runtime_ready(self, timeout_sec: float) -> bool:
        service_specs = [
            (self._connect_client, "/xmate3/cobot/connect"),
            (self._set_motion_control_mode_client, "/xmate3/cobot/set_motion_control_mode"),
            (self._set_operate_mode_client, "/xmate3/cobot/set_operate_mode"),
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

def run_example(binary_path: str, spec: ExampleSpec) -> Tuple[bool, subprocess.CompletedProcess[str]]:
    completed = subprocess.run(
        [binary_path],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=spec.timeout_sec,
        check=False,
    )

    success = completed.returncode == 0
    missing_markers = [marker for marker in spec.required_markers if marker not in completed.stdout]
    if missing_markers:
        success = False
    if not spec.allow_skip and "[skipped]" in completed.stdout:
        success = False
        missing_markers.append("unexpected [skipped] marker")

    if success:
        return True, completed

    sys.stderr.write(
        f"{binary_path} ({spec.mode}) exited with {completed.returncode} or failed marker validation\n"
    )
    if missing_markers:
        sys.stderr.write(f"{binary_path} validation issues: {', '.join(missing_markers)}\n")
    sys.stderr.write("----- stdout -----\n")
    sys.stderr.write(completed.stdout)
    sys.stderr.write("----- stderr -----\n")
    sys.stderr.write(completed.stderr)
    return False, completed


def sort_examples(paths: List[str]) -> List[str]:
    def key(path: str) -> Tuple[int, str]:
        name = Path(path).name
        match = re.match(r"example_(\d+)_", name)
        number = int(match.group(1)) if match else 10_000
        return (number, name)

    return sorted(paths, key=key)


def main(argv: List[str]) -> int:
    if len(argv) < 2:
        sys.stderr.write("Usage: gazebo_examples_full_runner.py <example_binary> [...]\n")
        return 2

    example_paths = sort_examples(argv[1:])
    specs: Dict[str, ExampleSpec] = {path: build_spec(path) for path in example_paths}

    rclpy.init()
    probe = ExampleProbe()
    try:
        if not probe.wait_for_heartbeat(45.0):
            sys.stderr.write("Timed out waiting for /xmate3/joint_states before running examples\n")
            return 1
        if not probe.wait_for_runtime_ready(45.0):
            return 1
        time.sleep(1.0)

        total = len(example_paths)
        for index, binary_path in enumerate(example_paths, start=1):
            spec = specs[binary_path]
            sys.stdout.write(
                f"[examples-full] ({index}/{total}) {Path(binary_path).name} mode={spec.mode} timeout={spec.timeout_sec:.0f}s\n"
            )
            sys.stdout.flush()

            if not probe.wait_for_runtime_ready(30.0):
                sys.stderr.write(f"Runtime not ready before {binary_path}\n")
                return 1
            time.sleep(0.75)

            success, _ = run_example(binary_path, spec)
            if not success:
                sys.stderr.write(f"Retrying {binary_path} after runtime settle...\n")
                if not probe.wait_for_runtime_ready(30.0):
                    return 1
                time.sleep(2.0)
                success, _ = run_example(binary_path, spec)
                if not success:
                    return 1

            if not probe.wait_for_fresh_heartbeat(10.0):
                sys.stderr.write(f"/xmate3/joint_states stopped updating after {binary_path}\n")
                return 1
            time.sleep(0.5)

        sys.stdout.write(f"[examples-full] completed {total}/{total} examples successfully\n")
        return 0
    finally:
        probe.cleanup_runtime(delete_entity=False)
        probe.drain_callbacks(5.0)
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
