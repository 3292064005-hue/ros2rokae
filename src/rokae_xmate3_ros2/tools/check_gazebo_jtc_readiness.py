#!/usr/bin/env python3
"""Fail-fast readiness gate for the default Gazebo/JTC public launch profile."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--controller', default='joint_trajectory_controller')
    parser.add_argument('--action', default='/joint_trajectory_controller/follow_joint_trajectory')
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    return parser.parse_args()


class JtcReadinessProbe(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('rokae_jtc_readiness_probe')
        self.args = args
        self.list_client = self.create_client(ListControllers, f'{args.controller_manager}/list_controllers')
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, args.action)

    def _deadline_left(self, deadline: float) -> float:
        return max(0.0, deadline - time.monotonic())

    def wait_for_list_controllers(self, deadline: float) -> bool:
        while self._deadline_left(deadline) > 0.0:
            if self.list_client.wait_for_service(timeout_sec=0.25):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def wait_for_controller_active(self, deadline: float) -> bool:
        request = ListControllers.Request()
        while self._deadline_left(deadline) > 0.0:
            future = self.list_client.call_async(request)
            while not future.done() and self._deadline_left(deadline) > 0.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                response = future.result()
                if response is not None:
                    for controller in response.controller:
                        if controller.name == self.args.controller and controller.state == 'active':
                            return True
            time.sleep(0.25)
        return False

    def wait_for_action_server(self, deadline: float) -> bool:
        while self._deadline_left(deadline) > 0.0:
            if self.trajectory_client.wait_for_server(timeout_sec=0.25):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = JtcReadinessProbe(args)
    deadline = time.monotonic() + max(args.timeout_sec, 1.0)
    try:
        if not node.wait_for_list_controllers(deadline):
            sys.stderr.write(f'timed out waiting for {args.controller_manager}/list_controllers\n')
            return 1
        if not node.wait_for_controller_active(deadline):
            sys.stderr.write(f'timed out waiting for controller {args.controller} to become active\n')
            return 1
        if not node.wait_for_action_server(deadline):
            sys.stderr.write(f'timed out waiting for action server {args.action}\n')
            return 1
        node.get_logger().info(
            f'JTC readiness confirmed: controller={args.controller} action={args.action}'
        )
        return 0
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
