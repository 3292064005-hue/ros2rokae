#!/usr/bin/env python3
"""Render robot_description and publish it on a transient-local topic for Gazebo spawn_entity."""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--package-share", required=True)
    parser.add_argument("--mesh-root", required=True)
    parser.add_argument("--enable-ros2-control", required=True)
    parser.add_argument("--enable-xcore-plugin", required=True)
    parser.add_argument("--backend-mode", required=True)
    parser.add_argument("--service-exposure-profile", required=True)
    parser.add_argument("--compatibility-alias-policy", required=True)
    parser.add_argument("--canonical-model", default="")
    parser.add_argument("--canonical-metadata", default="")
    parser.add_argument("--allow-noncanonical-model", default="false")
    parser.add_argument("--use-sim-time", default="false")
    parser.add_argument("--lifetime-sec", type=float, default=30.0)
    return parser.parse_args()


def render_description(args: argparse.Namespace) -> str:
    script_path = pathlib.Path(__file__).with_name("render_robot_description.py")
    cmd = [
        sys.executable,
        str(script_path),
        "--model", args.model,
        "--package-share", args.package_share,
        "--mesh-root", args.mesh_root,
        "--enable-ros2-control", args.enable_ros2_control,
        "--enable-xcore-plugin", args.enable_xcore_plugin,
        "--backend-mode", args.backend_mode,
        "--service-exposure-profile", args.service_exposure_profile,
        "--compatibility-alias-policy", args.compatibility_alias_policy,
        "--canonical-model", args.canonical_model,
        "--canonical-metadata", args.canonical_metadata,
        "--allow-noncanonical-model", args.allow_noncanonical_model,
    ]
    completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
    return completed.stdout


class DescriptionPublisher(Node):
    def __init__(self, args: argparse.Namespace, description: str) -> None:
        super().__init__("rokae_description_topic_publisher")
        self.set_parameters([
            Parameter("use_sim_time", value=args.use_sim_time.lower() in {"1", "true", "yes", "on"})
        ])
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, args.topic, qos)
        self.message = String(data=description)

    def publish_once(self) -> None:
        self.publisher.publish(self.message)


def main() -> int:
    args = parse_args()
    description = render_description(args)

    rclpy.init(args=None)
    node = DescriptionPublisher(args, description)
    try:
        deadline = time.monotonic() + max(args.lifetime_sec, 1.0)
        while time.monotonic() < deadline:
            node.publish_once()
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.4)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
