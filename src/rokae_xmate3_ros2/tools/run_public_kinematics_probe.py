#!/usr/bin/env python3
from __future__ import annotations

import sys
import math
import os
import time

import rclpy
from rclpy.node import Node

from rokae_xmate3_ros2.srv import CalcFk, CalcIk, GetJointPos


def wait_for_service(node: Node, client, name: str, timeout_sec: float = 30.0) -> None:
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(f"service not available: {name}")


def call_service(node: Node, client, request, label: str):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    if not future.done():
        raise RuntimeError(f"{label} timed out")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} returned no result")
    if not getattr(result, "success", False):
        message = getattr(result, "message", "")
        raise RuntimeError(f"{label} failed: {message}")
    return result


def max_abs_delta(lhs: list[float], rhs: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(lhs, rhs))


def try_fk_ik_round_trip(node: Node, fk_client, ik_client, joints: list[float], label: str) -> bool:
    fk_req = CalcFk.Request()
    fk_req.joint_positions = joints
    fk_res = call_service(node, fk_client, fk_req, f"{label}: calc_fk")

    ik_req = CalcIk.Request()
    ik_req.target_posture = list(fk_res.posture)
    ik_req.elbow = 0.0
    ik_req.has_elbow = False
    ik_req.conf_data = []
    ik_req.external = []
    try:
        call_service(node, ik_client, ik_req, f"{label}: calc_ik")
    except RuntimeError as exc:
        print(f"public_kinematics_probe: {label} did not close IK: {exc}", file=sys.stderr)
        return False
    return True


def wait_for_runtime_seed(node: Node, joint_client, seed: list[float]) -> list[float]:
    timeout_sec = float(os.environ.get("ROKAE_PUBLIC_KINEMATICS_TARGET_WAIT_SEC", "45"))
    tolerance = float(os.environ.get("ROKAE_PUBLIC_KINEMATICS_TARGET_TOLERANCE", "0.08"))
    deadline = time.monotonic() + timeout_sec
    last_joints: list[float] = []
    while True:
        joint_res = call_service(node, joint_client, GetJointPos.Request(), "get_joint_pos")
        last_joints = list(joint_res.joint_positions)
        if max_abs_delta(last_joints, seed) <= tolerance:
            return last_joints
        if time.monotonic() >= deadline:
            print(
                "public_kinematics_probe: runtime joint feedback did not reach stable seed "
                f"within {timeout_sec:.1f}s; max_delta={max_abs_delta(last_joints, seed):.4f}; "
                "using latest feedback for FK/IK probe",
            )
            return last_joints
        time.sleep(1.0)


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("xmate_er3_public_kinematics_probe")
    try:
        fk_client = node.create_client(CalcFk, "/xmate_er3/cobot/calc_fk")
        ik_client = node.create_client(CalcIk, "/xmate_er3/cobot/calc_ik")
        joint_client = node.create_client(GetJointPos, "/xmate_er3/cobot/get_joint_pos")
        wait_for_service(node, fk_client, "/xmate_er3/cobot/calc_fk")
        wait_for_service(node, ik_client, "/xmate_er3/cobot/calc_ik")
        wait_for_service(node, joint_client, "/xmate_er3/cobot/get_joint_pos")

        stable_seed_a = [0.0, 0.15, 1.55, 0.0, 1.35, math.pi]
        runtime_joints = wait_for_runtime_seed(node, joint_client, stable_seed_a)
        probe_sets = [
            ("runtime_joint_feedback", runtime_joints),
            ("stable_public_seed_a", stable_seed_a),
            ("stable_public_seed_b", [0.1, 0.2, 1.0, -0.1, 0.8, 0.5]),
        ]
        for label, joints in probe_sets:
            if try_fk_ik_round_trip(node, fk_client, ik_client, joints, label):
                print(f"public_kinematics_probe: fk/ik closed-loop success ({label})")
                return 0

        print("public_kinematics_probe: no FK/IK probe set closed the loop", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"public_kinematics_probe: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
