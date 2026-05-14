#!/usr/bin/env python3
"""Black-box NRT path smoothness probe for GenerateSTrajectory runtime output."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from rokae_xmate3_ros2.srv import GenerateSTrajectory


REQUESTED_MAX_VELOCITY_RAD_S = 1.0
REQUESTED_MAX_ACCELERATION_RAD_S2 = 2.0
THRESHOLDS = {
    "min_point_count": 3,
    "max_endpoint_error_rad": 1e-3,
    "velocity_limit_scale": 1.25,
    "acceleration_limit_scale": 1.50,
    "jerk_p99_est_rad_s3": 500.0,
    "jerk_max_est_rad_s3": 1500.0,
}


@dataclass
class SmoothnessMetrics:
    point_count: int = 0
    sample_dt_ms: float = math.inf
    max_endpoint_error_rad: float = math.inf
    max_velocity_est_rad_s: float = math.inf
    max_acceleration_est_rad_s2: float = math.inf
    jerk_p99_est_rad_s3: float = math.inf
    jerk_max_est_rad_s3: float = math.inf
    max_position_step_rad: float = math.inf

    def as_dict(self) -> dict[str, float | int]:
        return {
            "point_count": self.point_count,
            "sample_dt_ms": self.sample_dt_ms,
            "max_endpoint_error_rad": self.max_endpoint_error_rad,
            "max_velocity_est_rad_s": self.max_velocity_est_rad_s,
            "max_acceleration_est_rad_s2": self.max_acceleration_est_rad_s2,
            "jerk_p99_est_rad_s3": self.jerk_p99_est_rad_s3,
            "jerk_max_est_rad_s3": self.jerk_max_est_rad_s3,
            "max_position_step_rad": self.max_position_step_rad,
        }


class NrtPathSmoothnessProbe(Node):
    def __init__(self, namespace: str) -> None:
        super().__init__("nrt_path_smoothness_probe")
        self.namespace = namespace.rstrip("/")
        self._client = self.create_client(
            GenerateSTrajectory,
            f"{self.namespace}/cobot/generate_s_trajectory",
        )

    def wait_for_ready(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._client.wait_for_service(timeout_sec=0.1):
                return
            rclpy.spin_once(self, timeout_sec=0.01)
        raise RuntimeError("service_missing:generate_s_trajectory")

    def call_generate_s_trajectory(self) -> tuple[list[list[float]], float, list[float], list[float]]:
        request = GenerateSTrajectory.Request()
        request.start_joint_pos = [0.0, 0.15, 1.55, 0.0, 1.35, 3.1415926]
        request.target_joint_pos = [0.12, 0.25, 1.40, 0.04, 1.20, 3.00]
        request.max_velocity = REQUESTED_MAX_VELOCITY_RAD_S
        request.max_acceleration = REQUESTED_MAX_ACCELERATION_RAD_S2
        request.blend_radius = 0.1
        request.is_cartesian = False

        future = self._client.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if future.done():
                break
        if not future.done():
            raise RuntimeError("runtime_error:generate_s_trajectory timed out")

        response = future.result()
        if response is None or not response.success:
            message = getattr(response, "error_msg", "") if response is not None else "no response"
            raise RuntimeError(f"runtime_error:{message}")

        points = [list(point.pos) for point in response.trajectory_points]
        return points, float(response.total_time), list(request.start_joint_pos), list(request.target_joint_pos)


def max_abs(values: list[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil((pct / 100.0) * len(ordered)) - 1))
    return ordered[index]


def validate_points(points: list[list[float]]) -> None:
    if len(points) < THRESHOLDS["min_point_count"]:
        raise ValueError("trajectory_empty")
    for point_index, point in enumerate(points):
        if len(point) != 6:
            raise ValueError(f"nonfinite_sample:point_width[{point_index}]={len(point)}")
        for axis, value in enumerate(point):
            if not math.isfinite(value):
                raise ValueError(f"nonfinite_sample:point[{point_index}][{axis}]")


def compute_metrics(
    points: list[list[float]],
    total_time: float,
    start: list[float],
    target: list[float],
) -> SmoothnessMetrics:
    validate_points(points)
    if not math.isfinite(total_time) or total_time <= 0.0:
        raise ValueError("runtime_error:invalid total_time")

    sample_dt = total_time / float(len(points) - 1)
    if not math.isfinite(sample_dt) or sample_dt <= 0.0:
        raise ValueError("runtime_error:invalid sample_dt")

    endpoint_errors = [
        abs(points[0][axis] - start[axis]) for axis in range(6)
    ] + [
        abs(points[-1][axis] - target[axis]) for axis in range(6)
    ]

    position_steps: list[list[float]] = []
    velocity_estimates: list[list[float]] = []
    for prev, cur in zip(points, points[1:]):
        delta = [cur[axis] - prev[axis] for axis in range(6)]
        position_steps.append(delta)
        velocity_estimates.append([value / sample_dt for value in delta])

    acceleration_estimates: list[list[float]] = []
    for prev, cur in zip(velocity_estimates, velocity_estimates[1:]):
        acceleration_estimates.append([(cur[axis] - prev[axis]) / sample_dt for axis in range(6)])

    jerk_estimates: list[list[float]] = []
    for prev, cur in zip(acceleration_estimates, acceleration_estimates[1:]):
        jerk_estimates.append([(cur[axis] - prev[axis]) / sample_dt for axis in range(6)])

    jerk_abs = [abs(value) for sample in jerk_estimates for value in sample]
    return SmoothnessMetrics(
        point_count=len(points),
        sample_dt_ms=sample_dt * 1000.0,
        max_endpoint_error_rad=max(endpoint_errors, default=math.inf),
        max_velocity_est_rad_s=max((max_abs(sample) for sample in velocity_estimates), default=0.0),
        max_acceleration_est_rad_s2=max((max_abs(sample) for sample in acceleration_estimates), default=0.0),
        jerk_p99_est_rad_s3=percentile(jerk_abs, 99.0),
        jerk_max_est_rad_s3=max(jerk_abs, default=0.0),
        max_position_step_rad=max((max_abs(sample) for sample in position_steps), default=0.0),
    )


def classify(metrics: SmoothnessMetrics | None, error: str | None = None) -> str:
    if error is not None:
        if error.startswith("service_missing"):
            return "service_missing"
        if error.startswith("trajectory_empty"):
            return "trajectory_empty"
        if error.startswith("nonfinite_sample"):
            return "nonfinite_sample"
        return "runtime_error"
    if metrics is None or metrics.point_count < THRESHOLDS["min_point_count"]:
        return "trajectory_empty"
    if metrics.max_endpoint_error_rad > THRESHOLDS["max_endpoint_error_rad"]:
        return "endpoint_mismatch"
    if metrics.max_velocity_est_rad_s > REQUESTED_MAX_VELOCITY_RAD_S * THRESHOLDS["velocity_limit_scale"]:
        return "velocity_limit_regression"
    if metrics.max_acceleration_est_rad_s2 > REQUESTED_MAX_ACCELERATION_RAD_S2 * THRESHOLDS["acceleration_limit_scale"]:
        return "acceleration_limit_regression"
    if (
        metrics.jerk_p99_est_rad_s3 > THRESHOLDS["jerk_p99_est_rad_s3"]
        or metrics.jerk_max_est_rad_s3 > THRESHOLDS["jerk_max_est_rad_s3"]
    ):
        return "jerk_spike"
    return "pass"


def print_result(mode: str, metrics: SmoothnessMetrics | None, classification: str, error: str = "") -> None:
    payload: dict[str, object] = {
        "mode": mode,
        "classification": classification,
        "requested_max_velocity_rad_s": REQUESTED_MAX_VELOCITY_RAD_S,
        "requested_max_acceleration_rad_s2": REQUESTED_MAX_ACCELERATION_RAD_S2,
        "thresholds": THRESHOLDS,
        "error": error,
    }
    if metrics is not None:
        payload.update(
            {
                key: round(value, 6) if isinstance(value, float) else value
                for key, value in metrics.as_dict().items()
            }
        )
    print("NRT_PATH_SMOOTHNESS_JSON " + json.dumps(payload, sort_keys=True))
    if classification == "pass":
        assert metrics is not None
        print(
            "nrt_path_smoothness: PASS "
            f"mode={mode} point_count={metrics.point_count} "
            f"sample_dt_ms={metrics.sample_dt_ms:.3f} "
            f"vel_est={metrics.max_velocity_est_rad_s:.3f} "
            f"acc_est={metrics.max_acceleration_est_rad_s2:.3f} "
            f"jerk_p99_est={metrics.jerk_p99_est_rad_s3:.3f} "
            "classification=pass"
        )
    else:
        print(
            "nrt_path_smoothness: FAIL "
            f"mode={mode} classification={classification} error={error}",
            file=sys.stderr,
        )


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("headless", "gazebo"), required=True)
    parser.add_argument("--namespace", default="/xmate_er3")
    args = parser.parse_args(argv)

    rclpy.init()
    probe = NrtPathSmoothnessProbe(args.namespace)
    metrics: SmoothnessMetrics | None = None
    try:
        probe.wait_for_ready(45.0)
        points, total_time, start, target = probe.call_generate_s_trajectory()
        metrics = compute_metrics(points, total_time, start, target)
        classification = classify(metrics)
        print_result(args.mode, metrics, classification)
        return 0 if classification == "pass" else 1
    except ValueError as exc:
        error = str(exc)
        classification = classify(metrics, error)
        print_result(args.mode, metrics, classification, error)
        return 1
    except Exception as exc:
        error = str(exc)
        classification = classify(metrics, error)
        print_result(args.mode, metrics, classification, error)
        return 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
