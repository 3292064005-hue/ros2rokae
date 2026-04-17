from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchProfile:
    name: str
    backend_mode: str
    service_exposure_profile: str
    runtime_profile: str
    enable_ros2_control: str
    enable_xcore_plugin: str
    runtime_host: str


_PROFILES = {
    "public_xmate6_jtc": LaunchProfile("public_xmate6_jtc", "jtc", "public_xmate6_only", "nrt_strict_parity", "true", "true", "gazebo_plugin"),
    "internal_full_hybrid": LaunchProfile("internal_full_hybrid", "hybrid", "internal_full", "hybrid_bridge", "true", "true", "gazebo_plugin"),
    "daemon_hard_rt": LaunchProfile("daemon_hard_rt", "effort", "internal_full", "hard_1khz", "false", "false", "daemonized_runtime"),
}


def default_launch_profile_name() -> str:
    return "public_xmate6_jtc"


def profile_names():
    return tuple(_PROFILES.keys())


def is_valid_launch_profile(name: str) -> bool:
    return name in _PROFILES


def resolve_launch_profile(name: str) -> LaunchProfile:
    if not is_valid_launch_profile(name):
        allowed = ', '.join(profile_names())
        raise ValueError(f"unknown launch_profile '{name}'; allowed: {allowed}")
    return _PROFILES[name]
