from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict


def _resolve_policy_file() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    share_root = os.path.dirname(here)
    return os.path.join(share_root, "config", "default_runtime_host_policy.env")


def _read_policy() -> Dict[str, str]:
    policy: Dict[str, str] = {}
    policy_file = _resolve_policy_file()
    if not os.path.isfile(policy_file):
        return policy
    with open(policy_file, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            policy[key.strip()] = value.strip()
    return policy


_POLICY = _read_policy()
_DEFAULT_PROFILE_NAME = _POLICY.get("ROKAE_DEFAULT_PUBLIC_LAUNCH_PROFILE", "public_xmate_er3_sdk")
_DEFAULT_RUNTIME_HOST = _POLICY.get("ROKAE_DEFAULT_RUNTIME_HOST", "daemonized_runtime")
_DEFAULT_RUNTIME_PROFILE = _POLICY.get("ROKAE_DEFAULT_RUNTIME_PROFILE", "nrt_strict_parity")
_DEFAULT_BACKEND_MODE = _POLICY.get("ROKAE_DEFAULT_BACKEND_MODE", "effort")
_DEFAULT_SERVICE_EXPOSURE_PROFILE = _POLICY.get("ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE", "public_xmate_er3_only")
_DEFAULT_ENABLE_ROS2_CONTROL = _POLICY.get("ROKAE_DEFAULT_ENABLE_ROS2_CONTROL", "false")
_DEFAULT_ENABLE_XCORE_PLUGIN = _POLICY.get("ROKAE_DEFAULT_ENABLE_XCORE_PLUGIN", "false")
_DEFAULT_COMPATIBILITY_ALIAS_POLICY = _POLICY.get("ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY", "canonical_plus_compat")


@dataclass(frozen=True)
class LaunchProfile:
    name: str
    backend_mode: str
    service_exposure_profile: str
    runtime_profile: str
    enable_ros2_control: str
    enable_xcore_plugin: str
    runtime_host: str
    compatibility_alias_policy: str


_PROFILES = {
    "public_xmate_er3_sdk": LaunchProfile(
        "public_xmate_er3_sdk",
        _DEFAULT_BACKEND_MODE,
        _DEFAULT_SERVICE_EXPOSURE_PROFILE,
        _DEFAULT_RUNTIME_PROFILE,
        _DEFAULT_ENABLE_ROS2_CONTROL,
        _DEFAULT_ENABLE_XCORE_PLUGIN,
        _DEFAULT_RUNTIME_HOST,
        _DEFAULT_COMPATIBILITY_ALIAS_POLICY,
    ),
    "public_xmate_er3_jtc": LaunchProfile("public_xmate_er3_jtc", "jtc", "public_xmate_er3_only", "nrt_strict_parity", "true", "true", "gazebo_plugin", "canonical_plus_compat"),
    "internal_full_hybrid": LaunchProfile("internal_full_hybrid", "hybrid", "internal_full", "hybrid_bridge", "true", "true", "gazebo_plugin", "canonical_plus_compat"),
    "daemon_hard_rt": LaunchProfile("daemon_hard_rt", "effort", "internal_full", "hard_1khz", "false", "false", "daemonized_runtime", "canonical_plus_compat"),
}


def default_launch_profile_name() -> str:
    return _DEFAULT_PROFILE_NAME if _DEFAULT_PROFILE_NAME in _PROFILES else "public_xmate_er3_sdk"


def profile_names():
    return tuple(_PROFILES.keys())


def is_valid_launch_profile(name: str) -> bool:
    return name in _PROFILES


def resolve_launch_profile(name: str) -> LaunchProfile:
    if not is_valid_launch_profile(name):
        allowed = ', '.join(profile_names())
        raise ValueError(f"unknown launch_profile '{name}'; allowed: {allowed}")
    return _PROFILES[name]
