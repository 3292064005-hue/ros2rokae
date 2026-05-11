from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, Tuple


def _resolve_policy_file() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    share_root = os.path.dirname(here)
    return os.path.join(share_root, "config", "default_runtime_host_policy.env")


def _resolve_capability_matrix_file() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    share_root = os.path.dirname(here)
    return os.path.join(share_root, "config", "xmate_er3_capability_matrix.json")


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


def _read_capability_matrix() -> Dict[str, object]:
    matrix_file = _resolve_capability_matrix_file()
    if not os.path.isfile(matrix_file):
        return {}
    with open(matrix_file, "r", encoding="utf-8") as handle:
        return json.load(handle)


_POLICY = _read_policy()
_CAPABILITY_MATRIX = _read_capability_matrix()


def capability_matrix() -> Dict[str, object]:
    return _CAPABILITY_MATRIX


def _matrix_default(key: str, fallback: str) -> str:
    value = _CAPABILITY_MATRIX.get(key, fallback)
    return str(value) if value is not None else fallback


_DEFAULT_PROFILE_NAME = _POLICY.get(
    "ROKAE_DEFAULT_PUBLIC_LAUNCH_PROFILE",
    _matrix_default("default_launch_profile", "public_xmate_er3_jtc"),
)
_DEFAULT_RUNTIME_HOST = _POLICY.get("ROKAE_DEFAULT_RUNTIME_HOST", "gazebo_plugin")
_DEFAULT_RUNTIME_PROFILE = _POLICY.get("ROKAE_DEFAULT_RUNTIME_PROFILE", "nrt_strict_parity")
_DEFAULT_BACKEND_MODE = _POLICY.get("ROKAE_DEFAULT_BACKEND_MODE", "jtc")
_DEFAULT_SERVICE_EXPOSURE_PROFILE = _POLICY.get(
    "ROKAE_DEFAULT_SERVICE_EXPOSURE_PROFILE",
    _matrix_default("default_service_exposure_profile", "public_xmate_er3_only"),
)
_DEFAULT_ENABLE_ROS2_CONTROL = _POLICY.get("ROKAE_DEFAULT_ENABLE_ROS2_CONTROL", "true")
_DEFAULT_ENABLE_XCORE_PLUGIN = _POLICY.get("ROKAE_DEFAULT_ENABLE_XCORE_PLUGIN", "true")
_DEFAULT_COMPATIBILITY_ALIAS_POLICY = _POLICY.get(
    "ROKAE_DEFAULT_COMPATIBILITY_ALIAS_POLICY",
    _matrix_default("default_compatibility_alias_policy", "canonical_only"),
)


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
    required_capabilities: Tuple[str, ...] = ()


_BASE_PROFILES = {
    "public_xmate_er3_jtc": LaunchProfile(
        "public_xmate_er3_jtc",
        _DEFAULT_BACKEND_MODE,
        _DEFAULT_SERVICE_EXPOSURE_PROFILE,
        _DEFAULT_RUNTIME_PROFILE,
        _DEFAULT_ENABLE_ROS2_CONTROL,
        _DEFAULT_ENABLE_XCORE_PLUGIN,
        _DEFAULT_RUNTIME_HOST,
        _DEFAULT_COMPATIBILITY_ALIAS_POLICY,
        ("state_read", "trajectory_execution", "gazebo_physics"),
    ),
    "public_xmate_er3_headless_sdk_smoke": LaunchProfile(
        "public_xmate_er3_headless_sdk_smoke",
        "headless_sim",
        "public_xmate_er3_only",
        "nrt_strict_parity",
        "false",
        "false",
        "daemonized_runtime",
        "canonical_only",
        ("state_read", "effort_execution"),
    ),
    "public_xmate_er3_experimental_rt": LaunchProfile(
        "public_xmate_er3_experimental_rt",
        "hybrid",
        "public_xmate_er3_experimental",
        "rt_sim_experimental_best_effort",
        "true",
        "true",
        "gazebo_plugin",
        "canonical_only",
        ("state_read", "trajectory_execution", "effort_execution", "gazebo_physics", "rt_direct_command"),
    ),
    "public_xmate_er3_sdk": LaunchProfile(
        "public_xmate_er3_sdk",
        "headless_sim",
        "public_xmate_er3_only",
        "nrt_strict_parity",
        "false",
        "false",
        "daemonized_runtime",
        "canonical_only",
        ("state_read", "effort_execution"),
    ),
    "internal_full_hybrid": LaunchProfile(
        "internal_full_hybrid",
        "hybrid",
        "internal_full",
        "hybrid_bridge",
        "true",
        "true",
        "gazebo_plugin",
        "canonical_plus_compat",
        ("state_read", "trajectory_execution", "effort_execution", "gazebo_physics", "rt_direct_command"),
    ),
    "daemon_hard_rt": LaunchProfile(
        "daemon_hard_rt",
        "headless_sim",
        "internal_full",
        "hard_1khz",
        "false",
        "false",
        "daemonized_runtime",
        "canonical_plus_compat",
        ("state_read", "effort_execution", "rt_direct_command"),
    ),
}


def _profile_from_matrix(name: str, fallback: LaunchProfile) -> LaunchProfile:
    profile_specs = _CAPABILITY_MATRIX.get("launch_profiles", {})
    if not isinstance(profile_specs, dict):
        return fallback
    spec = profile_specs.get(name, {})
    if not isinstance(spec, dict):
        return fallback
    required = spec.get("required_capabilities", list(fallback.required_capabilities))
    if not isinstance(required, list):
        required = list(fallback.required_capabilities)
    return LaunchProfile(
        name=name,
        backend_mode=str(spec.get("backend_mode", fallback.backend_mode)),
        service_exposure_profile=str(spec.get("service_exposure_profile", fallback.service_exposure_profile)),
        runtime_profile=str(spec.get("runtime_profile", fallback.runtime_profile)),
        enable_ros2_control=str(spec.get("enable_ros2_control", fallback.enable_ros2_control)).lower(),
        enable_xcore_plugin=str(spec.get("enable_xcore_plugin", fallback.enable_xcore_plugin)).lower(),
        runtime_host=str(spec.get("runtime_host", fallback.runtime_host)),
        compatibility_alias_policy=str(spec.get("compatibility_alias_policy", fallback.compatibility_alias_policy)),
        required_capabilities=tuple(str(item) for item in required),
    )


_PROFILES = {name: _profile_from_matrix(name, profile) for name, profile in _BASE_PROFILES.items()}


def default_launch_profile_name() -> str:
    return _DEFAULT_PROFILE_NAME if _DEFAULT_PROFILE_NAME in _PROFILES else "public_xmate_er3_jtc"


def profile_names():
    return tuple(_PROFILES.keys())


def is_valid_launch_profile(name: str) -> bool:
    return name in _PROFILES


def resolve_launch_profile(name: str) -> LaunchProfile:
    if not is_valid_launch_profile(name):
        allowed = ', '.join(profile_names())
        raise ValueError(f"unknown launch_profile '{name}'; allowed: {allowed}")
    return _PROFILES[name]


def backend_mode_requirements(backend_mode: str) -> Tuple[str, ...]:
    backend_modes = _CAPABILITY_MATRIX.get("backend_modes", {})
    if not isinstance(backend_modes, dict):
        return ()
    spec = backend_modes.get(backend_mode, {})
    if not isinstance(spec, dict):
        return ()
    required = spec.get("required_capabilities", [])
    if not isinstance(required, list):
        return ()
    return tuple(str(item) for item in required)


def service_exposure_profiles() -> Tuple[str, ...]:
    profiles = _CAPABILITY_MATRIX.get("service_exposure_profiles", {})
    if not isinstance(profiles, dict):
        return ("public_xmate_er3_only", "public_xmate_er3_experimental", "internal_full")
    return tuple(str(name) for name in profiles.keys())
