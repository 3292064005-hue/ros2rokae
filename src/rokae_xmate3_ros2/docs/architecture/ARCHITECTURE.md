# Architecture

Status: Active
Audience: runtime and SDK maintainers
Purpose: high-level package identity, runtime authority, target layering, and extension boundaries

## Platform Identity

This repository is the xMateER3 public SDK compatibility lane. It keeps the historical ROS package name `rokae_xmate3_ros2` for compatibility, but the current public identity is the xMateER3 six-axis lane and the install-facing `xCoreSDK` surface.

It is not a generic multi-robot SDK, not a controller-grade hardware RT proof, and not a public RL/IO/calibration documentation set.

## Runtime Layers

1. public SDK facade: `include/rokae/*`, `xCoreSDK::*`
2. runtime services, actions, queries, and diagnostics
3. request coordinator, motion runtime, and state authority
4. backend provider and simulation/runtime host

Runtime is the only state authority. Public query and topic paths must flow through runtime/coordinator authority instead of raw fetchers.

## Target Layering Contract

The repository stays as one ROS 2 package, but build and install surfaces are frozen by target role:

- `rokae_xmate3_ros2_runtime_motion_core`
- `rokae_xmate3_ros2_runtime_state`
- `rokae_xmate3_ros2_runtime_facade`
- `rokae_xmate3_ros2_runtime_ros_bridge`
- `rokae_xmate3_ros2_runtime_control_bridge`
- `rokae_xmate3_ros2_runtime_core`
- `rokae_xmate3_ros2_runtime_test`
- `xCoreSDK_static`
- `xCoreSDK_shared`
- `xCoreSDK_core`
- `xCoreSDK_ros_bridge`

Dependency direction is fixed:

- motion/state/facade/ros_bridge/control_bridge feed the runtime assembly.
- `runtime_core` and `runtime_test` consume shared runtime assembly only.
- `xCoreSDK_static` and `xCoreSDK_shared` consume compat facade, runtime object layers, and SDK backend objects.
- `xCoreSDK_core` is the SDK-shaped minimum consumer surface; it is not an empty INTERFACE shell.
- `xCoreSDK_ros_bridge` is the explicit install-facing ROS 2 runtime/action/service bridge.
- public ABI exports `include/rokae/*` and `xCoreSDK::*`; backend/internal headers stay gated.

## Design Rules

- Preferred public surface is `/xmate_er3/*`; legacy `/xmate3/*` names require explicit compatibility alias policy.
- RT and NRT stay split. Gazebo RT paths are simulation-grade and are not controller-grade parity claims.
- A kinematics request must use one primary backend. KDL is primary; `improved_dh` is auxiliary.
- New motion/runtime extensions must pass the `motion_extension_contract` fail-fast checks before registration.
- Backend resolution uses the single provider chain described in [PROVIDER_BOUNDARY.md](PROVIDER_BOUNDARY.md): `RuntimeBackendProviderHost -> RuntimeBackendProvider -> BackendInterface`.
- Backend factories use one generic backend factory request shape, not host-specific factory entrypoints.

## Allowed Extension Domains

- xMateER3 public lane docs and compat facade
- runtime query authority
- diagnostics, catalog, and host lifecycle
- acceptance/reporting gates

Do not extend the default public lane with RL, IO, calibration, or experimental RT examples. Those remain explicit experimental/internal surfaces.

## Related Docs

- [PROVIDER_BOUNDARY.md](PROVIDER_BOUNDARY.md)
- [../public/RUNTIME_PROFILES.md](../public/RUNTIME_PROFILES.md)
- [../public/KINEMATICS_AND_MODEL.md](../public/KINEMATICS_AND_MODEL.md)
- [../release/ACCEPTANCE_LAYERS.md](../release/ACCEPTANCE_LAYERS.md)
