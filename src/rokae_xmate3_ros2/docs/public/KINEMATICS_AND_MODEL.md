# Kinematics and Model

Status: Active
Audience: model users, kinematics maintainers, acceptance reviewers
Purpose: current xMateER3 simulation-grade model policy and traceability

## 1. Scope

This page describes the xMateER3 six-axis public lane model behavior. It does not document calibration workflows or claim controller-model parity.

## 2. Source-of-Truth Split

The package maintains:

- joint names and limits
- velocity, acceleration, and jerk envelopes
- direct torque envelopes
- manual-style `official_dh`
- helper-style `improved_dh`

## 3. Runtime Policy

- primary backend: KDL
- auxiliary fallback: improved_dh
- Jacobian mode: native backend Jacobian when available
- single request must use exactly one primary backend

The fallback backend is allowed for seed generation, primary-failure retry, and regression comparison. It must not silently mix results inside one request.

## 4. Consumers

The model policy feeds URDF limits, runtime validators, FK/IK/Jacobian requests, soft-limit defaults, retimer envelopes, and SDK facade model calls.

## 5. Fidelity

- kinematics: simulation-grade
- Jacobian: simulation-grade
- dynamics and wrench mapping: approximate

The public lane is a simulation compatibility layer. It keeps interfaces continuous but does not claim controller-grade parity.

## 6. Traceability

- canonical description artifact: `<build>/generated/urdf/xMateER3.urdf`
- canonical description metadata: `<build>/generated/urdf/xMateER3.description.json`
- install-time copies: `share/rokae_xmate3_ros2/generated/urdf/*`
- diagnostics expose `model_revision`, `model_primary_backend`, and exactness summary fields

Non-canonical xacro/model overrides require `allow_noncanonical_model:=true`.

## 7. Public Model Facade Boundary

`include/rokae_xmate3_ros2/model_facade.hpp` owns its public wrapper types (`ModelFacade`, `ModelLoadContext`, `ModelDynamicsBreakdown`, `ModelDiagnostics`) and consumes the runtime `kinematics::Provider` boundary directly.

`include/rokae_xmate3_ros2/model.hpp` keeps concrete provider ownership behind `XMateModel::Impl`. Public headers must not include `runtime/kinematics_provider.hpp`, name `OwnedGazeboProvider`, or expose Gazebo symbols.

The public model facade includes only `runtime/kinematics_provider_interface.hpp`. Concrete simulation providers, including the Gazebo-backed provider, remain behind implementation-side headers or compiled translation units.

Related docs: [COMPATIBILITY.md](COMPATIBILITY.md), [../architecture/ARCHITECTURE.md](../architecture/ARCHITECTURE.md), [../architecture/PROVIDER_BOUNDARY.md](../architecture/PROVIDER_BOUNDARY.md).
