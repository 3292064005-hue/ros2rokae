# Extension Framework

This document freezes the long-term expansion plan so the package can continue to grow without re-breaking its architecture.

## Platform identity

Treat this repository as a **simulation-grade xCore SDK platform for xMate3**, not as a loose collection of ROS2 services.

That means every new feature must attach to one of six layers:

1. **Public API** – SDK-compatible headers and the ROS2 wrapper surface
2. **Contract** – message/service/action schemas and alignment docs
3. **Runtime** – the single source of truth for modes, ownership, diagnostics, RL state, tool/wobj state
4. **Kinematics** – FK, IK, Jacobian, singularity, wrench mapping, planner inverse projection
5. **Simulation / Control** – Gazebo + ros2_control profile wiring
6. **Verification** – unit, contract, strict, and harness tests

## Immutable design rules

### 1. Runtime is the only state authority

No new feature may invent a second truth in the wrapper or shim layer.
Any persistent motion mode, RT mode, RL option, active profile, tool catalog, or wobj catalog state must live in runtime.

### 2. Main contracts and legacy contracts must be explicit

Every domain keeps a preferred contract and an optional compatibility facade.
Examples:

- `ReadRegisterEx` / `WriteRegisterEx` are the preferred register contracts.
- `ReadRegister` / `WriteRegister` are compatibility facades.
- `GetEndWrench` is the preferred force/wrench contract.
- `GetEndEffectorTorque` is the compatibility facade.

### 3. Kinematics uses one primary backend per request

The default production path is:

- primary backend: `KDL`
- auxiliary backend: `improved_dh`
- finite-difference: test oracle only

A single FK / IK / Jacobian / inverse-projection request must not switch primary backends mid-solve.

### 4. RT and NRT remain permanently split

- `config/ros2_control_nrt.yaml` is the queued non-realtime profile.
- `config/ros2_control_rt.yaml` is the simulated realtime facade profile.

Do not merge them back into one control profile.

## Expansion domains

### Runtime observability

Next capabilities should land here:

- runtime trace topic
- motion session ID
- active planner name
- active kinematics backend
- fallback-trigger reason
- singularity warnings / degradation cause
- RT watchdog summary
- queue depth metrics

### Kinematics and model

Continue growing:

- manipulability / singularity grading
- improved seed pool management
- tool / payload context-aware scoring
- branch continuity scoring
- gravity / dynamics approximations exposed via model facade

### Planner

The next high-value addition is a **trajectory preflight validator** that reports:

- reachability
- chosen backend
- chosen IK branch
- continuity risk
- singularity risk
- joint-limit risk
- retimer family

### RT facade

Expand into a full subsystem with:

- field registry
- subscription plan
- loop statistics
- watchdog
- pre-arm checks
- mode capability matrix

### Communication / RL / cobot semantics

Stabilize these runtime domains:

- register namespaces and indexed references
- RL project/task catalogs
- tool and wobj descriptor catalogs
- wrench / external joint torque sampling semantics
- xPanel state as an explicit runtime domain instead of an isolated knob

## Release gate for new features

A feature is not “done” until all of the following are true:

1. it is attached to the correct layer above
2. its preferred vs legacy contract status is documented
3. runtime truth is clearly defined
4. diagnostics or trace visibility exists if the feature is stateful
5. at least one contract or unit test proves the claim

## Self-check questions

Before merging any future feature, ask:

- Did I attach this feature to the right layer, or did I smuggle logic into the shim?
- Does runtime remain the only source of truth?
- Did I define a preferred contract and a compatibility contract, or is the surface ambiguous?
- Can diagnostics explain the feature when it fails?
- Is there a test proving the alignment claim?
