# Model Traceability

The package centralizes xMate3 constants in `include/rokae_xmate3_ros2/spec/xmate3_spec.hpp`.

## Source-of-truth split
- joint limits
- velocity / acceleration / jerk envelopes
- direct torque envelopes
- the manual-table DH representation (`official_dh`)
- the improved DH helper model with explicit joint offsets (`improved_dh`)

## Runtime policy
The runtime no longer treats every kinematics implementation as equivalent.

- **Primary backend (default):** `KDL/URDF`
- **Auxiliary / fallback backend:** `improved_dh`
- **Single-request rule:** one FK / Jacobian / IK / trajectory request stays on a single primary backend from start to finish

This keeps Gazebo / URDF consistency on the main path while still preserving the improved-DH branch logic as a deterministic seed and fallback model.

## Consumers
- URDF joint limits and velocity caps
- kinematics backends
- soft-limit defaults
- joint retimer envelopes
- runtime validators and plugin fallback limits
- SDK facade model operations
