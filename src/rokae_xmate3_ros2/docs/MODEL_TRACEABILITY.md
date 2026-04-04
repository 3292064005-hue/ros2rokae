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

## Exactness grades

The diagnostics surface now publishes a compact model exactness summary. Current intent is:

- kinematics: simulation-grade
- jacobian: simulation-grade
- dynamics / wrench mapping: approximate

This keeps API surface continuity without overstating controller-model parity.

## Canonical description artifact

- Build-time canonical artifact: `<build>/generated/urdf/xMate3.urdf`
- Build-time provenance metadata: `<build>/generated/urdf/xMate3.description.json`
- Installed default artifact: `share/rokae_xmate3_ros2/generated/urdf/xMate3.urdf`
- Installed provenance metadata: `share/rokae_xmate3_ros2/generated/urdf/xMate3.description.json`

`tools/generate_description_metadata.py` records the source xacro path, expansion arguments, file size, and SHA-256 digest so launch-time robot_description expansion and model-loading paths can be audited against the same canonical description contract. Explicit xacro/model overrides are now treated as developer-mode only and require `allow_noncanonical_model:=true`.
