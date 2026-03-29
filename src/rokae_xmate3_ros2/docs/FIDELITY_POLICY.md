# Fidelity Policy

The project distinguishes between:

- **StrictAligned**: public shape and runtime semantics intentionally match the manual.
- **SimApprox**: public surface matches, but the implementation is simulation-grade.
- **Experimental**: useful, but not stable enough to claim parity.

## Kinematics-specific policy

Kinematics now follows a stricter rule than before:

- the public API exposes one `xMate3Kinematics` contract
- the default primary backend is `KDL/URDF`
- the improved-DH backend is retained for seed generation, regression comparison, and fallback
- a single request never mixes primary backends mid-solve

This avoids the previous ambiguity where different steps of the same solve could silently traverse different geometry models.
