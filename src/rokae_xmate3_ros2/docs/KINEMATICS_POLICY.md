# Kinematics Policy

This package uses a policy-driven kinematics stack.

## Default policy
- Primary backend: `KDL`
- Fallback backend: `improved_dh`
- Jacobian mode: native backend Jacobian when available
- IK seed mode: mixed (`current_state + improved_dh branch hints`)

## Hard rule
A single request must use exactly one primary backend for FK / Jacobian / IK scoring.
The fallback backend is only allowed for seed generation, retry on primary failure, and regression comparisons.

## Environment overrides
- `ROKAE_KINEMATICS_PRIMARY=kdl|improved_dh`
- `ROKAE_KINEMATICS_FALLBACK=improved_dh|none`
- `ROKAE_KINEMATICS_IK_SEEDS=current|dh|mixed`

## Contract enforcement
- `xMate3Kinematics::beginRequestContract()` / `requestContractState()` lock the primary backend for one logical request.
- If a request observes a fallback-marked IK selection note, the request is rejected as `backend_contract_violation` instead of silently mixing semantics.
