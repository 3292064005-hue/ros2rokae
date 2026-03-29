# Hardening Backlog

This backlog is intentionally short and ordered. It is the set of remaining items most likely to prevent future rework.

## P0 — Keep the current correct skeleton fixed

Do not revert these:
- runtime as the only state authority
- RT/NRT split profiles
- KDL primary backend + improved DH auxiliary backend
- preferred vs legacy contract split
- per-interface alignment documentation

## P1 — Enforce single-primary-backend requests

Goal:
- one FK / IK / Jacobian / planner inverse-projection request uses exactly one primary backend

Needed work:
- request-level tracing in kinematics
- request-level tests proving no silent backend mixing
- planner tests that bind continuity scoring to the same primary backend used for solving

## P2 — Planner preflight reporting

Goal:
- expose a first-class report before execution

Needed work:
- reachable / singularity / continuity / branch-switch / fallback notes
- a stable report object or service-level payload
- tests for failure reasons and fallback notes

## P3 — RT subsystem hardening

Goal:
- turn the simulated RT facade into a stronger subsystem

Needed work:
- RT field registry
- RT subscription plan
- watchdog / pre-arm checks
- richer loop statistics and diagnostics tests

## P4 — Runtime catalog normalization

Goal:
- normalize RL / tool / wobj runtime truth

Needed work:
- explicit runtime catalog models
- tests for preferred and legacy wrappers
- docs showing runtime as the only truth source

## P5 — Contract and docs lock-in

Goal:
- every stronger alignment claim is test-backed

Needed work:
- more contract tests
- no chapter-level overclaiming in README
- every preferred/legacy pair documented in one place
