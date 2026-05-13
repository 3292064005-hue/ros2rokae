# Runtime Profiles

Status: Active
Audience: users who choose launch profiles, runtime maintainers, acceptance owners
Purpose: profile choice, RT/NRT boundary, query authority, and readiness behavior

## 1. Profile Choices

| profile | Use | Surface |
|---|---|---|
| `public_xmate_er3_jtc` | default Gazebo/JTC simulation main chain | public |
| `public_xmate_er3_headless_sdk_smoke` | SDK smoke without Gazebo GUI workload | public smoke |
| `public_xmate_er3_experimental_rt` | explicit RT/drag/path opt-in | experimental |
| `internal_full_hybrid` | full internal mixed validation | internal |
| `daemon_hard_rt` | daemon-owned runtime validation | internal |

`launch_profile` is fail-fast: unknown values are errors. Default profile, backend mode, runtime host, and service exposure come from `config/default_runtime_host_policy.env`.

## 2. Public Readiness

`public_xmate_er3_jtc` defaults to `require_runtime_readiness:=true`. Launch waits for:

- controller manager service
- active `joint_trajectory_controller`
- `/joint_trajectory_controller/follow_joint_trajectory` action server

Disable readiness only for show-args, graphics, or dependency debugging.

`run_main_chain_smoke.sh` exercises the default JTC profile end to end. `run_headless_sdk_smoke.sh` reuses the same public service/action checks with `public_xmate_er3_headless_sdk_smoke`, proving SDK-facing behavior without Gazebo GUI/JTC.

## 3. RT / NRT Split

- NRT is the default public main chain.
- RT ROS services are not registered by the default public profile.
- `public_xmate_er3_experimental_rt` is opt-in and is not public release proof.
- `run_experimental_opt_in_smoke.sh` only checks explicit registration and minimum simulation-grade behavior for RT/drag/path services.
- strict 1kHz fail-fast RT profile remains internal/runtime lane.
- The only 1kHz simulation gate is `tools/run_rt_1khz_stress.sh <workspace-root> 60 5 daemon`, which uses `daemon_hard_rt` semantics: daemonized headless runtime, `hard_1khz`, `internal_full`, `canonical_plus_compat`, strict scheduler/memlock, and `shm_only` ingress.
- Passing the 1kHz gate requires the stress metrics and runtime diagnostics to agree: `avg_hz >= 995`, `p95_ms <= 1.05`, `p99_ms <= 1.20`, active RT scheduler, `shm_ring` transport, zero deadline misses, and max RT gap no greater than 1.2 ms.
- `mode=simulation` on the stress script is diagnostic-only and must not be used as 1kHz proof.
- Gazebo RT semantics are simulation-grade, not controller-grade hardware parity.

## 4. Query Authority

Runtime/coordinator is the only public state authority.

Public queries and publishers should derive from `MotionRequestCoordinator -> MotionRuntime::readAuthoritativeSnapshot()`. Diagnostics must report `query_authority=runtime_request_coordinator` when using this path.

Planner preflight is separated from legacy `ValidateMotion` through the internal `PlannerPreflightReport` service-level payload, with reachable, singularity, continuity, branch-switch, fallback notes, and reject reasons in one report object.

## 5. Operational Summary

- Main chain: `MoveReset -> MoveAppend -> MoveStart -> Stop(pause)`.
- Path replay is an experimental side-lane and does not occupy the default staged MoveAppend queue contract.
- Profile capability and diagnostics banners must show backend, authority, exposure policy, `authority_scope`, `fidelity_class`, and `model_revision`.
- Public lane does not promise IO, RL, or calibration.
- Observability remains runtime-owned.

## 6. Provider Boundary

`simulation.launch.py`, `xmate_er3_public.launch.py`, daemon runtime, and Gazebo runtime share the `runtime_host_builder` bootstrap and assembly contract.

The provider/backend chain is documented in [../architecture/PROVIDER_BOUNDARY.md](../architecture/PROVIDER_BOUNDARY.md). It replaces the old split pages for RT profile guide, hardening profile, capability matrix, query policy, and catalog policy.

`compatibility_alias_policy` accepts `canonical_plus_compat`, `canonical_only`, or `legacy_only`; the default is `canonical_only`.

Related docs: [../architecture/ARCHITECTURE.md](../architecture/ARCHITECTURE.md), [../release/RELEASE_GATE.md](../release/RELEASE_GATE.md), [../reference/CAPABILITY_MATRIX.md](../reference/CAPABILITY_MATRIX.md).
