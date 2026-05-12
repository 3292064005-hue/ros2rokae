# Documentation Index

Status: Active
Audience: users, SDK integrators, runtime maintainers, release/acceptance owners
Purpose: the single entry point for current documentation

## Fast Path

1. Run the package: [public/QUICKSTART.md](public/QUICKSTART.md)
2. Run public examples: [public/EXAMPLES.md](public/EXAMPLES.md)
3. Verify JTC/headless/experimental smoke: [public/QUICKSTART.md](public/QUICKSTART.md#verify)
4. Verify the environment and gates: [release/BUILD_RELEASE.md](release/BUILD_RELEASE.md)
5. Diagnose locked-target failures: [release/ENVIRONMENT_LOCK.md](release/ENVIRONMENT_LOCK.md)

## Public SDK / User Docs

- [public/COMPATIBILITY.md](public/COMPATIBILITY.md): public lane boundary, unsupported areas, experimental opt-in.
- [public/RUNTIME_PROFILES.md](public/RUNTIME_PROFILES.md): launch profiles, RT/NRT split, query authority.
- [public/KINEMATICS_AND_MODEL.md](public/KINEMATICS_AND_MODEL.md): simulation-grade model, backend policy, generated description provenance.
- [public/PUBLIC_SDK_ARTIFACT.md](public/PUBLIC_SDK_ARTIFACT.md): install-tree contents and public SDK packaging contract.

## Release / Acceptance Docs

- [release/BUILD_RELEASE.md](release/BUILD_RELEASE.md): source build, non-replay gate, install-tree validation.
- [release/RELEASE_GATE.md](release/RELEASE_GATE.md): release gate checklist.
- [release/ACCEPTANCE_LAYERS.md](release/ACCEPTANCE_LAYERS.md): L0-L5 acceptance layering.
- [release/HARDENING_BACKLOG.md](release/HARDENING_BACKLOG.md): remaining hardening work that still affects release confidence.

## Reference Docs

- [reference/SDK_ALIGNMENT.md](reference/SDK_ALIGNMENT.md): compatibility and ABI/reference alignment.
- [reference/CAPABILITY_MATRIX.md](reference/CAPABILITY_MATRIX.md): launch/profile capability matrix.
- [reference/RUNTIME_STATE_MACHINE.md](reference/RUNTIME_STATE_MACHINE.md): runtime status authority.
- [reference/RECORDED_PATH_SCHEMA.md](reference/RECORDED_PATH_SCHEMA.md): recorded path schema.
- [reference/xmate_er3_alignment_manifest.json](reference/xmate_er3_alignment_manifest.json): machine-readable alignment source.
- [reference/official_cpp_sdk_oracle.json](reference/official_cpp_sdk_oracle.json): source-scope official SDK oracle.

## Architecture Docs

- [architecture/ARCHITECTURE.md](architecture/ARCHITECTURE.md): high-level runtime/package layering.
- [architecture/PROVIDER_BOUNDARY.md](architecture/PROVIDER_BOUNDARY.md): provider/backend boundary.

## Removed History

Historical P0/P1 process notes and audit reports were removed from the active tree. Current docs are scoped to the xMateER3 public lane, release acceptance, install-tree consumption, and maintained runtime architecture.

Current documentation groups are `public/`, `architecture/`, `release/`, and `reference/`.
