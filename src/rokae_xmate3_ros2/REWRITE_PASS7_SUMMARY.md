# Pass 7 rewrite summary

## Focus

Pass 7 continues from Pass 6 and closes three high-value gaps:

1. explicit runtime profile capability query surface
2. runtime diagnostics exposure for active request and execution backend
3. SDK wrapper access to profile and runtime-option capability snapshots

## New public/runtime surface

- `srv/GetProfileCapabilities.srv`
- `src/runtime/query_profile_service.cpp`
- `src/sdk/robot_profiles.cpp`
- `docs/PROFILE_QUERY_POLICY.md`

## Diagnostics additions

`RuntimeDiagnostics` now also carries:

- `active_request_id`
- `active_execution_backend`

These fields make runtime ownership and active execution easier to explain from a single diagnostics surface.

## Architectural effect

Pass 7 does not change the core motion contract. It strengthens the package as a platform by making profile discovery and active execution observability first-class runtime-backed surfaces.
