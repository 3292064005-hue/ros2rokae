# Runtime Catalog Policy

This document freezes the rule that **runtime is the only source of truth** for queryable catalogs.

## Covered catalogs

- RL project catalog
- tool catalog
- wobj catalog

## Rule

No wrapper, query service, or compatibility facade may synthesize its own catalog truth.

All catalog reads must originate from runtime-owned state:

- `ProgramState` for RL project catalog
- `ToolingState` for tool and wobj catalogs
- `runtime_catalog_service` for normalized read models

## Normalization goals

The normalized catalog layer must ensure:

1. the active runtime selection is always visible even if the registry started empty
2. query services and SDK facades observe the same catalog order and content
3. preferred and legacy contract pairs do not diverge on active tool/wobj/project identity

## Current preferred surfaces

- `GetRlProjectInfo`
- `GetToolCatalog`
- `GetWobjCatalog`

These are query-domain reads, even if the mutations remain in the IO / program facade.
