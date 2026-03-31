# Profile Query Policy

`GetProfileCapabilities` is the preferred runtime surface for discovering:

- active runtime profile
- available profile descriptors
- runtime option descriptors

This service is intentionally **read-only**. It must never become a state-changing API.

## Design rules

1. profile truth comes from runtime, not from wrapper-side heuristics
2. option descriptors come from the runtime option catalog
3. diagnostics summaries and query results must agree
4. preferred contract is `GetProfileCapabilities`; summaries in diagnostics are convenience projections
