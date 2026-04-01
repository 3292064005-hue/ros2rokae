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


## 2026-04 strict runtime authority landing
- 原生 ROS facade `rokae::ros2::xMateRobot` 的 `projectInfo/toolsInfo/wobjsInfo/setProjectRunningOpt/pauseProject` 默认不再把 runtime 失败静默伪装成成功。
- SDK 兼容 wrapper 默认仍保留旧缓存回退语义；若需要 strict runtime authority，需显式设置 `ROKAE_SDK_STRICT_RUNTIME_AUTHORITY=true` 且关闭 `ROKAE_SDK_LEGACY_CATALOG_FALLBACK`。
- wrapper 析构不再调用全局 `rclcpp::shutdown()`；ROS 生命周期改由进程级 `RosContextOwner` 统一持有。


SDK-compatible wrappers (`rokae::Robot_T`, `rokae::Cobot`, `rokae::xMateRobot`) now default to the legacy catalog fallback policy across both their historical constructors and the `RosClientOptions` injection path so that historical SDK call sites keep their prior behavior unless they explicitly opt into strict runtime authority. The native ROS facade (`rokae::ros2::xMateRobot`) still defaults to strict runtime authority.
