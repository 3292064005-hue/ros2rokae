# Public SDK Artifact

> 状态：Active  
> 受众：install-tree consumer / packaging 维护者  
> 作用：public SDK 安装物说明  
> 上游事实来源：`cmake/targets_packaging.cmake`、`cmake/xCoreSDKConfig.cmake.in`  
> 最后校验：2026-04-18

## Public SDK install contents

- `include/rokae/*`
- `lib/cmake/xCoreSDK/*`
- `share/rokae_xmate3_ros2/cmake/*` required export glue
- public examples
- canonical public launch resources
- generated canonical description
- docs/public + docs/reference + docs/release (install-facing copies)
- install-facing acceptance entrypoints under `share/rokae_xmate3_ros2/tools/*`
- mirrored release wrapper scripts under `share/rokae_xmate3_ros2/tools/*` (workspace-source inputs remain required for static sanity / ctest stages)

## Excluded from public SDK

- internal/full service surfaces
- generated internal ROSIDL mirrors
- IO / RL / calibration
- experimental/internal-only examples

更多构建与发布规则见 [`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md)。
相关状态机与 replay schema 见 [`../reference/RUNTIME_STATE_MACHINE.md`](../reference/RUNTIME_STATE_MACHINE.md) 与 [`../reference/RECORDED_PATH_SCHEMA.md`](../reference/RECORDED_PATH_SCHEMA.md)。

> install-tree copies are materialized under `share/rokae_xmate3_ros2/docs/README.md`, where the corresponding links resolve as `release/BUILD_RELEASE.md`, `reference/RUNTIME_STATE_MACHINE.md`, and `reference/RECORDED_PATH_SCHEMA.md`. Public/install-facing docs must use install-tree tool entrypoints (`share/rokae_xmate3_ros2/tools/*`) unless a section is explicitly marked source-tree/internal. Acceptance entrypoints are install-tree runnable against a built workspace root; release wrappers remain workspace-source oriented because they execute static sanity and ctest stages.
