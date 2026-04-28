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
- generated canonical description (`generated/urdf/xMateER3.urdf`, `generated/urdf/xMateER3.description.json`)
- compatibility alias generated description (`generated/urdf/xMate3.urdf`, `generated/urdf/xMate3.description.json`)
- docs/public + docs/reference + docs/release (install-facing copies)
- install-facing acceptance entrypoints under `share/rokae_xmate3_ros2/tools/*`
- mirrored release wrapper scripts under `share/rokae_xmate3_ros2/tools/*` (workspace-source inputs remain required for static sanity / ctest stages)
- staged public_sdk packaging contract is smoke-verified by `ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON`, followed by a temporary-prefix component install, artifact assembly, and manifest checks
- 该 replay/install 路径只证明 public_sdk 组件形状、共享策略传播、docs/tools/description 物料存在，不证明真实 target ABI/link/install 消费闭环
- 真实 install-tree `find_package(xCoreSDK)` 消费验证仍归入 release gate / install-tree consumer tests
- replay/install smoke 模式与主线 launch/CMake 共享 `config/default_runtime_host_policy.env`；install-facing `xCoreSDK_BACKEND_MODE`、runtime host/profile 与 canonical description metadata 由同一默认策略派生，不再分叉

## Excluded from public SDK

- internal/full service surfaces
- generated internal ROSIDL mirrors
- IO / RL / calibration
- experimental/internal-only examples

更多构建与发布规则见 [`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md)。
相关状态机与 replay schema 见 [`../reference/RUNTIME_STATE_MACHINE.md`](../reference/RUNTIME_STATE_MACHINE.md) 与 [`../reference/RECORDED_PATH_SCHEMA.md`](../reference/RECORDED_PATH_SCHEMA.md)。

> install-tree copies are materialized under `share/rokae_xmate3_ros2/docs/README.md`, where the corresponding links resolve as `release/BUILD_RELEASE.md`, `reference/RUNTIME_STATE_MACHINE.md`, and `reference/RECORDED_PATH_SCHEMA.md`. Public/install-facing docs must use install-tree tool entrypoints (`share/rokae_xmate3_ros2/tools/*`) unless a section is explicitly marked source-tree/internal. Acceptance entrypoints are install-tree runnable against a built workspace root; release wrappers remain workspace-source oriented because they execute static sanity and ctest stages.
