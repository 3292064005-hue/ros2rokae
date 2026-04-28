# Build and Release

> 状态：Active  
> 受众：构建维护者 / 发布人员 / install-tree consumer 维护者  
> 作用：source-tree 依赖、artifact、环境锁定与验证分级的唯一主说明  
> 上游事实来源：`package.xml`、CMake targets、release scripts、target-env checks  
> 最后校验：2026-04-18

## 1. Source-tree build baseline

默认基线：
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11
- `python3` / `ROKAE_PYTHON_EXECUTABLE`

source-tree package metadata 必须诚实表达 canonical launch 与集成构建需要的依赖；这不等于 install-facing public SDK target 必须把同样的依赖导出成 public CMake contract。

## 2. Install-facing public artifact

public artifact 只包含：
- `include/rokae/*`
- `lib/cmake/xCoreSDK/*`
- `share/rokae_xmate3_ros2/cmake/*` 中被 `xCoreSDKConfig.cmake` 解析所需的导出胶水
- public compat examples
- canonical public launch resources
- public simulation resources
- docs / tools / generated canonical description（含 install-tree acceptance 入口与 mirrored release wrappers）

不属于 public contract：
- `rokae_xmate3_ros2/*` private backend headers
- internal/full service surfaces
- IO / RL / calibration
- experimental/internal-only examples
- generated `rosidl_generator_cpp/rokae_xmate3_ros2/*` 头文件镜像

release wrappers are mirrored for discoverability only; they still expect a workspace source tree when they invoke static sanity / ctest stages. Install-tree acceptance entrypoints must not depend on `src/rokae_xmate3_ros2/tools/*`.

## 3. Validation levels

### 静态/脚本级
- `check_repo_contract.py`
- `check_xmate_er3_alignment.py`
- `check_compat_public_abi.py`
- `check_docs_layout.py`
- `check_cpp_signature_sync.py`
- `check_public_sdk_packaging_contract.py`（smoke-only public_sdk packaging/install 形状验证）
- Python syntax checks

### 真实构建/运行级
- `colcon build`
- gtest 编译/运行
- `ros2 launch` smoke
- install-tree `find_package(xCoreSDK)` 消费验证（`compat_install_tree_consumer` / `compat_no_ros_env_external_consumer`，归入 `release_gate`）
- 上述 install-tree consumer 验证必须以 `xCoreSDK_PRIMARY_INSTALL_CONSUMER` 作为主运行时消费目标的解析输入；当前主消费者解析到 `xCoreSDK::xCoreSDK_core`；Robot/RT 运行时 consumers 通过显式 `COMPONENTS shared static ros_bridge` 请求桥接 target，其中 ROS2 bridge consumer 保留为 `xCoreSDK::xCoreSDK_ros_bridge`。
- `ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON` 只属于 smoke staging，不可替代真实 install consumer 验证

未做真实构建/运行验证时，不能把静态检查写成“已完全可交付”。

## 4. Target environment lock

环境锁定规则见 [`ENVIRONMENT_LOCK.md`](ENVIRONMENT_LOCK.md)。

## 5. Release rules

- 交付包必须无 `build/install/log`
- 无 `__pycache__` / `.pyc`
- 文档入口必须与 public artifact 同步
- public target 不得重新引入 Gazebo 直绑 public export

## 6. Release gate

release gate 执行链见 [`RELEASE_GATE.md`](RELEASE_GATE.md)。

## 6.1 Acceptance layers

确认方案要求的 L0-L5 分层验收体系见 [`ACCEPTANCE_LAYERS.md`](ACCEPTANCE_LAYERS.md)。
source-tree / CI 默认只强制 L0-L2；其中 L1 必须执行 `xmate_er3_alignment` 行为 bundle。L3-L5 通过独立脚本在真实 runtime 环境中执行。

## 7. Source-tree physical split

- public-only baseline keeps ROSIDL under `srv/` and examples under `examples/cpp/`.
- internal/backend-only surfaces live under `internal_interfaces/srv/` and `examples/internal/cpp/`.
- enabling `ROKAE_ENABLE_INTERNAL_SURFACE=ON` is required for internal/backend service registration, internal examples, and private SDK export bundles.

## 8. Maintenance docs

- 实现审计：[`../archive/audits/IMPLEMENTATION_AUDIT.md`](../archive/audits/IMPLEMENTATION_AUDIT.md)
- 剩余硬化项：[`HARDENING_BACKLOG.md`](HARDENING_BACKLOG.md)

## 9. Non-replay full source-tree build gate

`ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON` remains a smoke/staging gate only. The non-replay source-tree gate is:

```bash
tools/run_full_source_tree_build_gate.sh <workspace-root>
```

This script requires the locked Ubuntu 22.04 / ROS 2 Humble / Gazebo target environment, runs `tools/check_target_environment.sh`, configures the package with `ROKAE_PUBLIC_SDK_REPLAY_ONLY=OFF`, performs a real `colcon build`, then runs the requested `ctest` label set from the package build directory.

Passing this gate proves a real ROS2/Gazebo source build for the selected labels. It is still not 实机 / hardware validation and must not be described as a robot-side acceptance result.

## 10. Target-environment report requirement

Locked target-environment acceptance reports must include an explicit `status.full_source_gate` field and report-local evidence logs under `artifacts.expected_logs`. The quick and release gates delegate non-replay source-build ownership to `tools/run_full_source_tree_build_gate.sh`; a report that lacks `status.full_source_gate: passed`, lacks the referenced full-source log file, or lacks the `full-source-build-gate: passed` marker is not sufficient evidence for source-tree build closure.


## Full source gate execution boundary

The non-replay full source-tree gate is mandatory for target-environment release acceptance and must be run in the locked Ubuntu 22.04 / ROS 2 Humble / Gazebo environment. Passing replay-only smoke or static contract checks is not a substitute for this target-environment gate, and real hardware validation remains a separate layer.

### Provider-boundary release gate

Release checks assert that public model facade headers do not include concrete Gazebo provider headers. The Gazebo-backed provider remains behind implementation-side boundaries and is validated by the runtime source-integrity harness.

### Target-environment report verification

A target-environment run is not accepted as release evidence unless the generated acceptance report is verified with:

```bash
tools/verify_target_env_acceptance_report.py <report.json>
```

The verifier rejects `not_run`, `failed`, or missing `status.full_source_gate` evidence. This prevents a release checklist from treating an unexecuted ROS 2 Humble / Gazebo full source-tree gate as a pass.
