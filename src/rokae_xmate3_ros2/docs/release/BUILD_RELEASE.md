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
- `check_xmate6_official_alignment.py`
- `check_compat_public_abi.py`
- `check_docs_layout.py`
- `check_cpp_signature_sync.py`
- Python syntax checks

### 真实构建/运行级
- `colcon build`
- gtest 编译/运行
- `ros2 launch` smoke
- install-tree `find_package(xCoreSDK)` 消费验证（`compat_install_tree_consumer` / `compat_no_ros_env_external_consumer`，归入 `release_gate`）

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
source-tree / CI 默认只强制 L0-L2；其中 L1 必须执行 `xmate6_alignment` 行为 bundle。L3-L5 通过独立脚本在真实 runtime 环境中执行。

## 7. Source-tree physical split

- public-only baseline keeps ROSIDL under `srv/` and examples under `examples/cpp/`.
- internal/backend-only surfaces live under `internal_interfaces/srv/` and `examples/internal/cpp/`.
- enabling `ROKAE_ENABLE_INTERNAL_SURFACE=ON` is required for internal/backend service registration, internal examples, and private SDK export bundles.

## 8. Maintenance docs

- 实现审计：[`../archive/audits/IMPLEMENTATION_AUDIT.md`](../archive/audits/IMPLEMENTATION_AUDIT.md)
- 剩余硬化项：[`HARDENING_BACKLOG.md`](HARDENING_BACKLOG.md)
