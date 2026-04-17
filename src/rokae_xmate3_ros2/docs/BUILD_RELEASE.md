# Build and Release

> 状态：Active  
> 受众：构建维护者 / 发布人员 / install-tree consumer 维护者  
> 作用：source-tree 依赖、artifact、环境锁定与验证分级的唯一主说明  
> 上游事实来源：`package.xml`、CMake targets、release scripts、target-env checks  
> 最后校验：2026-04-17

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
- public compat examples
- canonical public launch resources
- public simulation resources
- docs / tools / generated canonical description

不属于 public contract：
- `rokae_xmate3_ros2/*` private backend headers
- internal/full service surfaces
- IO / RL / calibration
- experimental/internal-only examples

## 3. Validation levels

### 静态/脚本级
- `check_repo_contract.py`
- `check_xmate6_official_alignment.py`
- `check_compat_public_abi.py`
- Python syntax checks

### 真实构建/运行级
- `colcon build`
- gtest 编译/运行
- `ros2 launch` smoke
- install-tree `find_package(xCoreSDK)` 消费验证

未做真实构建/运行验证时，不能把静态检查写成“已完全可交付”。

## 4. Target environment lock

- preflight: `tools/check_target_environment.sh`
- target-env acceptance: `tools/run_target_env_acceptance.sh`
- release gate: `tools/run_release_gate.sh`
- portable release gate: `tools/run_release_gate_portable.sh`

## 5. Release rules

- 交付包必须无 `build/install/log`
- 无 `__pycache__` / `.pyc`
- 文档入口必须与 public artifact 同步
- public target 不得重新引入 Gazebo 直绑 public export
