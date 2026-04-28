# ROKAE xMateER3 Public SDK Compatibility Lane

`rokae_xmate3_ros2` 是面向 xMateER3 六轴机械臂的 ROS 2 / Gazebo 仿真与 public SDK 兼容包。仓库保留部分 legacy 包名和兼容入口，但当前有效说明范围以 xMateER3 public lane 为准。

本仓库的 public lane 只承诺公开 SDK 兼容面、仿真级运行时、基础状态查询、toolset、运动学模型、安装树消费和验收契约。不承诺坐标系标定、RL 工程、通用 IO / 寄存器 / xPanel parity，也不把 experimental RT 控制回路作为 public release 证明。

## 快速入口

- 总索引：[docs/INDEX.md](docs/INDEX.md)
- 快速上手：[docs/public/QUICKSTART.md](docs/public/QUICKSTART.md)
- 兼容范围：[docs/public/COMPATIBILITY.md](docs/public/COMPATIBILITY.md)
- Runtime profiles：[docs/public/RUNTIME_PROFILES.md](docs/public/RUNTIME_PROFILES.md)
- 架构说明：[docs/architecture/ARCHITECTURE.md](docs/architecture/ARCHITECTURE.md)
- Provider 边界：[docs/architecture/PROVIDER_BOUNDARY.md](docs/architecture/PROVIDER_BOUNDARY.md)
- 运动学与模型：[docs/public/KINEMATICS_AND_MODEL.md](docs/public/KINEMATICS_AND_MODEL.md)
- 构建与发布：[docs/release/BUILD_RELEASE.md](docs/release/BUILD_RELEASE.md)
- 环境锁定：[docs/release/ENVIRONMENT_LOCK.md](docs/release/ENVIRONMENT_LOCK.md)
- Release gate：[docs/release/RELEASE_GATE.md](docs/release/RELEASE_GATE.md)
- 验收分层：[docs/release/ACCEPTANCE_LAYERS.md](docs/release/ACCEPTANCE_LAYERS.md)
- 示例分层：[docs/public/EXAMPLES.md](docs/public/EXAMPLES.md)
- SDK 对齐参考：[docs/reference/SDK_ALIGNMENT.md](docs/reference/SDK_ALIGNMENT.md)
- Runtime 状态机：[docs/reference/RUNTIME_STATE_MACHINE.md](docs/reference/RUNTIME_STATE_MACHINE.md)
- 路径录制 schema：[docs/reference/RECORDED_PATH_SCHEMA.md](docs/reference/RECORDED_PATH_SCHEMA.md)

## 常用命令

### 构建

```bash
src/rokae_xmate3_ros2/tools/clean_build_env.sh
colcon build --packages-select rokae_xmate3_ros2 --symlink-install
```

完整 source-tree / Gazebo 构建需要目标环境提供 ROS 2 Humble、Gazebo、`colcon`、`ros2`、`rosdep` 和 `gazebo_ros`。

### 启动

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
ros2 launch rokae_xmate3_ros2 xmate_er3_public.launch.py
```

### public SDK 安装树消费

```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

`xCoreSDK::xCoreSDK_core` 是默认 install-facing C++ core SDK 入口。需要 Robot 会话、ROS bridge 或 runtime 组件时，显式请求：

```cmake
find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

## 行为边界

- `MoveAppend` 成功表示队列接收，真正执行由 `moveStart()` 提交。
- `stop()` 表示 pause，不清空队列；要丢弃待执行 NRT 请求请使用 `moveReset()`。
- `replayPath()` 是立即提交型 side-lane，不经过 `moveStart()`。
- `GetEndWrench` 是 public lane 推荐的扩展查询面。
- `MoveSP` 与路径录制 / 回放已纳入 public xMateER3 lane 的 NRT 扩展面。
- 当前 runtime 是 simulation-grade，不承诺 controller-grade 实机闭环 parity。
- 坐标系标定类接口只保留兼容签名，返回 `function_not_supported`。

## 生成物与契约锚点

- canonical description artifact：`<build>/generated/urdf/xMateER3.urdf`
- canonical metadata：`<build>/generated/urdf/xMateER3.description.json`
- compatibility alias artifact：`<build>/generated/urdf/xMate3.urdf`
- compatibility alias metadata：`<build>/generated/urdf/xMate3.description.json`
- runtime snapshot service：`/xmate_er3/cobot/get_runtime_state_snapshot`
- compatibility snapshot alias：`/xmate3/internal/get_runtime_state_snapshot`
- service contract manifest：`src/runtime/service_contract_manifest.hpp`
- service contract registry：`src/runtime/service_contract_manifest.cpp`

## 验证入口

```bash
src/rokae_xmate3_ros2/tools/run_quick_gate.sh <workspace-root>
src/rokae_xmate3_ros2/tools/run_full_source_tree_build_gate.sh <workspace-root>
src/rokae_xmate3_ros2/tools/run_xmate_er3_alignment_behavior_gate.sh <workspace-root>
```

`ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON` 只用于 smoke 级 install/package 形状验证，不能替代真实 release/install 证明。目标环境验收以 `tools/verify_target_env_acceptance_report.py` 接受的报告为准。

## 文档维护规则

`README.md` 只保留入口、边界和常用命令。完整架构、验收矩阵、发布证据和历史审计内容放在 `docs/` 下维护；阶段性过程文档归档到 `docs/archive/`。
