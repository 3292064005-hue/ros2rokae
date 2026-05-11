# ROKAE xMateER3 Public SDK Compatibility Lane

`rokae_xmate3_ros2` 是面向 xMateER3 六轴机械臂的 ROS 2 / Gazebo 仿真与 public SDK 兼容包。仓库保留部分 legacy 包名和兼容入口，但当前有效说明范围以 xMateER3 public lane 为准。

本仓库的 public lane 只承诺公开 SDK 兼容面、仿真级运行时、基础状态查询、toolset、运动学模型、安装树消费和验收契约。不承诺坐标系标定、RL 工程、通用 IO / 寄存器 / xPanel parity，也不把 experimental RT 控制回路作为 public release 证明。

## 快速入口

- 总索引：[docs/INDEX.md](docs/INDEX.md)
- 快速上手：[docs/public/QUICKSTART.md](docs/public/QUICKSTART.md)
- 兼容范围：[docs/public/COMPATIBILITY.md](docs/public/COMPATIBILITY.md)
- Runtime profiles：[docs/public/RUNTIME_PROFILES.md](docs/public/RUNTIME_PROFILES.md)
- Capability matrix：[docs/reference/CAPABILITY_MATRIX.md](docs/reference/CAPABILITY_MATRIX.md)
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
bash src/rokae_xmate3_ros2/tools/clean_build_env.sh
colcon build --packages-select rokae_xmate3_ros2 --symlink-install
```

完整 source-tree / Gazebo 构建需要目标环境提供 ROS 2 Humble、Gazebo、`colcon`、`ros2`、`rosdep` 和 `gazebo_ros`。

### 启动

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py launch_profile:=public_xmate_er3_jtc
ros2 launch rokae_xmate3_ros2 simulation.launch.py launch_profile:=public_xmate_er3_headless_sdk_smoke
```

`public_xmate_er3_jtc` 默认启用 `require_runtime_readiness:=true`，启动后会等待 `controller_manager/list_controllers`、`joint_trajectory_controller` active 状态和 `/joint_trajectory_controller/follow_joint_trajectory` action server；缺失时 fail-fast。只做 launch 参数或图形调试时可显式设置 `require_runtime_readiness:=false`，但该模式不能作为 Gazebo/JTC 主链路验证。

### public SDK 安装树消费

```cmake
find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_core)
```

canonical install-facing identity: `xCoreSDK`

`xCoreSDK::xCoreSDK_core` 是默认 install-facing C++ core SDK 入口。需要 Robot 会话、ROS bridge 或 runtime 组件时，显式请求：

```cmake
find_package(xCoreSDK COMPONENTS shared static ros_bridge CONFIG REQUIRED)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

## 行为边界

- `MoveAppend` 成功表示队列接收，真正执行由 `moveStart()` 提交。
- `stop()` 表示 pause，不清空队列；要丢弃待执行 NRT 请求请使用 `moveReset()`。
- `replayPath()` 是 experimental immediate-submit side-lane，默认 public profile 不注册路径录制/回放服务。
- `GetEndWrench` 是 public lane 推荐的扩展查询面，响应包含字段有效性标注。
- `MoveSP`、路径录制/回放和 RT/drag 需要 `public_xmate_er3_experimental` 或 internal exposure；默认 public profile 会拒绝 `MoveSP` action payload，不注册路径/RT/drag 服务。
- 当前 runtime 是 simulation-grade，不承诺 controller-grade 实机闭环 parity。
- 坐标系标定类接口只保留兼容签名，返回 `function_not_supported`。

## 生成物与契约锚点

- canonical description artifact：`<build>/generated/urdf/xMateER3.urdf`
- canonical metadata：`<build>/generated/urdf/xMateER3.description.json`
- compatibility alias artifact：`<build>/generated/urdf/xMate3.urdf`
- compatibility alias metadata：`<build>/generated/urdf/xMate3.description.json`
- runtime snapshot service：`/xmate_er3/cobot/get_runtime_state_snapshot`
- compatibility snapshot alias：默认不发布；需要旧 `/xmate3` 名称时设置 `compatibility_alias_policy:=canonical_plus_compat`
- service contract manifest：`src/runtime/service_contract_manifest.hpp`
- service contract registry：`src/runtime/service_contract_manifest.cpp`

## 验证入口

```bash
bash src/rokae_xmate3_ros2/tools/run_quick_gate.sh <workspace-root>
bash src/rokae_xmate3_ros2/tools/run_full_source_tree_build_gate.sh <workspace-root>
bash src/rokae_xmate3_ros2/tools/run_xmate_er3_alignment_behavior_gate.sh <workspace-root>
```

`ROKAE_PUBLIC_SDK_REPLAY_ONLY=ON` 只用于 smoke 级 install/package 形状验证，不能替代真实 release/install 证明。目标环境验收以 `tools/verify_target_env_acceptance_report.py` 接受的报告为准。

## 文档维护规则

`README.md` 只保留入口、边界和常用命令。完整架构、验收矩阵、发布证据和历史审计内容放在 `docs/` 下维护；阶段性过程文档归档到 `docs/archive/`。
