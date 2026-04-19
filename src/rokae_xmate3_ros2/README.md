# ROKAE xMate6 Public SDK Compatibility Lane (`rokae_xmate3_ros2`)

> 面向 **xMate 六轴 public compatibility lane** 的 ROS2/Gazebo-backed SDK 兼容包。保留 legacy 包名与部分历史入口，但当前有效说明只覆盖 **xMate 六轴主线**。
>
> 说明范围按仓内约束冻结：**不写机械臂标定、不做 RL、不做 IO**；若与外部手册基础数据冲突，以本仓当前 contract、源码和测试约束为准。构建方式允许不同，但源码、接口与行为语义必须一致。

## 先看这里

- **总索引**：[`docs/INDEX.md`](docs/INDEX.md)
- **3 分钟上手**：[`docs/public/QUICKSTART.md`](docs/public/QUICKSTART.md)
- **public SDK 兼容范围**：[`docs/public/COMPATIBILITY.md`](docs/public/COMPATIBILITY.md)
- **runtime profile / authority / RT-NRT 规则**：[`docs/public/RUNTIME_PROFILES.md`](docs/public/RUNTIME_PROFILES.md)
- **架构总览**：[`docs/architecture/ARCHITECTURE.md`](docs/architecture/ARCHITECTURE.md)
- **provider 边界**：[`docs/architecture/PROVIDER_BOUNDARY.md`](docs/architecture/PROVIDER_BOUNDARY.md)
- **运动学 / 模型 / fidelity**：[`docs/public/KINEMATICS_AND_MODEL.md`](docs/public/KINEMATICS_AND_MODEL.md)
- **构建 / 发布**：[`docs/release/BUILD_RELEASE.md`](docs/release/BUILD_RELEASE.md)
- **示例分层**：[`docs/public/EXAMPLES.md`](docs/public/EXAMPLES.md)
- **runtime 状态机**：[`docs/reference/RUNTIME_STATE_MACHINE.md`](docs/reference/RUNTIME_STATE_MACHINE.md)
- **路径录制 schema**：[`docs/reference/RECORDED_PATH_SCHEMA.md`](docs/reference/RECORDED_PATH_SCHEMA.md)

## 当前有效范围

### 纳入说明
- xMate 六轴 public lane
- 非实时主链：`MoveReset -> MoveAppend -> MoveStart -> Stop(pause)`
- 基础状态查询、toolset、运动学模型、兼容安装面
- canonical launch 与 public artifact 使用方式
- ABI 兼容安装面

### 不纳入说明
- 坐标系标定；`calibrateFrame()` 仅保留兼容签名，返回 `function_not_supported`
- RL 工程说明
- IO / 寄存器 / xPanel 公共承诺
- public lane 中的 experimental RT 控制回环示例
- 安装态 public xMate6 lane 不再承诺通用 IO / RL / xPanel parity

## 关键语义

- `MoveAppend` 成功边界是 **queue accepted**；真正执行由 `moveStart()` 提交。
- `stop()` 是 **pause**，不会清空队列；彻底丢弃待执行 NRT 请求应使用 `moveReset()`。
- `GetEndWrench` 是 public lane 的首选扩展查询面；`MoveSP` 与路径录制/回放已纳入 public xMate6 lane 的 NRT 扩展面。
- 当前 runtime 是 simulation-grade，不承诺 controller-grade 实机闭环 parity。
- 本仓不支持任何坐标系标定功能；标定类接口只作为兼容 stub，返回 `function_not_supported`。

## canonical 入口

### 启动
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
ros2 launch rokae_xmate3_ros2 xmate6_public.launch.py
```

### 构建
```bash
src/rokae_xmate3_ros2/tools/clean_build_env.sh
colcon build --packages-select rokae_xmate3_ros2 --symlink-install
```

### install-facing CMake 消费
```cmake
find_package(xCoreSDK CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_static)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_shared)
```

说明：
- canonical install-facing identity: `xCoreSDK`
- `xCoreSDK::xCoreSDK_static` 是真实静态目标
- 配置期同样需要系统可见的 ROS2/Gazebo 依赖
- `rokae_xmate3_ros2` 仅作为 source-tree / legacy package alias 保留

## 文档规则

- `README.md` 只做入口说明，不再承载完整架构与审计细节。
- `docs/INDEX.md` 是唯一总索引。
- 主说明分成四组：`public/`、`architecture/`、`release/`、`reference/`。
- 阶段性过程文档已从主树删除，只在 `docs/archive/` 保留历史归档。

## 内部契约与生成物锚点

- build-generated canonical description artifact：`<build>/generated/urdf/xMate3.urdf`
- generated description metadata：`<build>/generated/urdf/xMate3.description.json`
- aggregated runtime snapshot query surface：`/xmate3/internal/get_runtime_state_snapshot`
- single-source service contract manifest：`src/runtime/service_contract_manifest.hpp`

## 维护与审计入口

- 对齐事实源：[`docs/reference/xmate6_official_alignment_manifest.json`](docs/reference/xmate6_official_alignment_manifest.json)
- 对齐参考：[`docs/reference/SDK_ALIGNMENT.md`](docs/reference/SDK_ALIGNMENT.md)
- 当前实现审计：[`docs/archive/audits/IMPLEMENTATION_AUDIT.md`](docs/archive/audits/IMPLEMENTATION_AUDIT.md)
- 剩余硬化任务：[`docs/release/HARDENING_BACKLOG.md`](docs/release/HARDENING_BACKLOG.md)
- canonical description artifact：`<build>/generated/urdf/xMate3.urdf`

- 分层验收矩阵：[`docs/release/ACCEPTANCE_LAYERS.md`](docs/release/ACCEPTANCE_LAYERS.md)
