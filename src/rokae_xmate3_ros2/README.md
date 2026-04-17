# ROKAE xMate6 Public SDK Compatibility Lane (`rokae_xmate3_ros2`)

> 面向 **xMate 六轴 public compatibility lane** 的 ROS2/Gazebo-backed SDK 兼容包。保留 legacy 包名与部分历史入口，但当前有效说明只覆盖 **xMate 六轴主线**。
>
> 说明范围按仓内约束冻结：**不写机械臂标定、不做 RL、不做 IO**；若与外部手册基础数据冲突，以本仓当前 contract、源码和测试约束为准。构建方式允许不同，但源码、接口与行为语义必须一致。

## 先看这里

- **3 分钟上手**：[`docs/QUICKSTART.md`](docs/QUICKSTART.md)
- **完整导航**：[`docs/INDEX.md`](docs/INDEX.md)
- **public SDK 兼容范围 / ABI / 对齐矩阵**：[`docs/COMPATIBILITY.md`](docs/COMPATIBILITY.md)
- **runtime profile / authority / RT-NRT 规则**：[`docs/RUNTIME_PROFILES.md`](docs/RUNTIME_PROFILES.md)
- **架构与扩展规则**：[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)
- **运动学 / 模型 / exactness**：[`docs/KINEMATICS_AND_MODEL.md`](docs/KINEMATICS_AND_MODEL.md)
- **构建 / 安装 / 发布**：[`docs/BUILD_RELEASE.md`](docs/BUILD_RELEASE.md)
- **示例分层**：[`docs/EXAMPLES.md`](docs/EXAMPLES.md)

## 当前有效范围

### 纳入说明
- xMate 六轴 public lane
- 非实时主链：`MoveReset -> MoveAppend -> MoveStart -> Stop(pause)`
- 基础状态查询、toolset、运动学模型、兼容安装面
- canonical launch 与 public artifact 使用方式

### 不纳入说明
- 坐标系标定；`calibrateFrame()` 仅保留兼容签名，返回 `function_not_supported`
- RL 工程说明
- IO / 寄存器 / xPanel 公共承诺
- public lane 中的 experimental RT 示例

## 关键语义

- `MoveAppend` 成功边界是 **queue accepted**；真正执行由 `moveStart()` 提交。
- `stop()` 是 **pause**，不会清空队列；彻底丢弃待执行 NRT 请求应使用 `moveReset()`。
- `GetEndWrench` 是 public lane 的首选扩展查询面；`ReadRegisterEx / WriteRegisterEx / GetRlProjectInfo / SetXPanelVout` 仅保留 internal/backend 语义。
- install-facing `xCoreSDK` lane 仍是 **ROS2-backed** 兼容安装包，但 public target 不再把 Gazebo 作为 public CMake 直绑依赖导出。

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
```


## 内部契约与生成物锚点

- build-generated canonical description artifact：`<build>/generated/urdf/xMate3.urdf`
- generated description metadata：`<build>/generated/urdf/xMate3.description.json`
- aggregated runtime snapshot query surface：`/xmate3/internal/get_runtime_state_snapshot`
- single-source service contract manifest：`src/runtime/service_contract_manifest.hpp`

## 文档规则

- `README.md` 只做入口说明，不再承载完整架构与审计细节。
- `docs/INDEX.md` 是唯一总索引。
- 当前主说明只有 7 份：
  - `COMPATIBILITY.md`
  - `RUNTIME_PROFILES.md`
  - `ARCHITECTURE.md`
  - `KINEMATICS_AND_MODEL.md`
  - `BUILD_RELEASE.md`
  - `EXAMPLES.md`
  - `QUICKSTART.md`
- `docs/` 根目录中其余旧文件名默认都是 **兼容跳转页 / 维护指针 / 归档指针**，不是当前主说明。
- 历史实施摘要与深度复核报告已移入 `docs/archive/`。

## 维护入口

- 当前实现审计：[`docs/IMPLEMENTATION_AUDIT.md`](docs/IMPLEMENTATION_AUDIT.md)
- 剩余硬化任务：[`docs/HARDENING_BACKLOG.md`](docs/HARDENING_BACKLOG.md)
- 历史归档：[`docs/archive/`](docs/archive/)
