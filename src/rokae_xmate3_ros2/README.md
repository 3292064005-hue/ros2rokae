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
- ABI 兼容安装面

### 不纳入说明
- 坐标系标定；`calibrateFrame()` 仅保留兼容签名，返回 `function_not_supported`
- RL 工程说明
- IO / 寄存器 / xPanel 公共承诺
- public lane 中的 experimental RT 示例
- 安装态 public xMate6 lane 不再承诺通用 IO / RL / xPanel parity。

## 关键语义

- `MoveAppend` 成功边界是 **queue accepted**；真正执行由 `moveStart()` 提交。
- `stop()` 是 **pause**，不会清空队列；彻底丢弃待执行 NRT 请求应使用 `moveReset()`。
- `GetEndWrench` 是 public lane 的首选扩展查询面；`ReadRegisterEx / WriteRegisterEx / GetRlProjectInfo / SetXPanelVout` 仅保留 internal/backend 语义。
- install-facing `xCoreSDK` lane 仍是 **ROS2-backed** 兼容安装包，但 public target 不再把 Gazebo 作为 public CMake 直绑依赖导出。
- 当前 runtime 是 simulation-grade，不承诺 controller-grade 实机闭环 parity；`ppToMain()` 仅返回最近一次成功加载的工程路径。
- 本仓不支持任何坐标系标定功能；标定类接口只作为兼容 stub，返回 `function_not_supported`。
- `setFcCoor()`、`useRciClient(true)` 与 `setRtNetworkTolerance()` 保留兼容配置语义，实际能力以 runtime profile 和诊断输出为准。
- RT 元数据字段通过 `RtCompatFields::samplePeriod_s` / `RtCompatFields::sampleFresh` 暴露采样周期与新鲜度。

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
- `xCoreSDK::xCoreSDK_shared` 是 install-facing shared 入口。
- `xCoreSDK::xCoreSDK_static` 是真实静态目标，不再伪装成 shared alias。
- 当前兼容安装面仍是 ROS2/Gazebo-backed；配置期同样需要系统可见的 ROS2/Gazebo 依赖。


## 内部契约与生成物锚点

- build-generated canonical description artifact：`<build>/generated/urdf/xMate3.urdf`
- generated description metadata：`<build>/generated/urdf/xMate3.description.json`
- aggregated runtime snapshot query surface：`/xmate3/internal/get_runtime_state_snapshot`
- single-source service contract manifest：`src/runtime/service_contract_manifest.hpp`
- service exposure switch：`service_exposure_profile`
- runtime lifecycle assembly：`runtime_host_builder`

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

## 实现审计

当前实现审计与剩余硬化任务仍保留在 `docs/IMPLEMENTATION_AUDIT.md` 与 `docs/HARDENING_BACKLOG.md`，作为 release / review 的维护入口。
