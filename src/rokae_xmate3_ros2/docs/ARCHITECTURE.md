# Architecture

> 状态：Active  
> 受众：Runtime / SDK 维护者  
> 作用：主链、fidelity、扩展和 catalog 规则的唯一高层架构说明  
> 上游事实来源：runtime sources、launch/runtime host builder、contract tests  
> 最后校验：2026-04-17

## 1. Platform identity

本仓当前身份是：
- **xMate6 public SDK compatibility lane**
- 通过 ROS2/Gazebo-backed runtime 提供 install-facing compat facade
- 保留 legacy `rokae_xmate3_ros2` 包名与部分历史入口，仅用于兼容

不是：
- 全机型通用 SDK 文档仓
- 真机控制柜级 authoritative RT 文档仓
- RL / IO / calibration 公共说明仓

## 2. Main layers

1. public SDK façade (`include/rokae/*`, `xCoreSDK::*`)
2. runtime services / actions / queries
3. request coordinator / motion runtime / state authority
4. backend / simulation host

## 3. Immutable design rules

### Runtime is the only state authority
所有状态真值都必须回到 runtime/coordinator authority surface。

### Main contracts and legacy contracts must be explicit
- preferred surface: public xMate6 lane
- legacy facade: 兼容历史名称，但不能伪装成新的主契约

### Kinematics uses one primary backend per request
单个请求必须使用一个 primary backend；详见 `KINEMATICS_AND_MODEL.md`。

### RT and NRT remain permanently split
RT 与 NRT 语义永久分层；RT 在 Gazebo 中只声明 simulation-grade，不伪装成 controller-grade。

## 4. Fidelity semantics

- **StrictAligned**：接口与语义已收口到当前 public 主线
- **SimApprox**：接口可用，但底层为仿真近似
- **Experimental**：只允许 internal/runtime lane 公开

## 5. Catalog and contract policy

- runtime catalog 是 tool/wobj/runtime state 的唯一真值源
- public lane 不能继续扩写 RL / IO / calibration 的公共承诺
- 新能力接入前必须过 `motion_extension_contract` 启动期 fail-fast 校验

## 6. Extension domains

允许扩展：
- xMate6 public lane 文档与 compat facade
- runtime query authority
- diagnostics / catalog / host lifecycle

不应再在 public lane 继续扩展：
- RL
- IO
- calibration
- experimental RT examples

## 7. Related docs

- [`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- [`KINEMATICS_AND_MODEL.md`](KINEMATICS_AND_MODEL.md)
- [`COMPATIBILITY.md`](COMPATIBILITY.md)
