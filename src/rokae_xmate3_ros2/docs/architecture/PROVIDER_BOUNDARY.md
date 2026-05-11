# Provider Boundary

> 状态：Active  
> 受众：runtime / backend 维护者  
> 作用：provider / host / backend 边界的唯一专门说明  
> 上游事实来源：`src/runtime/backend_provider.*`、`src/gazebo/runtime_bootstrap.cpp`、`src/runtime/sim_runtime_main.cpp`  
> 最后校验：2026-04-18

## 1. 边界链

唯一允许的运行边界是：

`RuntimeBackendProviderHost -> RuntimeBackendProvider -> BackendInterface`

host/bootstrap、plugin、daemon runtime 只能消费：
- `RuntimeBackendProviderHost`
- `RuntimeBackendProvider`
- `BackendContractDescriptor`
- `BackendInterface`

## 2. generic backend factory request

`RuntimeBackendProviderHost` 只暴露通用工厂请求：

- `RuntimeBackendFactoryRequest`
- `supportsFactory(...)`
- `advertisedFactoryKeys()`
- `createBackend(const RuntimeBackendFactoryRequest &request)`

不再允许把 `simulation` / `headless` 这类 backend family 专用工厂方法写进公共抽象。

## 3. provider 职责

provider 负责：
- 解析 factory key
- 选择 host 提供的 backend factory
- 组装 trajectory/controller 依赖
- 暴露 capability flags 与 contract descriptor

host 负责：
- 提供宿主级 backend factory 能力
- 不把 Gazebo / headless concrete context 类型泄漏到公共 provider 接口

## 4. 扩展规则

新增 backend family 时：
- 可以新增 factory key
- 可以新增 provider 实现
- 可以新增 host 实现
- **不能** 再修改 `RuntimeBackendProviderHost` 公共抽象来增加专用工厂方法

## 5. 门禁

- `test/harness/check_provider_boundary.py`
- `tools/run_static_sanity.sh`

这两个门禁用于阻止旧的 `hostFlavor/createSimulationBackend/createHeadlessTestBackend` 风格重新回到公共抽象。

## 6. 相关文档

- [`ARCHITECTURE.md`](ARCHITECTURE.md)
- [`../public/RUNTIME_PROFILES.md`](../public/RUNTIME_PROFILES.md)
- [`../release/RELEASE_GATE.md`](../release/RELEASE_GATE.md)
