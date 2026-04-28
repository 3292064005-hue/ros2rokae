# Kinematics and Model

> 状态：Active  
> 受众：运动学 / 模型维护者 / 需要理解 exactness 的调用者  
> 作用：运动学策略、模型溯源、fidelity 与 traceability 的唯一主说明  
> 上游事实来源：`include/rokae_xmate3_ros2/spec/xmate_er3_truth.hpp`（canonical truth）、`include/rokae_xmate3_ros2/spec/xmate_six_axis_common.hpp`（shared six-axis basis）、`xmate3_spec.hpp`（compat alias）、kinematics policy、model helpers  
> 最后校验：2026-04-18

## 1. Scope

只描述当前 xMateER3 六轴主线上实际使用的模型规则；不复述官方基础表格，不讨论标定流程。

## 2. Source-of-truth split

当前仓内集中维护：
- joint limits
- velocity / acceleration / jerk envelopes
- direct torque envelopes
- manual-style `official_dh`
- helper-style `improved_dh`

## 3. Runtime policy

- primary backend: `KDL`
- fallback backend: `improved_dh`
- jacobian mode: native backend Jacobian when available
- single request must use exactly one primary backend

fallback backend 只允许用于：
- seed generation
- retry on primary failure
- regression comparison

## 4. Consumers

- URDF limits and runtime validators
- IK / FK / Jacobian requests
- soft-limit defaults
- retimer envelopes
- SDK facade model calls

## 5. Fidelity / exactness grades

当前意图：
- kinematics: simulation-grade
- jacobian: simulation-grade
- dynamics / wrench mapping: approximate

因此：当前 public lane 只提供**仿真兼容层**，接口保持连续，但不宣称 controller-model parity，也不在本轮为未来硬件 backend 预埋 backend-neutral 公共模型承诺。

## 6. Traceability and revision

- canonical description artifact：`<build>/generated/urdf/xMateER3.urdf`
- canonical description metadata：`<build>/generated/urdf/xMateER3.description.json`
- install-time copies：`share/rokae_xmate3_ros2/generated/urdf/*`
- runtime/profile/diagnostics 查询必须暴露 `model_revision`，用于把行为语义绑定到当前模型修订

显式替换非 canonical xacro/model 时，必须设置 `allow_noncanonical_model:=true`。

## 7. Related docs

- [`COMPATIBILITY.md`](COMPATIBILITY.md)
- [`../architecture/ARCHITECTURE.md`](../architecture/ARCHITECTURE.md)

## 8. Public model facade boundary

`include/rokae_xmate3_ros2/model_facade.hpp` owns its public wrapper types (`ModelFacade`, `ModelLoadContext`, `ModelDynamicsBreakdown`, `ModelDiagnostics`) and consumes the runtime `kinematics::Provider` boundary directly. It no longer includes or delegates through `rokae_xmate3_ros2/gazebo/model_facade.hpp` in the public facade layer.

The default provider used by this package is still simulation-grade because the current scope explicitly does not add a hardware backend. The facade therefore provides a provider-owned public boundary, not hardware-model parity and not a future hardware backend contract.

`include/rokae_xmate3_ros2/model.hpp` keeps concrete provider ownership behind an opaque `XMateModel::Impl`. Public headers must not include `runtime/kinematics_provider.hpp`, name `OwnedGazeboProvider`, or expose Gazebo symbols. The default implementation is compiled into the SDK and remains simulation-grade.


### Public provider interface boundary

The public model facade includes only `runtime/kinematics_provider_interface.hpp`. Concrete simulation providers, including the Gazebo-backed provider, remain behind implementation-side headers or compiled translation units and must not be introduced transitively through public model facade headers.
