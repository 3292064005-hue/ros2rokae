# Kinematics and Model

> 状态：Active  
> 受众：运动学 / 模型维护者 / 需要理解 exactness 的调用者  
> 作用：运动学策略、模型溯源、exactness 语义的唯一主说明  
> 上游事实来源：`include/rokae_xmate3_ros2/spec/xmate3_spec.hpp`、kinematics policy、model helpers  
> 最后校验：2026-04-17

## 1. Scope

只描述当前 xMate 六轴主线上实际使用的模型规则；不复述官方基础表格，不讨论标定流程。

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

## 5. Exactness grades

当前 intent：
- kinematics: simulation-grade
- jacobian: simulation-grade
- dynamics / wrench mapping: approximate

因此：接口保持连续，但不宣称 controller-model parity。

## 6. Canonical description artifact

- build-time canonical URDF: `<build>/generated/urdf/xMate3.urdf`
- build-time metadata: `<build>/generated/urdf/xMate3.description.json`
- install-time canonical URDF: `share/rokae_xmate3_ros2/generated/urdf/xMate3.urdf`
- install-time metadata: `share/rokae_xmate3_ros2/generated/urdf/xMate3.description.json`

显式替换非 canonical xacro/model 时，必须设置 `allow_noncanonical_model:=true`。
