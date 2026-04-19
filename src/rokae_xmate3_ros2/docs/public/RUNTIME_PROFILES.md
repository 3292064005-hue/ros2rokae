# Runtime Profiles

> 状态：Active  
> 受众：Runtime 维护者 / profile 审计人员 / 需要理解 query authority 的集成人员  
> 作用：profile、RT/NRT 边界、query authority、hardening 规则的唯一主说明  
> 上游事实来源：`launch/_launch_profile.py`、`launch/_simulation_support.py`、runtime query services、diagnostics policy  
> 最后校验：2026-04-18

## 1. Canonical profiles

| profile | 用途 | 对外级别 |
|---|---|---|
| `public_xmate6_jtc` | 默认 public 仿真主线 | public |
| `internal_full` | 内部全暴露验证 | internal |
| `daemon_hard_rt` | daemon-owned runtime 验证 | internal |

`launch_profile` 现在 **fail-fast**：未知 profile 直接报错，不再静默回退默认值。

## 2. RT / NRT split

- NRT：默认 public 主链。
- RT：只保留 install-facing 兼容接口与 internal/runtime 验证入口；Gazebo 语义仍是 simulation-grade。
- public lane 不再公开 experimental RT 示例，也不再公开 public experimental RT profile。
- strict 1kHz fail-fast RT profile 只允许存在于 internal/runtime lane。

## 3. Query authority

runtime / coordinator 是唯一权威状态面。

因此：
- `GetRuntimeStateSnapshot`、`GetRtJointData`、`GetPosture`、`GetCartPosture`、`GetJointPos`、`GetJointVel`、`GetJointTorques`、`GetBaseFrame` 等查询，都应从 `MotionRequestCoordinator -> MotionRuntime::readAuthoritativeSnapshot()` 读取或投影。
- legacy raw fetcher 不再是 query 真值源。
- diagnostics 中的 `query_authority=runtime_request_coordinator` 必须与实际读取路径一致。

## 4. Capability and hardening summary

- runtime main chain: `MoveReset -> MoveAppend -> MoveStart -> Stop(pause)`
- profile capability / diagnostics banner 必须说明当前 backend、authority 与 exposure policy
- public lane 不承诺 IO / RL / calibration
- Observability remains runtime-owned
- profile capability 查询返回 machine-readable `authority_scope / fidelity_class / model_revision`

## 5. Runtime host and provider boundary

`simulation.launch.py`、`xmate6_public.launch.py`、daemon runtime 与 Gazebo runtime 共享 `runtime_host_builder` 的 bootstrap / assembly contract。

provider 具体边界规则不再散落在 `RT_PROFILE_GUIDE.md`、`RT_HARDENING_PROFILE.md`、`PROFILE_CAPABILITY_MATRIX.md`、`PROFILE_QUERY_POLICY.md` 这类拆分页，而统一收口到：

- [`../architecture/PROVIDER_BOUNDARY.md`](../architecture/PROVIDER_BOUNDARY.md)

## 6. Related docs

- [`../architecture/ARCHITECTURE.md`](../architecture/ARCHITECTURE.md)
- [`../architecture/PROVIDER_BOUNDARY.md`](../architecture/PROVIDER_BOUNDARY.md)
- [`../release/RELEASE_GATE.md`](../release/RELEASE_GATE.md)
