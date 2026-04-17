# Runtime Profiles

> 状态：Active  
> 受众：Runtime 维护者 / profile 审计人员 / 需要理解 query authority 的集成人员  
> 作用：profile、RT/NRT 边界、query authority、hardening 规则的唯一主说明  
> 上游事实来源：`launch/_launch_profile.py`、`launch/_simulation_support.py`、runtime query services、diagnostics policy  
> 最后校验：2026-04-17

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

## 3. Query authority

### Rule
runtime / coordinator 是唯一权威状态面。

### Consequence
- `GetRuntimeStateSnapshot`、`GetRtJointData`、`GetPosture`、`GetCartPosture`、`GetJointPos`、`GetJointVel`、`GetJointTorques`、`GetBaseFrame` 等查询，都应从 `MotionRequestCoordinator -> MotionRuntime::readAuthoritativeSnapshot()` 读取或投影。
- legacy raw fetcher 不再是 query 真值源。
- diagnostics 中的 `query_authority=runtime_request_coordinator` 必须与实际读取路径一致。

## 4. Hardening rules

- runtime main chain: `MoveReset -> MoveAppend -> MoveStart -> Stop(pause)`
- profile capability / diagnostics banner 必须能说明当前 backend、authority 与 exposure policy
- public lane 不承诺 IO / RL / calibration
- strict 1kHz fail-fast RT profile 只允许存在于 internal/runtime lane

## 5. Runtime host

`simulation.launch.py`、`xmate6_public.launch.py`、daemon runtime 与 Gazebo runtime 现在共享 `runtime_host_builder` 的 bootstrap / assembly contract。host lifecycle 已尽量收口，但运行时仍是 ROS2/Gazebo-backed。

## 6. Related docs

- [`ARCHITECTURE.md`](ARCHITECTURE.md)
- [`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- [`QUICKSTART.md`](QUICKSTART.md)
