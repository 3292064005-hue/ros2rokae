# Runtime State Machine

> 状态：Active  
> 受众：runtime / planner / executor / 诊断维护者  
> 作用：主链状态机与 lifecycle event 的唯一说明  
> 上游事实来源：`src/runtime/runtime_state_machine.*`、`src/runtime/runtime_types.hpp`、`src/runtime/runtime_execution_switch.cpp`、`test/unit/test_runtime_state_machine.cpp`  
> 最后校验：2026-04-18

## 1. Authority rule

`RuntimeStateMachine` 是请求生命周期状态的唯一权威收敛点。

planner、executor、watchdog、owner arbitration 只能通过 `RuntimeEvent` 驱动状态变化，不能各自私自改写最终对外状态语义。

补充约束：
- executor 允许报告 **observed execution state**，但只能通过 `RuntimeEvent{progress_updated}` 上送；
- `runtime_execution_switch.cpp` 不允许直接改写 `RuntimeStatus.state / message / completed_segments / current_segment_index`；
- `settling`、segment blend 期间的 `queued` 也必须通过状态机事件显式暴露，不能以“先 apply 再覆盖 state”的方式旁路。

## 2. Execution states

- `idle`
- `planning`
- `queued`
- `executing`
- `paused`
- `settling`
- `completed`
- `completed_relaxed`
- `failed`
- `stopped`

## 3. Runtime phases

当前 coarse runtime phase 以 `runtime_types.hpp` 为准，主线至少覆盖：
- `idle`
- `planning`
- `executing`
- `faulted`

progress event 如携带 observed state，则 phase 收敛规则固定为：
- `planning -> planning`
- `queued / executing / settling -> executing`
- `paused / stopped / completed / completed_relaxed -> idle`
- `failed -> faulted`

## 4. Lifecycle events

- `reset`
- `request_queued`
- `planning_requested`
- `planning_rejected`
- `plan_queued`
- `execution_started`
- `progress_updated`
- `trajectory_retimed`
- `paused`
- `watchdog_triggered`
- `completed`
- `completed_relaxed`
- `failed`
- `stopped`
- `owner_changed`
- `phase_override`

## 5. Main-chain semantics

主链语义固定为：

`request_queued -> planning_requested -> plan_queued -> execution_started -> progress_updated -> completed|completed_relaxed|failed|stopped`

补充规则：
- `planning_rejected` 与 `watchdog_triggered` 直接进入 `failed / faulted`
- `paused` 会把 coarse phase 收敛回 `idle`，但 execution state 保持 `paused`
- terminal event 若未携带 backend，则保留当前 backend，不允许把 backend 信息静默擦除
- executor 进入 `settling` 必须通过 `progress_updated(has_observed_state=true, observed_state=settling)` 体现
- segment blend 进入 `queued` 也必须通过 `progress_updated(has_observed_state=true, observed_state=queued)` 体现

## 6. Observability contract

以下信息必须能从 runtime 状态或诊断面观察到：
- `request_id`
- `execution_state`
- `runtime_phase`
- `execution_backend`
- `completed_segments / total_segments`
- `last_event`
- `terminal_success`

## 7. Enforcement

静态 gate 至少必须验证：
- `runtime_state_machine` source / doc / manifest 一致
- `runtime_execution_switch.cpp` 不存在直接写 `active_status_.state / message / completed_segments / current_segment_index` 的旁路
- `test/unit/test_runtime_state_machine.cpp` 覆盖 planning → queue → execute → settle/complete 的关键收敛语义

## 8. Related docs

- [`RECORDED_PATH_SCHEMA.md`](RECORDED_PATH_SCHEMA.md)
- [`../architecture/ARCHITECTURE.md`](../architecture/ARCHITECTURE.md)
- [`SDK_ALIGNMENT.md`](SDK_ALIGNMENT.md)
- [`../release/ACCEPTANCE_LAYERS.md`](../release/ACCEPTANCE_LAYERS.md)
