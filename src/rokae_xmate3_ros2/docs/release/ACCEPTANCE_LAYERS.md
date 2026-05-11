# Acceptance Layers

> 状态：Active  
> 受众：发布维护者 / 测试维护者 / 实机联调人员  
> 作用：确认方案中的 L0-L5 分层验收体系唯一说明  
> 上游事实来源：`tools/run_acceptance_layers.sh`、`tools/run_real_dryrun_acceptance.sh`、`tools/run_loaded_sensor_acceptance.sh`、`tools/run_full_task_acceptance.sh`、`docs/release/acceptance_layers_manifest.json`  
> 最后校验：2026-04-18

## 1. Layer matrix

| 层级 | 目标 | 默认入口 | 环境要求 |
|---|---|---|---|
| L0 | unit / contract correctness | `tools/run_full_source_tree_build_gate.sh` + `ctest -L quick_gate` | locked build/test env |
| L1 | simulated semantic closure | `ctest -L semantic_gate` | build/test env |
| L2 | Gazebo / launch main-chain smoke | `tools/run_launch_smoke.sh` + `tools/run_main_chain_smoke.sh` | Ubuntu 22.04 + ROS2 Humble + Gazebo11 |
| L3 | real dry-run acceptance | `tools/run_real_dryrun_acceptance.sh` | real runtime namespace + robot endpoint |
| L4 | loaded sensor acceptance | `tools/run_loaded_sensor_acceptance.sh` | L3 + external sensor topic(s) |
| L5 | full task acceptance | `tools/run_full_task_acceptance.sh` | L4 + explicit motion approval |

## 2. No-skip rule

禁止越级：
- L2 失败或未执行时，不允许宣称 L3/L4/L5 通过
- L3 未执行时，不允许宣称 L4/L5 通过
- L4 未执行时，不允许宣称 L5 通过
- 只有“已通过的最高层级”可以用于外部交付表述

## 3. Execution rules

### L0
- 关注算法/状态机/契约单元层正确性
- 不得用 launch smoke 替代

### L1
- 关注 simulated/runtime semantic closure
- 必须覆盖 queue/start/pause/stop/replay 等主链语义

### L2
- 关注 canonical launch / install-tree / Gazebo main chain
- 必须包含 launch discovery 与 main-chain smoke

### L3
- 只允许 dry-run，不允许隐式运动
- 至少覆盖 connect / mode query / diagnostics / disconnect

### L4
- 要求外部传感器 topic heartbeat 可见
- 允许 start/stop/save record，但不要求 full task motion

### L5
- 要求显式运动审批环境变量 `ROKAE_ACCEPTANCE_ALLOW_MOTION=1`
- 必须记录 action/service log，并对 runtime diagnostics 做收尾核验

## 4. Entrypoints

- `tools/run_acceptance_layers.sh`
- `tools/run_real_dryrun_acceptance.sh`
- `tools/run_loaded_sensor_acceptance.sh`
- `tools/run_full_task_acceptance.sh`
- `tools/run_launch_smoke.sh`
- `tools/run_main_chain_smoke.sh`

install-tree public artifact 会同步镜像这些入口到 `share/rokae_xmate3_ros2/tools/`，且这些入口必须通过 sibling tool resolution 与 installed config/helper 直接工作，不能硬编码回 `src/rokae_xmate3_ros2/tools/*`。L1 默认还必须执行 `run_xmate_er3_alignment_behavior_gate.sh`。

## 5. Related docs

- [`RELEASE_GATE.md`](RELEASE_GATE.md)
- [`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- [`../reference/RUNTIME_STATE_MACHINE.md`](../reference/RUNTIME_STATE_MACHINE.md)


## Full source gate execution boundary

The non-replay full source-tree gate is mandatory for target-environment release acceptance and must be run in the locked Ubuntu 22.04 / ROS 2 Humble / Gazebo environment. Passing replay-only smoke or static contract checks is not a substitute for this target-environment gate, and real hardware validation remains a separate layer.

## Evidence rule

`status.full_source_gate` is the release evidence field for the non-replay source-tree build gate. Target-environment acceptance scripts verify this field before returning success; missing or `not_run` evidence is treated as failure, not as an unknown/pass state. A verified report must also reference report-local evidence logs through `artifacts.expected_logs`, and the full-source evidence log must contain `full-source-build-gate: passed`. The verifier also rejects reports that do not identify `ROS_DISTRO=humble` or that mark required ROS/Gazebo tools as unavailable.
