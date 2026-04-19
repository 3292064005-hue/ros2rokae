# Release Gate

> 状态：Active  
> 受众：发布维护者 / CI 维护者  
> 作用：release gate 的唯一执行说明  
> 上游事实来源：`.github/workflows/acceptance-humble-gazebo11.yml`、`tools/run_release_gate.sh`、`tools/run_target_env_acceptance.sh`  
> 最后校验：2026-04-18

## 1. 默认门禁

锁环境 release gate 现在是默认 CI 约束：

- workflow：`.github/workflows/acceptance-humble-gazebo11.yml`
- 触发：`push` / `pull_request`
- 目标环境：Ubuntu 22.04 + ROS 2 Humble + Gazebo 11
- 必须执行：`--release-gate`

## 2. release gate 组成

- environment preflight：`tools/check_target_environment.sh`
- static sanity：`tools/run_static_sanity.sh`
- install-tree consumers
- launch smoke
- locked target-environment acceptance
- layered acceptance ownership matrix (`docs/release/ACCEPTANCE_LAYERS.md`)
- xMate6 alignment behavior gate (`tools/run_xmate6_alignment_behavior_gate.sh`, default in quick/release gate and L1 acceptance)

release gate wrappers are mirrored into the install-tree artifact for discoverability, but they remain workspace-source wrappers because static sanity / ctest operate on the source tree.

## 3. 本地入口

```bash
tools/run_release_gate.sh
tools/run_release_gate_portable.sh
tools/run_target_env_acceptance.sh --release-gate --launch-smoke
```

## 4. 结果解释

### 已静态确认
仅说明：
- 文档/manifest/gate 一致
- 脚本与配置存在且语法正确

### 已真实环境验证
必须至少包含：
- `colcon build`
- `ctest -L release_gate`
- install-tree consumer
- launch smoke

未完成真实环境验证时，禁止把静态检查写成“已完全可交付”。

## 5. 相关文档

- [`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- [`ENVIRONMENT_LOCK.md`](ENVIRONMENT_LOCK.md)

## 6. Acceptance layers

L0-L5 的唯一矩阵见 [`ACCEPTANCE_LAYERS.md`](ACCEPTANCE_LAYERS.md)。
release gate 只能覆盖到 L0-L2；其中 L1 默认包含 `xmate6_alignment` 行为验收 bundle。L3-L5 需要真实 runtime namespace / 传感器 / 明确动作审批。

