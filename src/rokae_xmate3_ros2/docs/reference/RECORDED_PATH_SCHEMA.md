# Recorded Path Schema

> 状态：Active  
> 受众：runtime / replay / 数据契约维护者  
> 作用：路径录制与回放资产的唯一 schema 说明  
> 上游事实来源：`src/runtime/runtime_snapshots.hpp`、`src/runtime/program_state.*`、`src/runtime/path_facade.cpp`、`src/runtime/request_adapter.cpp`  
> 最后校验：2026-04-18

## 1. 适用范围

该 schema 描述 **xMateER3 public compatibility lane** 下的路径录制/回放资产最小消费契约。

明确不描述：
- RL 数据
- IO / 寄存器
- 标定资产
- controller-grade 实机采集承诺

## 2. Metadata contract

`ReplayPathAssetMetadata` 当前字段：

- `version = v2`
- `robot = xMateER3`
- `robot_model = xMateER3`
- `canonical_identity = xCoreSDK:xmate_er3`
- `source = <producer label>`
- `created_at_sec = <absolute capture timestamp of first accepted sample>`
- `monotonic_step_sec = 0.01`

说明：
- `robot` 表示 public lane 家族身份，不再把 source-tree legacy 包名或 Gazebo model 名称当成 replay schema 的主身份。
- `robot_model` 固定为 canonical ER3 模型身份，不再把 legacy alias 或底层仿真资源名称写入 replay schema。
- `canonical_identity` 是 install-facing 契约锚点。
- `v1 -> v2` 的兼容策略是：消费侧允许读取 `v1`，并在归一化阶段补齐默认字段后按 `v2` 解释。

## 3. Sample contract

每个 `RecordedPathSample` 最少包含：

- `time_from_start_sec`
- `joint_position[6]`
- `joint_velocity[6]`
- `has_end_pose`
- `end_pose[6]`
- `has_contact_force`
- `contact_force`
- `contact_established`
- `image_frame_id`
- `task_phase`
- `source_id`

规则：
- 只允许 6 轴 joint sample
- 时间必须严格单调递增
- 外部时间戳缺失或倒退时，runtime 必须用 `monotonic_step_sec` 进行补偿
- `task_phase` 与 `source_id` 不允许在消费链路中缺失；legacy `v1` 资产进入消费前必须补齐默认值
- `end_pose / contact_force / image_frame_id` 可以在录制阶段为空缺，但 analysis / report 只有在每个 sample 都具备这些字段时才会被判定为 ready

## 4. Tooling provenance

`ReplayPathAsset` 附带：
- tool / wobj / base frame
- tool mass / center of mass
- source label

replay 入口在提交前必须检查 recorded toolset 与当前 active toolset 的兼容性。

## 5. Behavioral contract

- `startRecordingPath()`：重置捕获缓冲区并开始录制
- `stopRecordingPath()`：停止继续写入，但保留缓冲区用于保存
- `cancelRecordingPath()`：停止并丢弃缓冲区
- `saveRecordedPath(name)`：把当前 capture buffer 封装成 replay asset；core 层必须拒绝空 name、空 buffer 和 replay 契约校验失败的资产，analysis/report readiness 作为独立消费判定保留在资产报告中
- `ReplayPath`：只接受 NRT 兼容模式下的 replay 请求
- `request_adapter`：必须接受 `v1` / `v2`，拒绝未知 schema version，并在 replay 前通过统一 contract report 校验

## 6. Consumer contract

当前统一消费入口包括：
- replay retimer / replay request adapter
- runtime path facade replay preflight
- `buildReplayPathConsumptionReport()` / `validateReplayPathAssetForConsumption()`
- `buildReplayPathAnalysisInput()`：把同一 asset 归一化成 analysis 视图
- `buildReplayPathReportSummary()`：把同一 asset 归一化成 report 汇总视图
- `PathFacade::handleSaveRecordPath()`：把消费报告附着到 save 响应，暴露 analysis/report readiness
- `MotionRequestCoordinator::submitReplayPath()`：把 report summary 附着到 replay 提交结果
- contract/unit tests
- 文档/manifest/static gates

说明：
- replay readiness 只要求 replay 最小契约成立。
- analysis readiness 只有在每个 sample 都具备 `end_pose / contact_force / image_frame_id / task_phase / source_id` 时才为真。
- report readiness 建立在 analysis readiness 之上，并输出统一 `ReplayPathReportSummary`。
- 当前公共兼容面仍不承诺 controller-grade 的影像或压力业务解释，但 replay / analysis / report 不再分裂为不同数据结构。

## 7. Related docs

- [`RUNTIME_STATE_MACHINE.md`](RUNTIME_STATE_MACHINE.md)
- [`SDK_ALIGNMENT.md`](SDK_ALIGNMENT.md)
- [`../public/COMPATIBILITY.md`](../public/COMPATIBILITY.md)
- [`../release/ACCEPTANCE_LAYERS.md`](../release/ACCEPTANCE_LAYERS.md)
