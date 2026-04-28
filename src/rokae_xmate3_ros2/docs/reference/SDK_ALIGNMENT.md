# SDK Alignment Reference

> 状态：Active  
> 受众：兼容性审计人员 / SDK 维护者  
> 作用：兼容范围、ABI 边界、对齐矩阵的参考总表  
> 上游事实来源：`docs/reference/xmate_er3_alignment_manifest.json`、compat tests、`cmake/xCoreSDKConfig.cmake.in`  
> 最后校验：2026-04-20

## 1. 单一事实源

当前对齐结论只信三类证据：

- machine-readable manifest：`docs/reference/xmate_er3_alignment_manifest.json`
- 兼容安装面与导出规则：`cmake/xCoreSDKConfig.cmake.in`、`cmake/targets_packaging.cmake`
- 契约门禁：`test/harness/check_public_contract_manifest.py`、`test/harness/check_compat_public_abi.py`

历史上的 `COMPAT_ABI.md`、`API_ALIGNMENT_MATRIX.md`、`XMATE_ER3_OFFICIAL_ALIGNMENT_MATRIX.md` 已并入本页和 `docs/public/COMPATIBILITY.md`，不再保留独立正文。

## 2. public contract 摘要

### install-facing 公开面
- headers：`include/rokae/*`
- primary install consumer target：`xCoreSDK::xCoreSDK_core`
- core-only target：`xCoreSDK::xCoreSDK_core`
- additional exported targets：`xCoreSDK::xCoreSDK_static`、`xCoreSDK::xCoreSDK_ros_bridge`
- canonical identity：`xCoreSDK:xmate_er3`

### 不属于 public contract
- `rokae_xmate3_ros2/*` private/runtime headers
- internal/full service exposure
- RL / IO / calibration / xPanel
- experimental RT examples

## 3. 行为对齐摘要

当前行为对齐必须至少同时绑定以下代码侧证据：
- `test/unit/test_move_queue_semantics.cpp`：queue / busy / reset reopen 语义
- `test/unit/test_service_facade.cpp`：`moveStart()` / path record/replay facade 语义
- `test/unit/test_runtime_state_machine.cpp`：planning / execute / settle / terminal 收敛语义
- `test/harness/check_xmate_er3_alignment_behaviors.py`：alignment 文档、manifest、CMake labels、执行 gate 入口与关键断言 token 的一致性
- `tools/run_xmate_er3_alignment_behavior_gate.sh`：在 build-capable 环境中执行 `ctest -L xmate_er3_alignment` 的强行为验收入口；默认绑定进 `run_quick_gate.sh`、`run_release_gate.sh` 和 L1 acceptance
- `src/runtime/runtime_publish_bridge.cpp` / `src/sdk/robot_motion.cpp` / `src/sdk/robot_model.cpp`：queue accepted、moveReset first、calibrateFrame not_supported 的源码行为锚点

- `MoveAppend` success = **queue accepted**
- `moveStart()` 才是 staged NRT 主链的执行提交点
- `replayPath()` = immediate-submit side-lane
- `stop()` = pause-only
- `moveReset()` = 清空队列和执行缓存
- `calibrateFrame()` 仅保留兼容签名，返回 `function_not_supported`
- `GetEndWrench` 是 public lane 首选扩展查询面
- `MoveSP`、路径录制/回放已纳入 xMateER3 public lane 的 NRT 扩展面
- preferred register path: `ReadRegisterEx` / `WriteRegisterEx`
- legacy register path: `ReadRegister` / `WriteRegister` (**Legacy facade**)
- preferred wrench path: `GetEndWrench`
- legacy wrench path: `GetEndEffectorTorque`
- RT profile label: `rt_hardened`
- RT policy label: `best_effort_non_controller_grade`

## 4. 对齐矩阵

| 区域 | 当前结论 | 说明 |
|---|---|---|
| 基础状态与工具工件 | 对齐 | public 主链已收口 |
| NRT 运动 | 对齐 | queue/start/pause 语义固定 |
| RT 兼容接口 | 部分对齐 | install-facing 保留，语义是 simulation-grade |
| 运动学 / 模型 | 仿真近似 | 见 `docs/public/KINEMATICS_AND_MODEL.md` |
| RL / IO / xPanel | 不纳入 public | 仅 internal/backend 语义 |

## 5. 相关文档

- [`../public/COMPATIBILITY.md`](../public/COMPATIBILITY.md)
- [`../public/RUNTIME_PROFILES.md`](../public/RUNTIME_PROFILES.md)
- [`../release/BUILD_RELEASE.md`](../release/BUILD_RELEASE.md)
- [`../release/ACCEPTANCE_LAYERS.md`](../release/ACCEPTANCE_LAYERS.md)

## 6. Install-tree consumer notes

- ROS2/Gazebo-backed install-facing compatibility lane
- primary install consumer: `xCoreSDK::xCoreSDK_core`
- core-only install consumer: `xCoreSDK::xCoreSDK_core`
- additional exported targets retained for ABI/runtime continuity: `xCoreSDK::xCoreSDK_static`、`xCoreSDK::xCoreSDK_ros_bridge`
- `xCoreSDK_PRIMARY_INSTALL_CONSUMER = cxx_sdk_core_consumer` must be the switch input that resolves the install-tree runtime consumer harness target selection
- install-tree runtime examples and official xMateER3 subset examples must link through the target selected from `xCoreSDK_PRIMARY_INSTALL_CONSUMER`; current resolved target is `xCoreSDK::xCoreSDK_core`
- core-only `find_package(xCoreSDK COMPONENTS core CONFIG REQUIRED)` must be independently configure/build-validated against the install tree
- core-only model / planner examples must link through `xCoreSDK::xCoreSDK_core`
- runtime execution still requires the ROS2/Gazebo-backed runtime stack
- `xCoreSDK::xCoreSDK_static` is a native static library
- `rokae/sdk_shim*.hpp` is not installed as part of the public SDK surface
- compatibility gates include `check_public_contract_manifest.py` and `check_compat_public_abi.py`


- `compatibility_alias_policy`：`canonical_plus_compat` / `canonical_only` / `legacy_only`，默认导出为 `canonical_plus_compat`。
