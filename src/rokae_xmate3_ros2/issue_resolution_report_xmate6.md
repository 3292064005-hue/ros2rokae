# xMate3 六轴问题逐项解决报告

## 已解决项

1. **native SDK facade `lastErrorCode()` 全入口统一回写**
   - 新增 `ScopedLastError` 作用域记录器
   - 覆盖 `src/sdk/*.cpp` 中所有公开 `std::error_code& ec` 入口以及关键 `Impl` 辅助入口
   - 结果：`lastErrorCode()` 不再只在局部 RT 链可见

2. **xMate3 六轴 RT 层移除 `psi_* / elbow*` 占位字段**
   - 删除 `src/runtime/rt_field_registry.cpp` 中 synthetic placeholder 描述
   - 删除 `src/sdk/robot_rt.cpp` 与 `include/rokae/detail/sdk_shim_core.hpp` 中对 `elbow*` 的缓存分支
   - 更新 `examples/cpp/17_state_stream_cache.cpp`，不再请求 `RtSupportedFields::elbow_m`

3. **文档与契约检查同步**
   - README / IMPLEMENTATION_AUDIT / RT_HARDENING_PROFILE / EXTENSION_FRAMEWORK 已同步
   - `test/unit/test_contract_surface.cpp` 新增文本契约检查
   - `test/harness/check_repo_contract.py` 新增仓库级静态约束检查

## 本地可执行校验

- `python3 test/harness/check_repo_contract.py`：通过
- 自定义静态校验：
  - 所有 `src/sdk/*.cpp` 中带 `std::error_code& ec` 的公开入口均已加入 `track_last_error(...)`
  - xMate3 六轴 RT registry / native RT cache / shim RT cache / state-stream example 中均已不存在 `elbow` placeholder 暴露

## 仍然不属于仓库内可直接代码修复的问题

- 当前容器没有完整 ROS2 Humble + ament_cmake 目标环境，因此仍无法在本容器完成目标环境联编验收。
- 这不是当前仓库代码缺陷，但仍建议在锁定环境中再执行一次 `colcon build` + launch smoke。
