# P0-1 二次精修实施摘要

## 已落地方案项
- 导出链：将 `xCoreSDKTargets` 与 `xCoreSDKPrivateTargets` 分离，兼容安装面只公开 compat targets。
- 消费链：补充 staged install + install-tree consumer harness，新增 shared symbol leak 检查与 planner 最小消费者工程。
- 示例链：将 examples 分成 public compat 与 internal/backend 两组；移出依赖 internal service/RL 的 public 示例；将 `18_toolset_only.cpp` 收口为 toolset-only。

## 主要改动文件
- `cmake/targets_sdk_compat.cmake`
- `cmake/targets_packaging.cmake`
- `cmake/xCoreSDKConfig.cmake.in`
- `cmake/targets_examples.cmake`
- `cmake/targets_tests.cmake`
- `examples/README.md`
- `examples/cpp/09_advanced_sdk_compat.cpp`
- `examples/cpp/18_toolset_only.cpp`
- `docs/COMPAT_ABI.md`
- `docs/QUICKSTART.md`
- `README.md`
- `test/compat/install_tree/CMakeLists.txt`
- `test/compat/install_tree/minimal_planner.cpp`
- `test/harness/run_install_tree_consumer.py`
- `test/harness/check_exported_symbols.py`
- `test/harness/check_compat_public_abi.py`
- `test/unit/test_contract_surface.cpp`

## 兼容性处理
- public compat examples 默认只走 compat facade；internal/backend examples 继续保留源码树验证能力。
- `xCoreSDKConfig.cmake` 先加载 private targets，再加载 public compat targets，以保证现有 ROS2/Gazebo-backed 实现仍可解析其内部依赖。
- 未把 RL/标定重新引入 public compat contract。

## 异常边界与说明
- 本轮没有扩展 RT 真机级契约；只收口 install/build/example 边界。
- `xCoreSDKConfig.cmake` 仍需解析 ROS2/Gazebo 私有实现依赖；这反映当前实现仍是 ROS-backed compatibility layer，而不是完全独立于 ROS 的原生 SDK 二进制。
