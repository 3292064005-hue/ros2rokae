# P0-1 问题逐项修复摘要

本轮修复针对上一轮深度复核中明确暴露的问题，继续保持范围收口：

- 只处理 xMate 六轴 compatibility lane
- 不做 RL 语义硬化
- 不做标定链
- 不按官方基础参数回退

## 已修复问题

### 1. xCoreSDK 安装态配置仍强依赖 private ROS/Gazebo CMake 目标
- 文件：`cmake/xCoreSDKConfig.cmake.in`
- 修复：
  - 公共安装面先加载 `xCoreSDKTargets.cmake`，优先提供 `xCoreSDK::xCoreSDK_shared`
  - 仅当 private ROS/Gazebo 依赖可解析时，才加载 `xCoreSDKPrivateTargets.cmake`
  - 若 native static 不可用，则自动提供 `xCoreSDK::xCoreSDK_static -> xCoreSDK::xCoreSDK_shared` 的兼容别名
- 结果：
  - install-tree consumer 的 CMake 配置不再被 private imported targets 阻塞
  - 原有 `xCoreSDK::xCoreSDK_static` 名称仍保留

### 2. compat 目标的 public 依赖面过宽
- 文件：`cmake/targets_sdk_compat.cmake`
- 修复：
  - 移除 compat target 对 ROS/Gazebo/KDL/typesupport 的直接依赖暴露
  - compat target 只对外保留 `Eigen3::Eigen`
  - 其余实现依赖全部通过 `${PROJECT_NAME}_sdk_backend` 内聚
- 结果：
  - public xCoreSDK shared target 的导出面显著收缩

### 3. public/private export set 仍未完全分开
- 文件：`cmake/targets_packaging.cmake`
- 修复：
  - `xCoreSDK_shared` 只进入 `xCoreSDKTargets`
  - `${PROJECT_NAME}_runtime_core`、`${PROJECT_NAME}_sdk_backend`、`xCoreSDK_static` 进入 `xCoreSDKPrivateTargets`
- 结果：
  - public 安装态目标与 private 实现目标完成进一步切割

### 4. install-tree consumer 只做编译，不做可执行链检查
- 文件：`test/harness/run_install_tree_consumer.py`
- 修复：
  - staged install 后执行 consumer configure/build
  - 新增对 `minimal_connect / minimal_model / minimal_rt_header / minimal_planner` 的实际运行
  - 运行前注入 staging `lib` 到 `LD_LIBRARY_PATH` / `PATH`
- 结果：
  - install-tree gate 从“只编译”变成“编译 + 最小执行”

### 5. install-tree consumer 未显式检查 shared/static 目标存在
- 文件：`test/compat/install_tree/CMakeLists.txt`
- 修复：
  - 显式检查 `xCoreSDK::xCoreSDK_shared`
  - 显式检查 `xCoreSDK::xCoreSDK_static`
- 结果：
  - 安装包若缺 target，会在 configure 阶段直接失败

### 6. internal/backend 示例源码仍混入安装资源面
- 文件：`cmake/targets_packaging.cmake`
- 修复：
  - 不再整目录安装 `examples/`
  - 默认只安装 `examples/README.md` 和 public compat example 源文件
  - internal/backend example 源文件仅在 `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES=ON` 时安装到 `examples/internal`
- 结果：
  - 安装树不再默认暴露 internal/backend 示例源码

### 7. public 示例 18 的命名仍携带 calibration 历史包袱
- 文件：
  - `examples/cpp/18_toolset_only.cpp`
  - `examples/cpp/18_toolset_and_calibration.cpp`
  - `cmake/targets_examples.cmake`
  - `cmake/targets_tests.cmake`
  - `examples/README.md`
  - `docs/QUICKSTART.md`
- 修复：
  - 新增 `18_toolset_only.cpp` 作为 public 示例真实入口
  - `18_toolset_and_calibration.cpp` 改为 legacy source shim，仅保留源码树可发现性
  - public example target 改为 `example_18_toolset_only`
- 结果：
  - public 安装面不再继续传播 calibration 命名污染

### 8. 文档对安装态/门禁的表述过强
- 文件：
  - `README.md`
  - `docs/COMPAT_ABI.md`
  - `docs/QUICKSTART.md`
  - `examples/README.md`
  - `test/unit/test_contract_surface.cpp`
- 修复：
  - 明确 `xCoreSDK::xCoreSDK_shared` 是首选 install-facing 入口
  - 明确 `xCoreSDK::xCoreSDK_static` 可能解析为 shared-target 兼容别名
  - 明确 shared 目标在 CMake 配置时不解析 private imported targets，但运行时仍需要系统可见的 ROS2/Gazebo 动态库
  - 删除“当前默认门禁应为 16/16 全绿”这类未经当前环境验证的表述
- 结果：
  - 文档强度与证据强度重新对齐

## 仍然保留的事实边界

- 当前 compatibility lane 仍然是 ROS2/Gazebo-backed implementation
- 本轮没有把实现层彻底从 ROS2/Gazebo 剥离成原生 SDK
- 当前容器未完成目标环境实跑，因此不能把目标环境 build/ctest 说成已验证通过
