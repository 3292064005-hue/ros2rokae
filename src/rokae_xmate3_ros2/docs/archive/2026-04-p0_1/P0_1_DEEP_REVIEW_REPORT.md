# P0-1 二次精修后的全量深度复核报告

## 一、方案落实映射表

| 优先级 | 方案项编号 | 对应代码文件 | 对应函数 / 类 / 模块 | 已落实内容 | 未落实内容 | 风险残留 |
|---|---|---|---|---|---|---|
| P0 | P0-EX-1 导出链分离 | `cmake/targets_packaging.cmake`, `cmake/xCoreSDKConfig.cmake.in` | `xCoreSDKTargets`, `xCoreSDKPrivateTargets` | compat/public export 与 private/backend export 已拆分；`xCoreSDKConfig` 先加载 private targets 再加载 public targets | `xCoreSDKConfig` 仍需解析 ROS2/Gazebo 依赖，尚未做到“仅 Eigen 公共依赖即可完成包解析” | **Major**：安装面仍是 ROS2/Gazebo-backed |
| P0 | P0-EX-2 compat target 依赖面收口 | `cmake/targets_sdk_compat.cmake` | `xCoreSDK_static`, `xCoreSDK_shared` | `Eigen3::Eigen` 作为 public 依赖；backend/runtime/typesupport/KDL/Gazebo 已改为 private link | 静态库消费时仍需 private target 解析链，无法完全宣称“无 backend implementation dependency” | **Major** |
| P0 | P0-EX-3 install-tree consumer 真实入口 | `test/compat/install_tree/CMakeLists.txt`, `test/compat/install_tree/*.cpp`, `test/harness/run_install_tree_consumer.py`, `cmake/targets_tests.cmake` | staged install consumer harness | install-tree skeleton 已覆盖 `connect/model/rt_header/planner`；CTest 已新增 `compat_install_tree_consumer` | 当前容器未完成真实 configure/build 实跑验证 | **Major（环境验证缺失）** |
| P0 | P0-EX-4 exported symbol gate | `test/harness/check_exported_symbols.py`, `cmake/targets_tests.cmake` | 动态符号表检查 | 已改为检查 **dynamic/exported symbol table**，避免把本地实现符号误判为 ABI 泄漏 | 尚未在真实构建产物 `.so` 上执行本 gate | **Major（环境验证缺失）** |
| P0 | P0-EX-5 public/internal 示例分层 | `cmake/targets_examples.cmake`, `examples/README.md` | `ROKAE_PUBLIC_COMPAT_EXAMPLES`, `ROKAE_INTERNAL_BACKEND_EXAMPLES` | 示例已分为 public compat 与 internal/backend 两组 | `examples/` 源码目录仍整体安装到 `share/`，因此 internal 示例源码仍会出现在安装树的文档资源中 | **Minor**：源码资源可见，但默认不安装 internal example 二进制 |
| P0 | P0-EX-6 public example 越界项剥离 | `examples/cpp/09_advanced_sdk_compat.cpp`, `examples/cpp/10_sdk_workflow_xmate3.cpp`, `examples/cpp/18_toolset_only.cpp` | 示例主流程 | 已去除 public 示例中的 RL / 标定 / internal service 词面与调用链污染；`18` 收口为 toolset-only | 文件名 `18_toolset_only.cpp` 保留历史名，仍带“calibration”字样 | **Minor**：名称历史包袱，不影响调用链 |
| P0 | P0-EX-7 internal example 默认不随安装面导出 | `CMakeLists.txt`, `cmake/targets_packaging.cmake`, `README.md`, `docs/*` | `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES` | 默认仅安装 public compat example 二进制；internal/backend example 需显式 `ON` | 无 | Low |
| P1 | P1-RL 语义硬化 | —— | —— | 本轮按确认方案 **不实施** | RL façade 仍存在，仅未作为 public compat 示例入口 | Out of scope |
| P2 | P2-标定 / 多机型 / 基础参数对齐 | —— | —— | 本轮按确认方案 **不实施** | `calibrateFrame` 兼容签名仍保留，不提供真实能力 | Out of scope |

## 二、问题清单

### Blocker

无。当前复核后未发现必须阻塞交付、且已经确认未修复的缺陷。

### Critical

无。当前复核后未发现会直接导致 ABI 面声明/定义断裂或 public/internal 示例分层失效的未修复缺陷。

### Major

#### 1. `xCoreSDKConfig.cmake` 仍然依赖 ROS2/Gazebo 目标环境
1. 问题描述：compat 安装面虽然完成了 public/private export set 分离，但配置包解析仍需要 `ament_index_cpp`、`rclcpp`、`gazebo_ros` 等依赖。
2. 触发条件：在没有与本包一致的 ROS2/Gazebo 环境中执行 `find_package(xCoreSDK CONFIG REQUIRED)`。
3. 影响范围：install-tree 外部 consumer；`xCoreSDKConfig.cmake.in`；README/QUICKSTART 文档语义。
4. 根因判断：compat layer 仍是 ROS2/Gazebo-backed implementation；private targets 需要这些依赖完成 imported target 解析。
5. 修复建议：若要完全去 ROS 化，需要把 compat static/shared 的实现进一步内聚，避免通过 private target 链解析 backend/runtime imported targets。

#### 2. install-tree consumer / exported-symbol gate 在当前容器未完成真实执行验证
1. 问题描述：代码和 harness 已补齐，但当前复核环境未具备完整 ROS2/Gazebo 构建运行栈，无法在本容器内完成 staged install consumer 构建与 `.so` 动态符号 gate 实跑。
2. 触发条件：要求在当前容器直接给出“已实跑通过”的结论时。
3. 影响范围：`compat_install_tree_consumer`、`compat_exported_symbols` 的最终置信度。
4. 根因判断：环境约束，不是源码语法或调用链直接缺陷。
5. 修复建议：在目标 Humble + Gazebo11 环境中执行 `ctest -L abi_gate` 或 release gate。

### Minor

#### 3. `examples/` 目录作为文档资源整体安装，internal 示例源码仍会出现在安装树分享资源中
1. 问题描述：虽然 internal/backend 示例二进制默认不安装，但 `install(DIRECTORY ... examples ...)` 仍会把源码示例目录整体复制到 share 目录。
2. 触发条件：用户检查安装树 `share/${PROJECT_NAME}/examples`。
3. 影响范围：安装树文档/资源可见性，不影响编译链接。
4. 根因判断：当前文档资源安装策略采用目录整体复制。
5. 修复建议：后续可把 public compat examples 与 internal examples 的源码资源也做目录级拆分安装。

#### 4. `18_toolset_only.cpp` 文件名保留历史命名
1. 问题描述：文件内容已收口为 toolset-only，但文件名仍包含 `calibration`。
2. 触发条件：按文件名搜索示例时。
3. 影响范围：认知负担；不影响调用链和 ABI。
4. 根因判断：为兼容现有索引/引用保留历史文件名。
5. 修复建议：后续如允许调整示例索引，可改名为 `18_toolset_only.cpp` 并同步 target/doc alias。

## 三、Checklist 逐项结论

### 1. [ ] 方案覆盖率
**否。**
- P0 的“导出链、消费链、示例链”主体均已落实。
- 但 `xCoreSDKConfig.cmake` 仍未达到“只保留 public header 依赖即可完成配置包解析”的理想目标；当前只能诚实表述为 **ROS2/Gazebo-backed install-facing compatibility lane**。
- 受影响范围：install-tree 下游 consumer 环境前提、README/QUICKSTART/COMPAT_ABI 文档语义。

### 2. [ ] 边界测试
**否（未做到全覆盖实跑）。**
- 已覆盖/补强：
  - 配置缺失：`xCoreSDKConfig.cmake.in` 条件加载 `xCoreSDKPrivateTargets.cmake`
  - 依赖失败：`find_dependency(...)` 明确失败点
  - 资源不存在：`check_exported_symbols.py` 对缺失库文件直接报错
  - 动态符号检查口径：从全符号表改为动态导出符号表
- 未在本容器完成实跑覆盖：
  - 超时/构建失败：`run_install_tree_consumer.py` 的 staged install consumer 仍缺目标环境实跑
  - 依赖失败：未在缺失 ROS2/Gazebo 环境中做负向执行验证
  - 资源释放/清理失败：consumer harness 使用 `shutil.rmtree` 清理，但未做异常恢复测试
- 风险等级：Major（环境实跑缺失）

### 3. [ ] 兼容性
**是（无已知阻塞性回归）。**
理由：
- 接口签名：本轮未改 public ABI 签名，仅调整 CMake/export/test/docs 与示例范围。
- 返回结构：未变更。
- 调用时序：未改核心 runtime/RT/NRT 调用顺序。
- 默认值行为：新增 `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES=OFF`，是安装面收口，不破坏旧功能旁路；internal 示例仍可显式开启安装。
- 配置兼容：`ROKAE_BUILD_COMPAT_SDK`、`ROKAE_INSTALL_BACKEND_DEV_HEADERS` 保持不变。
- 错误处理行为：`check_exported_symbols.py` 判定口径修正为 exported symbols，更接近设计目标，不会误伤内部实现符号。
- 旧逻辑旁路：internal/backend examples 仍保留；只是不默认安装二进制。

### 4. [ ] 环境一致性
**否。**
- README/QUICKSTART/COMPAT_ABI 现已同步说明：compat 安装面仍是 ROS2/Gazebo-backed。
- 但“当前容器已完成目标环境构建/运行验证”这一点不能成立。
- 偏离位置：当前审查容器不是完整 Humble + Gazebo11 + ament/colcon 实跑环境。
- 影响范围：无法在本次回复中把 install-tree consumer 和 exported symbol gate 说成“已实跑通过”。
- 影响等级：Major。

## 四、最终结论

- **当前版本是否达到可交付标准：**
  - **达到“可提交目标环境复测”的交付标准**。
  - **未达到“已在本容器完成目标环境实跑验收”的最终关闭标准**。

- **是否仍存在已知风险：**
  - 是。

- **风险性质：**
  - `xCoreSDKConfig.cmake` 仍依赖 ROS2/Gazebo 目标环境：**可接受风险（已文档化）**
  - install-tree consumer 与 exported-symbol gate 未在当前容器实跑：**可接受但必须在目标环境补验的风险**
  - internal 示例源码仍作为资源出现在安装树：**可接受风险**

