# ROKAE xMate3 ROS2 仿真 SDK

> 基于 xCore SDK C++ API 形状的 ROS2 Gazebo 仿真功能包

[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-brightgreen)](https://docs.ros.org/en/humble/)
[![Gazebo 11](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)

## 目录
- [功能特性](#功能特性)
- [文件结构](#文件结构)
- [快速开始](#快速开始)
- [ABI 兼容安装面](#abi-兼容安装面)
- [示例程序](#示例程序)
- [API 参考](#api-参考)
- [架构说明](#架构说明)
- [ROS2 接口](#ros2-接口)
- [扩展框架](#扩展框架)
- [实现审计](#实现审计)
- [常见问题](#常见问题)

## 功能特性

### xCore SDK C++ API 兼容支持

状态标记说明：
- `已对齐`：接口形状与 Gazebo 仿真语义都已经收口到默认实现
- `近似实现`：接口可用，但内部是仿真近似，不等价于真机控制器
- `实验性`：接口或语义仍在收口，适合验证链路，不适合当精度保证
- `不支持`：当前版本明确不提供

| 章节 | 功能 | 状态 |
|------|------|------|
| **4.3** | 机器人基本操作及信息查询 | 已对齐 |
| **4.4** | 非实时运动控制接口 | 已对齐 |
| **4.5** | 实时控制接口 | 实验性 |
| **4.6** | IO 与通信接口 | 已移出安装态 public xMate6 contract（内部保留） |
| **4.7** | RL 工程接口 | 已移出安装态 public xMate6 contract（内部保留近似实现） |
| **4.8** | 协作机器人专属接口 | 近似实现 |
| **8.3.7** | 路径规划类 | 近似实现 |
| **8.3.8** | 模型库 | 近似实现 |
| **8.3.9** | 工具函数 | 已对齐 |

完整示例覆盖范围见 [examples/README.md](examples/README.md)。

> 当前版本明确不支持任何坐标系标定功能；`calibrateFrame()` 仅保留兼容签名，并始终返回 `std::errc::function_not_supported`。
> 目标环境预检请先运行 `tools/check_target_environment.sh`；若刻意偏离 Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 锁定环境，需显式设置 `ROKAE_IGNORE_ENV_LOCK=1`。

更严格的当前实现面判断见 [docs/IMPLEMENTATION_AUDIT.md](docs/IMPLEMENTATION_AUDIT.md)，剩余硬化任务见 [docs/HARDENING_BACKLOG.md](docs/HARDENING_BACKLOG.md)。

当前 quick/release gate 已额外执行 `semantic_gate`，用于拦截“接口成功但 runtime 不生效”“RT 再次退回 NRT 主链”“catalog provenance 丢失”等语义回退。

### 运动能力
- RT 字段注册 / 订阅计划 / 预启动检查 / watchdog 诊断（仿真级）
- `MoveAbsJ`
- `MoveJ`
- `MoveL`
- `MoveC`
- `MoveCF`
- `MoveSP`
- Jog / 路径录制 / 路径回放
- RT 位置 / 阻抗 / 跟随 / S 线 / 力矩示例（实验性，默认走 runtime-owned direct RT command channel；仍非控制柜级 1ms 契约）
- RT `MoveC` 现已在 xMate 六轴 compatibility lane 上落地为**几何圆弧 RT 兼容实现**：显式做起点校验、退化圆检测、圆弧插补与姿态球面插值；仍然属于 simulation-grade direct RT command channel，而非控制柜级 UDP/1 kHz 保证

### 运动学与模型
- 正运动学 (FK)
- 逆运动学 (IK)
- 雅可比矩阵（近似实现，默认优先 KDL/URDF 后端）
- 关节/笛卡尔速度与加速度（近似实现）
- 力矩估计与无摩擦力矩（近似实现）

### 兼容扩展
- `rokae/robot.h`、`rokae/data_types.h`、`rokae/model.h`、`rokae/planner.h`、`rokae/motion_control_rt.h`
- `rokae/utility.h` 与官方示例同名工具头
- `/xmate3/io/*` 与 `/xmate3/cobot/*` 双命名空间兼容
- 寄存器、RL、奇异位规避等接口仅保留源码兼容符号；安装态 public xMate6 lane 不再承诺这些能力
- runtime-backed 寄存器命名空间描述（内部骨架）
- 扩展主接口优先策略：`GetEndWrench` 仍是 public xMate6 lane 的首选扩展接口；`ReadRegisterEx / WriteRegisterEx / GetRlProjectInfo / SetXPanelVout` 仅保留 backend/internal 语义与源码兼容 façade
- `moveStart()` 现在先检查连接态与公开运动模式，再刷新本地缓存命令，避免断连场景误消费 queued cache
- `enableDrag()` 现在严格对齐官方示例前置：需要 **manual 模式 + 下电状态**，不再允许上电拖动伪成功
- `ReadRegister* / WriteRegister* / SetXPanelVout / SendCustomData` 继续保留 backend/internal runtime facade；安装态 public xMate6 lane 对前 3 类入口返回 deterministic `not_implemented`。
- `replayPath()` 现在要求名称非空、速率位于 `(0, 3)`、当前 motion mode 处于 NRT 兼容态，且当前 toolset 上下文与录制时一致，避免把路径回放当成隐式 toolset 覆盖入口
- 兼容别名 `/xmate3/cobot/get_joint_torque`、`/xmate3/cobot/get_end_torque` 保留不变，底层类型统一到 `GetJointTorques` / `GetEndEffectorTorque`
- 内建 `/xmate3/internal/validate_motion`、`/xmate3/internal/get_runtime_diagnostics` 和 `/xmate3/internal/runtime_status` 用于预验证与运行时诊断


## ABI 兼容安装面

P0-1 之后，安装态对外 ABI 被收口到单独的 xCoreSDK compatibility lane：

- 安装目标：`xCoreSDK::xCoreSDK_static` 与 `xCoreSDK::xCoreSDK_shared` 现在都是真实 install-facing 目标；其中 `xCoreSDK::xCoreSDK_static` 打包了 compatibility façade 与 runtime/backend 对象代码，名称与官方 static lane 保持一致
- 导出策略：public compat targets 统一通过 `xCoreSDKTargets.cmake` 导出；backend/runtime targets 仍可通过 `xCoreSDKPrivateTargets.cmake` 按需暴露，但不属于 public contract
- 公共头：`rokae/robot.h`、`rokae/model.h`、`rokae/motion_control_rt.h`、`rokae/planner.h`、`rokae/data_types.h`、`rokae/utility.h`
- 范围：**仅 xMate 六轴 compatibility lane**
- 目标：让外部工程可以按官方 SDK 的安装/链接方式消费，而不是直接依赖源码树里的 `sdk_shim` 或 `rokae_xmate3_ros2/*` 实现头
- 当前 install-facing 兼容面额外保证：`rokae::Robot_T<rokae::WorkType::collaborative, 6>` 可直接实例化；`xMateRobot(remoteIP, localIP)` 按官方风格在构造后立即尝试连接；`connectToRobot(remoteIP, localIP="")`（无 `ec`）失败会抛异常；`connectToRobot(ec)` 仅走 `error_code` 通道；默认构造不再隐式使用固定 IP（必须显式提供 remoteIP）；`setToolset(toolName, wobjName, ec)` 返回 `Toolset`；`jointTorque(ec)` 为主入口、`jointTorques(ec)` 为兼容别名；`flangePos(ec)` 以 deprecated 兼容别名保留；`std::array<double[2], 6>` 软限位接口、`executeCommand(std::vector<MoveCCommand>, ec)` 都已在公共 ABI 头中保留。开启软限位时若省略 limit 数组，将沿用当前控制侧已保存的软限位数值，不会把区间清零。

说明：当前 compatibility lane 仍然是 **ROS2/Gazebo-backed** 的安装包；`xCoreSDK::xCoreSDK_static` 与 `xCoreSDK::xCoreSDK_shared` 都是可直接消费的 install-facing 目标。为了让真实静态目标具备完整链接闭环，配置期同样需要系统可见的 ROS2/Gazebo 依赖；运行时则仍需要相同的 ROS2/Gazebo 动态库与插件环境。private 实现依赖不属于 public C++ ABI 头文件面。

下游安装态示例：

```cmake
find_package(xCoreSDK CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_static)
```

`main.cpp` 只需要安装后的公共头：

```cpp
#include <rokae/robot.h>
#include <rokae/model.h>
#include <rokae/motion_control_rt.h>
#include <rokae/planner.h>
```

说明：

- `include/rokae/detail/*`、`include/rokae/sdk_shim*.hpp`、`include/rokae_xmate3_ros2/*` 仍保留在源码树用于现有内部实现/契约测试，但不再属于官方兼容安装面。
- 当前 ABI 分层只解决 **安装/链接/头文件污染边界**，不宣称真机级 RT 语义完全等价。
- 兼容检查资产见 `docs/COMPAT_ABI.md`、`test/harness/check_compat_public_abi.py`、`test/harness/run_install_tree_consumer.py`、`test/harness/check_exported_symbols.py` 与 `test/compat/install_tree/`。

## 文件结构

```text
rokae_xmate3_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   ├── rokae/
│   └── rokae_xmate3_ros2/
├── src/
│   ├── robot.cpp
│   ├── sdk/
│   │   ├── robot_internal.hpp
│   │   ├── robot_clients.cpp
│   │   ├── robot_state.cpp
│   │   ├── robot_motion.cpp
│   │   └── robot_model.cpp
│   ├── gazebo/
│   └── runtime/
├── examples/
│   ├── README.md
│   └── cpp/
├── test/
│   ├── unit/
│   ├── harness/
│   └── strict/
├── launch/
│   ├── simulation.launch.py
│   ├── rviz_only.launch.py
│   ├── xmate3_simulation.launch.py
│   └── xmate3_gazebo.launch.py
├── config/
│   ├── ros2_control.yaml
│   └── xMate3.rviz
├── urdf/
│   ├── xMate3.xacro
│   ├── xMate3.ros2_control.xacro
│   └── materials.xacro
├── models/
│   └── rokae_xmate3_ros2/
│       ├── model.config
│       ├── model.sdf
│       └── meshes/
├── worlds/
│   └── empty.world
├── docs/
│   └── QUICKSTART.md
├── msg/
├── srv/
└── action/
```

> `xMate3.xacro` 是主要维护入口；构建期会在 **构建目录** 生成 `generated/urdf/xMate3.urdf`（即 `<build>/generated/urdf/xMate3.urdf`）供 KDL/测试主路径直接加载，而不是在源码树提交一份派生 URDF。与此同时会生成 `<build>/generated/urdf/xMate3.description.json`，记录 canonical description 的来源 xacro、关键展开参数与 SHA-256，避免 launch 链与模型链各自消费不透明描述源。mesh 路径默认走 `models/rokae_xmate3_ros2/meshes/...`。
> `src/sdk/` 负责 SDK façade 分层；`src/gazebo/` 聚焦模型与 Gazebo 插件；`src/runtime/` 聚焦规划、状态与执行链；`test/` 按 `unit / harness / strict` 三层组织。

## 快速开始

### 1. 环境准备

```bash
sudo apt install \
  ros-humble-desktop-full \
  ros-humble-gazebo-ros \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  libeigen3-dev \
  python3-numpy \
  python3-lxml
```

### 2. 编译

```bash
cd ~/ros2_ws0
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

release build、`ctest` 与源码归档打包建议统一通过 `src/rokae_xmate3_ros2/tools/clean_build_env.sh` 运行。
它会优先使用 `ROKAE_PYTHON_EXECUTABLE` 或当前系统 `python3`，并清理 Conda 相关环境变量，避免影子库路径污染构建或发布结果。
release-grade build/test/package 必须通过这个 wrapper 运行；它只负责清理环境污染，不再向运行时主动注入当前目录或 `build/*`、`install/*` 动态库路径。

### 3. 启动仿真

```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py

# 可选
ros2 launch rokae_xmate3_ros2 simulation.launch.py gui:=false rviz:=false
ros2 launch rokae_xmate3_ros2 simulation.launch.py backend_mode:=effort
ros2 launch rokae_xmate3_ros2 simulation.launch.py backend_mode:=jtc enable_xcore_plugin:=false
ros2 launch rokae_xmate3_ros2 xmate3_simulation.launch.py
ros2 launch rokae_xmate3_ros2 xmate3_gazebo.launch.py
ros2 launch rokae_xmate3_ros2 rviz_only.launch.py
```

`simulation.launch.py` is the canonical launch entry. `xmate3_simulation.launch.py` and `xmate3_gazebo.launch.py` are maintained as thin compatibility aliases that forward the full parameter set.
By default the launch path rejects non-canonical model overrides; use `allow_noncanonical_model:=true` only for developer-mode validation.


推荐把门禁命令固化为：
```bash
./tools/run_static_sanity.sh
src/rokae_xmate3_ros2/tools/run_quick_gate.sh ~/ros2_ws0
src/rokae_xmate3_ros2/tools/run_release_gate.sh ~/ros2_ws0
```

原生 ROS facade `rokae::ros2::xMateRobot` 与 SDK 兼容 wrapper (`rokae::Robot_T` / `rokae::Cobot` / `rokae::xMateRobot`) 在 backend/internal lane 上仍可读取 runtime authoritative catalog；安装态 public xMate6 contract 不再把 RL 视为支持能力。
若确实需要兼容旧缓存旁路，请显式设置：
```bash
export ROKAE_SDK_LEGACY_CATALOG_FALLBACK=true
# 兼容旧环境变量时，也可显式关闭 strict authority（兼容别名，建议逐步迁移）
export ROKAE_SDK_STRICT_RUNTIME_AUTHORITY=false
```

说明：Gazebo/runtime facade 仍可在 backend/internal lane 上提供 **simulation-grade** `ppToMain()`；安装态 public xMate6 contract 不再把该 RL 能力视为受支持入口。

### 4. 运行示例

```bash
ros2 run rokae_xmate3_ros2 example_99_complete_demo
ros2 run rokae_xmate3_ros2 example_04_motion_basic
ros2 run rokae_xmate3_ros2 example_25_rt_s_line
```

常用诊断命令：
```bash
ros2 service call /xmate3/internal/get_runtime_diagnostics rokae_xmate3_ros2/srv/GetRuntimeDiagnostics "{}"
ros2 service call /xmate3/internal/get_runtime_state_snapshot rokae_xmate3_ros2/srv/GetRuntimeStateSnapshot "{}"
ros2 topic echo /xmate3/internal/runtime_status --once
```

安装态 xCoreSDK 兼容包现在默认采用严格 public install 策略：

- `ROKAE_STRICT_PUBLIC_INSTALL=ON`：install-facing 仅暴露 `include/rokae/*` 与 `xCoreSDK::xCoreSDK_shared/static`
- `ROKAE_EXPORT_PRIVATE_SDK_TARGETS=OFF`：默认不向 install tree 导出 native/private ROS2/Gazebo target；这不会影响 `xCoreSDK::xCoreSDK_static` 作为真实 install-facing 静态库被导出
- install-tree consumer coverage 已扩展到基础状态/运动与纯链接最小示例，见 `test/compat/install_tree/minimal_state_and_motion.cpp`、`minimal_static_link_only.cpp` 与 `minimal_shared_link_only.cpp`

SDK wrapper 当前对以下生命周期/模式入口执行幂等成功语义：

- `connectToRobot()`
- `disconnectFromRobot()`
- `setPowerState()`
- `setOperateMode()`
- `setMotionControlMode()`

也就是说，当 aggregated runtime snapshot 已经表明目标状态与请求一致时，wrapper 会直接成功返回，而不是把“重复调用”当成错误。

诊断输出建议额外关注：`rt_subscription_plan` / `rt_prearm_status` / `rt_watchdog_summary` / `rt_state_source` / `model_primary_backend` / `model_fallback_used`。

RT 兼容层当前新增了三条必须显式理解的运行语义：

- `setEndEffectorFrame()` 与 `setLoad()` 现在会同时进入 `xMateModel` 与 runtime RT bridge，避免 model / RT 使用不同 TCP 或负载上下文。
- `setFcCoor()` 现在会进入笛卡尔阻抗期望力的坐标系变换链；它仍是 simulation-grade force-frame 语义，不是控制柜原生力控实现。
- `setRtNetworkTolerance()` 会影响 direct RT command timeout 窗口；`useRciClient(true)` 会显式阻断当前 simulated direct RT 控制面，避免与兼容 RCI 客户端语义混用。
- native SDK facade 现在对所有公开 `std::error_code& ec` 入口统一回写 `lastErrorCode()`，避免只在局部链路可见错误状态。
- `startReceiveRobotState()/updateRobotState()` 现在默认执行 **strict RT state semantics**：只接受手册允许的状态发送周期（`1/2/4/8ms/1s`），重复订阅必须先 `stopReceiveRobotState()` 再重启；1ms in-loop 订阅只接受 controller-native / 本地派生 RT 字段；慢速 polled 状态流只允许 service-backed 的 `tcpPose* / tauExt*` 字段；`psi_*` / `elbow*` 在 xMate3 六轴 shim 中不再作为可订阅 RT 字段暴露。RT 取数失败或订阅计划降级会直接失败，不再静默回退到 `GetJointPos/GetJointVel/GetJointTorques` 的 NRT 快照。
- 为了让 `getStateData<double>() / getStateData<bool>()` 不再只是“有接口没字段源”，兼容层额外提供两个 **compat-only** RT metadata 字段：`rokae::RtCompatFields::samplePeriod_s`（本次状态样本周期，单位秒）与 `rokae::RtCompatFields::sampleFresh`（本次缓存是否来自成功 RT 更新）。这两个字段是 ROS2/Gazebo-backed compat 扩展，不属于官方 xCore RT 字段集。

默认发布验收命令：
```bash
cd ~/ros2_ws0
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  colcon build --packages-select rokae_xmate3_ros2 --symlink-install
cd build/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest --output-on-failure
```

全量 headless examples harness（非默认门禁）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_FULL_EXAMPLES_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -R gazebo_examples_full --output-on-failure
```

backend mode smoke（非默认，覆盖 `effort / jtc / hybrid`）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_BACKEND_MODE_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -L gazebo_integration_modes --output-on-failure
```

teardown 质量回归（非默认，验证 `prepare_shutdown` 契约、phase 推进与优雅停机路径）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -L gazebo_teardown_quality --output-on-failure
```

teardown 重复压力回归（非默认，连续跑 10 次 shutdown 路径收竞态）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_ENABLE_GAZEBO_TEARDOWN_QUALITY_TESTS=ON /home/chen/ros2_ws0/src/rokae_xmate3_ros2
cmake --build . -j4
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest -R gazebo_teardown_quality_repeat --output-on-failure
```

源码归档一致性检查（发布 zip 前，确保候选包按 `package root + workspace sidecars` 布局，且包含 `owner_arbiter.hpp`、工作区根 `.gitignore` / `colcon_defaults.yaml` 与 `flange/tool0/tcp/payload` 终局化 xacro）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_SOURCE_ARCHIVE=/path/to/src.zip /home/chen/ros2_ws0/src/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build . --target verify_source_archive
```

生成并校验唯一候选源码归档（推荐的发布出口）：
```bash
cd ~/ros2_ws0/build/rokae_xmate3_ros2
cmake -DROKAE_SOURCE_ARCHIVE_OUTPUT=$PWD/rokae_source_candidate.zip /home/chen/ros2_ws0/src/rokae_xmate3_ros2
../../src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build . --target release_candidate
```

禁止直接上传手工打包的 `src.zip`；只允许分发 `package_verified_source_archive` 生成并通过 archive gate 的候选包。
候选包主体固定为 `src/rokae_xmate3_ros2` 的 package root，工作区根只白名单附带 `.gitignore` 和 `colcon_defaults.yaml`。
`Testing/`、工作区根 PDF、`tmp_*` 日志以及缓存/构建产物都会被 archive gate 拒绝。
候选包旁边会同时生成 `.manifest.txt` 与 `.sha256` sidecar，用来核对“外发包就是刚刚验过的那个包”。
manifest 会额外记录 `package.xml` 版本号、git commit hash 和 UTC 生成时间，方便后续追溯候选包来源。

### Release package rule

Only the archive produced by `release_candidate` / `package_verified_source_archive` may be distributed.

Do not distribute:
- workspace snapshots
- manually zipped source trees
- archives containing `Testing/`, `build/`, `log/`, `__pycache__`, `.pyc`, `.pdf` or `tmp_*`
- target-environment acceptance is automated via `tools/run_target_env_acceptance.sh` and `.github/workflows/acceptance-humble-gazebo11.yml`; when you are already on Ubuntu 22.04 / Humble / Gazebo11, pass `--local-target-env` to run the same bundle without docker/podman. The local path now resolves `PKG_ROOT` before entering the branch, fails fast if `rosdep` / `ros2` / `gazebo_ros` are missing from the sourced Humble environment, and always emits a machine-readable acceptance report into `artifacts/target_env_acceptance/` unless you override `--report-dir`, including failed environment checks, failed image builds, and failed gate stages.
- `--launch-smoke` now runs `tools/run_launch_smoke.sh`, which validates installed-package launch discovery, xacro expansion, and `ros2 launch --show-args` resolution inside the locked Humble/Gazebo11 image

Recommended release flow:

```bash
cd ~/ros2_ws0
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake -S src/rokae_xmate3_ros2 -B build/rokae_release
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build build/rokae_release -j
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  ctest --test-dir build/rokae_release --output-on-failure
src/rokae_xmate3_ros2/tools/clean_build_env.sh \
  cmake --build build/rokae_release --target release_candidate
```

默认 `ctest --output-on-failure` 固定为 16 项门禁：
- 13 个纯单测
- `gazebo_sdk_regression`
- `gazebo_examples_smoke`
- `gazebo_alias_smoke`

teardown 质量与 full examples 业务回归已经拆成两个质量信号：
- `gazebo_examples_full` 只关注 `27/27` examples 业务通过
- `gazebo_teardown_quality` 单独关注 `prepare_shutdown` 返回的统一 contract：`owner / runtime_phase / shutdown_phase / safe_to_delete / safe_to_stop_world`
- `gazebo_teardown_quality_repeat` 单独关注 teardown 路径的重复稳定性

当前 JTC 主链定义为 `position trajectory backend`，不是完整闭环伺服替身。

### Runtime contract headers

公共 runtime contract 头固定从 `include/rokae_xmate3_ros2/runtime/*` 引用：

- `include/rokae_xmate3_ros2/runtime/runtime_contract.hpp`
- `include/rokae_xmate3_ros2/runtime/owner_arbiter.hpp`
- `include/rokae_xmate3_ros2/runtime/shutdown_coordinator.hpp`

`src/runtime/*.hpp` 下保留的同名头只用于兼容旧 include，新代码不要再依赖这些路径。

详细步骤请参考 [docs/QUICKSTART.md](docs/QUICKSTART.md)。

## 示例程序

| 示例 | 对应手册章节 | 功能说明 |
|------|-------------|---------|
| 01 | 4.3 | 机器人基本连接与信息查询 |
| 02 | 4.3 | 关节位置与笛卡尔位姿读取 |
| 03 | 4.3 | 运动学计算 (FK/IK) |
| 04 | 4.4 | 基础运动控制 |
| 05 | 4.4 | 笛卡尔空间运动 |
| 07 | 4.3/4.8 | 安全与碰撞检测 |
| 08 | 4.8 | 路径录制与回放（internal/backend only） |
| 09 | 4.5/4.8 | RT / 运行时选项 / public-lane 边界 |
| 10 | 4.3/4.4/4.8 | 官方 SDK 工作流映射 |
| 11 | 4.4 | `confData`、偏移、在线调速、`MoveSP` |
| 12 | 4.3 | 多线程状态流读取 |
| 13 | 4.7 | RL 工程查询、加载、运行控制（internal/backend only） |
| 14 | 8.3.8 | 扩展模型计算 |
| 15 | 4.3/4.4 | 运动队列、事件回调、控制器日志 |
| 17 | 4.3/4.5 | 状态缓存与异步轮询 |
| 18 | 4.8 | 工具、工件与工具组管理（不含标定） |
| 19 | 4.8 | 末端力矩、奇异规避与诊断 |
| 20-26 | 4.5 | RT 控制系列示例（Gazebo simulated RT facade） |
| 99 | - | 综合演示 |

更多说明请参考 [examples/README.md](examples/README.md)。

## API 参考

### 机器人基本操作

```cpp
#include "rokae/robot.h"

rokae::xMateRobot robot;
std::error_code ec;
robot.connectToRobot(ec);
robot.setPowerState(true, ec);
robot.setOperateMode(rokae::OperateMode::automatic, ec);
auto joints = robot.jointPos(ec);
```

### 运动控制

`setDefaultSpeed()` 使用 `mm/s`，`setDefaultZone()` 使用 `mm`，`adjustSpeedOnline(scale)` 会对当前与后续 NRT 轨迹做仿真重定时。

接口能力分级：
- `GenerateSTrajectory(joint)`：近似实现，strict jerk-limited retimer compatibility path
- `GenerateSTrajectory(cartesian)`：近似实现，基于路径弧长与 seeded IK 的笛卡尔近似轨迹
- 碰撞检测：近似实现，residual-based 仿真监督，不等价于真机安全功能
- 实时控制：实验性，simulated RT facade，不承诺真机 1kHz/硬实时语义
- 模型库 / 力矩 / 无摩擦力矩：近似实现，基于统一仿真代理项，不是完整刚体动力学库

```cpp
robot.setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
robot.setDefaultSpeed(500, ec);  // mm/s
robot.setDefaultZone(5, ec);      // mm
robot.moveReset(ec);
robot.moveStart(ec);
```

统一规划失败原因：
- `unreachable_pose`
- `conf_mismatch`
- `soft_limit_violation`
- `near_singularity_rejected`
- `runtime_busy`
- `shutdown_in_progress`

### 运动学计算

```cpp
auto model = robot.model();
auto pose = model.calcFk(rokae::JointPosition(6, 0.0));
auto ik = model.calcIk(pose);
```

`xMateModel` 当前使用 KDL/URDF 运动学后端提供 FK / Jacobian / 速度映射，动力学相关接口仍是统一近似模型而非完整刚体动力学库。

### 启动装配模式

- `backend_mode:=hybrid`：默认模式，同时启用 xCore plugin 与 JTC，由 owner 状态机仲裁执行权
- `backend_mode:=effort`：只启用 xCore plugin / effort runtime
- `backend_mode:=jtc`：只启用 ros2_control / JTC；JTC 当前是 position trajectory backend，`velocity/acceleration` 作为轨迹边界与重定时辅助信息
- `enable_xcore_plugin:=false`：显式关闭 xCore Gazebo plugin 装配

## 架构说明

```text
用户应用/示例
        |
        | SDK facade / ROS2 服务 / Action / 话题
        v
Request adapter / Service facade
        |
        | 几何路径 / lookahead / zone / unified retimer
        v
Motion runtime / Owner arbiter / Supervisor
        |
        | backend dispatch
        v
JTC backend        Effort backend
        |                 |
        +-------- Gazebo plugin --------+
                          |
                          v
               Gazebo 11 + xMate3 URDF/KDL/model facade
```

## 扩展框架

## 实现审计

- 当前真实实现面：[`docs/IMPLEMENTATION_AUDIT.md`](docs/IMPLEMENTATION_AUDIT.md)
- 剩余硬化任务：[`docs/HARDENING_BACKLOG.md`](docs/HARDENING_BACKLOG.md)

这个部分专门回答两个问题：

1. 现在到底哪些功能已经实现了。
2. 哪些地方仍然只是仿真级或尚未完全闭合。

后续所有更强的“已对齐”结论，都应该以这里和对应测试为准。


- runtime 仍然是唯一状态真源
- KDL 为默认主运动学后端，`improved_dh` 用于 seed / fallback / regression
- RT / NRT 永久分 profile，不再回混
- 新 contract 为主、legacy contract 只做兼容 façade
- 逐接口对齐矩阵见 `docs/API_ALIGNMENT_MATRIX.md`
- 可持续扩展蓝图见 `docs/EXTENSION_FRAMEWORK.md`

## ROS2 接口

### 服务
- `/xmate3/cobot/connect`
- `/xmate3/cobot/disconnect`
- `/xmate3/cobot/get_joint_pos`
- `/xmate3/cobot/get_joint_torque`
- `/xmate3/cobot/get_end_torque`
- `/xmate3/cobot/get_cart_posture`
- `/xmate3/cobot/calc_fk`
- `/xmate3/cobot/calc_ik`
- `/xmate3/cobot/move_reset`
- `/xmate3/cobot/move_start`
- `/xmate3/cobot/stop`
- `/xmate3/internal/validate_motion`
- `/xmate3/internal/get_runtime_diagnostics`
- `/xmate3/internal/prepare_shutdown`

### 话题
- `/xmate3/joint_states`
- `/xmate3/cobot/operation_state`
- `/xmate3/internal/runtime_status`

### 动作
- `/xmate3/cobot/move_append`
- `/xmate3/cobot/jog`
- `/xmate3/cobot/replay_path`

## 常见问题

### Q: 如何启动仿真？
```bash
ros2 launch rokae_xmate3_ros2 simulation.launch.py
```

### Q: 示例程序无法连接？
确认仿真已启动并且机器人已生成：
```bash
ros2 service list | grep xmate3
```

### Q: 运行示例后机械臂不动？
等待控制插件初始化完成后再运行示例：
```bash
ros2 action list | grep move_append
```

### Q: `spawn_entity.py` 报 `ModuleNotFoundError`？
```bash
sudo apt install python3-numpy python3-lxml
```

## 相关资源
- [快速入门指南](docs/QUICKSTART.md)
- [示例程序说明](examples/README.md)
- xCore SDK C++ 使用手册
- [ROKAE 官网](https://www.rokae.com)

## 许可证

Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD.

Licensed under the Apache License, Version 2.0.


## 新增架构文档

- `docs/API_ALIGNMENT_MATRIX.md`：逐接口完成度矩阵
- `docs/MODEL_TRACEABILITY.md`：手册参数与内部表示映射
- `docs/FIDELITY_POLICY.md`：StrictAligned / SimApprox / Experimental 语义
- `config/ros2_control_nrt.yaml` / `config/ros2_control_rt.yaml`：控制周期配置拆分
- `docs/RUNTIME_CATALOG_POLICY.md`：RL / tool / wobj 目录以 runtime 为唯一真相源


## Pass 3 架构收口

- 新增 `runtime_state_machine`，把 runtime 最终状态落地从执行循环里显式抽出。
- 将 `QueryFacade` 继续拆成 state / kinematics / diagnostics 三个实现单元。
- 将 SDK state 侧继续拆成 cache / queries 两块，减少 wrapper 混域。
- 新增 `planner_preflight` 与 `planner_trace`，让 planner 进入“可解释、可拒绝”的结构化阶段。


## Pass 4 平台定型

- 新增 `runtime_catalog_service` 与 `query_catalog_service`，把 RL / tool / wobj 目录正式收回 runtime query 域。
- `ValidateMotion` 现在先运行结构化 `planner_preflight`，再进入 planner，从而稳定输出 preflight / backend / branch policy 线索。
- 增加 `test_planner_preflight` 与 `test_runtime_catalog_service`，把解释型规划和 runtime 目录真相源写进测试。


## Pass 6 additions

- Runtime profile capability matrix
- Runtime option catalog descriptors
- Diagnostics exposure for profile/runtime-option/catalog summaries


## Pass 7 highlights

- Added an internal/runtime profile capability query surface
- Runtime diagnostics now expose the active request id and execution backend
- SDK wrapper can fetch profile and runtime-option capability snapshots


## ROS integration ownership

- `rokae::ros2::xMateRobot` now accepts `RosClientOptions` so embedded callers can inject an external ROS `Context` / `Node` / `Executor` instead of forcing wrapper-owned globals.
- SDK compatibility headers are split into a true umbrella (`rokae/sdk_shim.hpp`) plus narrower entrypoints. For six-axis source compatibility, `rokae/robot.h` again transitively includes `rokae/planner.h`.
- planner / IK requests now lock a single primary backend per request. Any relaxed fallback path becomes a visible contract violation instead of a silent success path.


## Environment contract

See `docs/ENVIRONMENT_LOCK.md` for the pinned ROS/Gazebo/Python assumptions used by the gate scripts and launch path resolution.


SDK-compatible wrappers (`rokae::Robot_T`, `rokae::Cobot`, `rokae::xMateRobot`) and the native ROS facade (`rokae::ros2::xMateRobot`) now both default to strict runtime authority. Returning stale wrapper-side catalog data is treated as an explicit compatibility opt-in rather than a silent default.


- `rokae/planner.h` 与 `src/runtime/unified_retimer.cpp` 已统一为同一套严格 `jerk-limited` 轨迹核，不再分裂为 planner 严格 jerk / runtime quintic 两套主线。


- 2026-04-02 follow-up hardening: fixed SDK wrapper semantics for `setSoftLimit(false, ...)`, moved remaining servo-loop metadata reads to `DataStoreState::RtSemanticSnapshot`, and guarded `RuntimeBootstrap` executor release with ownership tracking.

## 实施审计

- `src/runtime/service_contract_manifest.hpp` 现在是 primary service / compatibility alias 的单一契约源；运行时注册、描述符校验与契约测试围绕它派生，减少手工多处同步。
- `launch/_simulation_support.py` 与 `launch/xmate3_simulation.launch.py` 默认优先消费 canonical `generated/urdf/xMate3.urdf`；当需要非默认展开参数时，必须显式打开 `allow_noncanonical_model:=true`，再把 `model:=urdf/xMate3.xacro` 交给 `tools/render_robot_description.py` 动态展开；默认路径只允许 canonical description。
- native SDK 读取链新增 `/xmate3/internal/get_runtime_state_snapshot` 聚合只读快照面；`jointPos/jointVel/jointTorque/baseFrame/toolset` 会优先复用同一 runtime-owned snapshot，再退回旧 service，保持兼容同时减少高频离散往返。
- RT 诊断摘要现在会附带 `policy_source=rt_field_registry` 的字段约束说明，明确哪些字段允许 strict in-loop，哪些字段只能走 polled snapshot。


- `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES=ON` 可显式安装 internal/backend example 二进制；默认 **OFF**，避免把源码树内部验证入口混入 public 安装面。


- RT compatibility hardening: `MoveJ/MoveL/MoveC` now validate official-style speed factors in `(0, 1]`; Cartesian start/target checks use translation + quaternion angular tolerance; and install-facing/source-tree RT commands share one semantic command bridge. `MoveC` interpolation density now scales with both sweep angle and radius.
