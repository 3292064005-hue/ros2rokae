# 修复摘要（2026-03-17）

本次修复面向 **xMate3 ROS2 + Gazebo11** 仿真包与 **xCore SDK C++ 官方示例/手册** 的接口一致性。

## 已修复的关键问题

### 1. `moveAppend()` 语义错误

原实现每次 `moveAppend()` 都会隐式调用 `moveReset()`，导致官方示例里连续多次 `moveAppend()` 的缓存语义失效。

现已修复为：

- `moveAppend()` 只负责**追加**缓存；
- `moveReset()` 只在显式调用时清空缓存；
- `executeCommand()` 仍保持“重置 -> 追加 -> 启动”的即时执行语义；
- 增加与手册一致的数量校验：
  - `moveAppend`: 1~100
  - `executeCommand`: 1~1000

### 2. `startJog()` 单位语义错误

原实现把 `step` 直接按 m/rad 使用，不符合官方手册。

现已修复为：

- 笛卡尔 Jog：`step` 按 **mm** 解释并转换为 **m**；
- 轴空间 Jog：`step` 按 **deg** 解释并转换为 **rad**；
- 调用顺序与官方 API 保持一致。

### 3. 官方 `rokae/utility.h` 缺失

新增：

- `include/rokae/utility.h`
- `Utils::postureToTransArray(...)`
- `Utils::arrayToTransMatrix(...)`
- `Utils::transMatrixToArray(...)`

同时保留原有 `postureToMatrix()` / `matrixToPosture()`。

### 4. 官方示例常见构造方式不兼容

补齐以下 SDK 风格构造/赋值方式：

- `MoveLCommand(target, speed, zone)`
- `MoveJCommand(target, speed, zone)`
- `MoveAbsJCommand(vector<double> joints, speed, zone)`
- `MoveSPCommand(target, radius, radius_step, angle, direction, speed)`
- `offset = {Offset::offs, array<double,6>{...}}`
- `toolset.ref = {{x,y,z},{rx,ry,rz}}`

### 5. 补充官方示例配套打印头

新增：

- `examples/print_helper.hpp`

方便直接按官方示例风格组织演示代码。

## 已知边界

- 本包目标仍然是 **xMate3 六轴 Gazebo 仿真**。
- 官方示例中涉及 **xMateErProRobot / StandardRobot / 工业机型 / 7 轴实时模型** 的内容，不代表本包已经具备同等真机能力。
- 由于当前环境缺少 ROS2 Humble / Gazebo11 运行时，本次以**静态审查 + 源码修复**为主，未在容器内完成 `colcon build` 和联机仿真回归。

