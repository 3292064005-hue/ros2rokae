# SDK 兼容性说明（ROS2 Humble + Gazebo 11）

本包的目标是让 **xCore SDK C++ 手册中的主要接口** 在 ROS2 / Gazebo 仿真环境中拥有一致的调用入口与可预期的行为。

需要说明的是：

- **接口兼容**：优先保证方法名、话题/服务入口、示例代码组织与 SDK 手册保持一致。
- **语义兼容**：对真实控制器依赖很强的能力，在 Gazebo 中提供 **可运行的仿真实现**，而不是空壳接口。
- **硬件差异**：涉及实时网络、控制器固件、寄存器总线、真实碰撞/力传感器的功能，在 Gazebo 中只能做到 **高保真模拟**，不能等价替代真机控制器。

---

## 本次增强重点

### 1. 修复了运行时最关键的不兼容项

`xMateRobot` 客户端使用的是：

- `/xmate3/io/get_di`
- `/xmate3/io/set_do`
- `/xmate3/io/get_ai`
- `/xmate3/io/set_simulation_mode`

而旧版 Gazebo 插件实际暴露的是：

- `/xmate3/cobot/get_di`
- `/xmate3/cobot/set_do`
- `/xmate3/cobot/get_ai`
- `/xmate3/cobot/set_simulation_mode`

这会导致 SDK 风格的 DI/DO/AI/AO 调用直接找不到服务。现在插件同时提供：

- **标准 SDK 命名空间**：`/xmate3/io/*`
- **旧版兼容命名空间**：`/xmate3/cobot/*`

从而既兼容 SDK 客户端，也兼容旧脚本。

### 2. 将仓库里“已定义但未接线”的接口真正接入 Gazebo

新增了对应服务端/客户端实现：

- 实时控制模式：`SetRtControlMode`
- 实时关节数据：`GetRtJointData`
- 自定义数据：`SendCustomData`
- 数据回调注册：`RegisterDataCallback`
- 动力学扭矩计算：`CalcJointTorque`
- S 型关节轨迹生成：`GenerateSTrajectory`
- 笛卡尔力到关节力矩映射：`MapCartesianToJointTorque`
- RL 工程接口：`LoadRLProject` / `StartRLProject` / `StopRLProject`

### 3. 补充了手册常见但仓库缺失的扩展能力

新增：

- 寄存器接口：`ReadRegister` / `WriteRegister`
- 奇异位规避开关：`SetAvoidSingularity` / `GetAvoidSingularity`
- 末端六维力/力矩估计：`GetEndTorque`
- `xMateRobot::startJog(...)` 的离散 Jog 兼容实现

### 4. 新增对手册关键兼容缺口的补齐

新增/修复：

- `Robot_T::calibrateFrame(...)`：补上 Gazebo 下可运行的标定近似实现，支持工具坐标系与基/工件类坐标系的几何估计
- `RtMotionControlCobot::setCollisionBehaviour(...)`：补上实时接口同名函数，并把阈值转换为仿真侧碰撞检测灵敏度
- `MotionControl<MotionControlMode::RtCommand>::automaticErrorRecovery(...)`：补上已弃用但手册仍保留的兼容入口
- `setControlLoop(...)/startLoop(...)`：修复 `setFinished()` 结束语义和非阻塞循环退出问题
- `useStateDataInLoop=true`：补上循环内自动刷新状态缓存的行为
- `updateRobotState()/getStateData()`：补上 `jointAcc_c` 与 `tauVel_c` 的导数估计，使官方实时示例更贴近手册行为

### 5. 补齐缺失 launch 与 SDK 辅助头文件

新增：

- `launch/rviz_only.launch.py`
- `launch/xmate3_simulation.launch.py`
- `include/rokae_xmate3_ros2/sdk_compat.hpp`
- `include/rokae_xmate3_ros2/utils.hpp`
- `include/rokae_xmate3_ros2/model.hpp`
- `include/rokae_xmate3_ros2/motion_generators.hpp`
- 示例：`examples/cpp/09_advanced_sdk_compat.cpp`

---

## 覆盖矩阵

| SDK 手册章节 | 状态 | 说明 |
|---|---|---|
| 4.3 基础连接 / 状态 / 位姿 / FK/IK | ✅ | 已支持 |
| 4.4 非实时运动控制 | ✅ | 已支持，补充离散 Jog 兼容实现 |
| 4.5 实时控制 | ✅（仿真） | 通过 `SetRtControlMode` / `GetRtJointData` 提供仿真版 |
| 4.6 IO 与通信 | ✅ | 修复 `/xmate3/io/*` 命名空间，并补充寄存器与自定义数据 |
| 4.7 RL 工程 | ✅（仿真） | 提供加载/启动/停止仿真工程生命周期 |
| 4.8 协作机器人能力 | ✅（仿真） | 拖动、路径录制、奇异位规避、末端力/力矩、动力学映射 |
| 8.3.7 路径规划类 | ✅（兼容头） | `motion_generators.hpp` 提供 SDK 风格封装 |
| 8.3.8 模型库 | ✅（兼容头） | `model.hpp` 提供 `XMateModel` 封装 |
| 8.3.9 工具函数 | ✅ | `utils.hpp` 提供角度/位姿转换 |

---

## 仿真实现与真机行为的差异

以下能力属于“高保真仿真”，但不等于真实控制器：

- **实时控制**：这里是 ROS2 服务风格的仿真控制，不涉及真机实时以太网链路。
- **RL 工程**：这里模拟的是工程生命周期与状态切换，不会执行真实控制器上的 RL runtime。
- **寄存器**：这里是插件内的键值存储寄存器，不连接外部 PLC/现场总线。
- **末端力/力矩**：这里使用雅可比矩阵和 Gazebo 关节力估计，不等于真实 F/T 传感器读数。
- **碰撞/奇异规避**：这里是运动学和状态机级别的仿真策略，不是控制器固件级安全闭环。

---

## 推荐包含方式

```cpp
#include "rokae_xmate3_ros2/sdk_compat.hpp"
```

这个头会统一引入：

- `robot.hpp`
- `utils.hpp`
- `model.hpp`
- `motion_generators.hpp`

适合作为 SDK 风格开发的统一入口。
