# MoveIt2 集成使用指南

本文档说明如何在 xMate3 ROS2 仿真包中使用 MoveIt2。

---

## 目录

- [安装 MoveIt2](#安装-moveit2)
- [快速开始](#快速开始)
- [Launch 文件说明](#launch-文件说明)
- [MoveIt2 配置](#moveit2-配置)
- [示例程序](#示例程序)
- [架构说明](#架构说明)
- [常见问题](#常见问题)

---

## 安装 MoveIt2

### ROS2 Humble

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-moveit-ros \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-ros-move-group
```

### 从源码编译 (可选)

如果需要最新的 MoveIt2 功能，可以从源码编译：

```bash
cd ~/ros2_ws0/src
git clone https://github.com/ros-planning/moveit2.git -b humble
cd ~/ros2_ws0
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to moveit
```

---

## 快速开始

### 1. 编译

```bash
cd ~/ros2_ws0
colcon build --packages-select rokae_xmate3_ros2 --symlink-install
source install/setup.bash
```

### 2. 启动仿真 + MoveIt2

```bash
ros2 launch rokae_xmate3_ros2 simulation_moveit.launch.py
```

这会启动：
- Gazebo 仿真器
- xCore 控制插件
- MoveIt2 move_group
- RViz2 (带 MoveIt2 插件)

### 3. 在 RViz2 中使用

启动后，在 RViz2 中可以：

1. **使用交互标记拖动目标位姿**
   - 点击 "Interact" 工具
   - 拖动末端执行器的交互标记到目标位置

2. **规划路径**
   - 在 "MotionPlanning" 面板中点击 "Plan"
   - 查看规划的轨迹

3. **执行轨迹**
   - 点击 "Execute" 执行规划好的轨迹

### 4. 运行 MoveIt2 示例程序

```bash
# MoveIt2 集成示例
ros2 run rokae_xmate3_ros2 example_moveit_integration

# MoveIt2 笛卡尔运动示例
ros2 run rokae_xmate3_ros2 example_moveit_cartesian
```

---

## Launch 文件说明

### simulation_moveit.launch.py

完整的仿真 + MoveIt2 启动文件。

**参数**：
- `model`: URDF/xacro 模型文件路径
- `world`: Gazebo world 文件路径
- `gui`: 是否启动 Gazebo GUI (默认: true)
- `rviz`: 是否启动 RViz (默认: true)
- `verbose`: Gazebo 详细输出 (默认: true)
- `use_sim_time`: 使用仿真时间 (默认: true)

**使用示例**：

```bash
# 完整启动 (默认)
ros2 launch rokae_xmate3_ros2 simulation_moveit.launch.py

# 无 GUI 模式 (服务器运行)
ros2 launch rokae_xmate3_ros2 simulation_moveit.launch.py gui:=false

# 不启动 RViz
ros2 launch rokae_xmate3_ros2 simulation_moveit.launch.py rviz:=false
```

### moveit_planning_execution.launch.py

仅启动 MoveIt2 (不启动 Gazebo)，用于连接已有的仿真或真实机器人。

**使用示例**：

```bash
# 终端 1: 启动仿真 (无 MoveIt2)
ros2 launch rokae_xmate3_ros2 simulation.launch.py rviz:=false

# 终端 2: 启动 MoveIt2
ros2 launch rokae_xmate3_ros2 moveit_planning_execution.launch.py
```

---

## MoveIt2 配置

配置文件位于 `config/moveit/` 目录：

### xMate3.srdf

Semantic Robot Description Format，定义：
- 规划组 (planning groups)
- 末端执行器
- 预定义位姿 (home, ready, folded)
- 碰撞对禁用

### kinematics.yaml

运动学求解器配置：

```yaml
xmate3_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

**可选的运动学求解器**：
- `kdl_kinematics_plugin/KDLKinematicsPlugin` (默认)
- `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` (推荐，更快更稳定)
- `lma_kinematics_plugin/LMAKinematicsPlugin`

### ompl_planning.yaml

OMPL (Open Motion Planning Library) 规划器配置：

```yaml
xmate3_arm:
  default_planner_config: RRTConnect
  planner_configs:
    - SBL
    - EST
    - LBKPIECE
    - RRT
    - RRTConnect
    - RRTstar
    - ...
```

**常用规划器**：
- `RRTConnect`: 快速双向 RRT，适合大多数情况 (默认)
- `RRT`: 基础 RRT
- `RRTstar`: 渐进最优 RRT
- `EST`: 扩展空间树
- `PRM`: 概率 roadmap

### joint_limits.yaml

关节速度和加速度限制：

```yaml
joint_limits:
  xmate_joint_1:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 2.0
```

### moveit_controllers.yaml

MoveIt2 控制器配置，定义如何与实际控制器通信。

### controllers.yaml

可用控制器列表。

---

## 示例程序

### 10_moveit_integration.cpp

MoveIt2 集成基础示例，演示：
- 初始化 xCore SDK 和 MoveIt2
- 获取当前状态
- 关节空间规划 + 执行
- 笛卡尔空间规划 + 执行
- 笛卡尔路径规划

```cpp
// 创建 MoveGroupInterface
moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "xmate3_arm"
);

// 设置目标
move_group.setJointValueTarget(target_joints);

// 规划
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

// 执行
if (success) {
    // 方式 1: MoveIt2 直接执行
    // move_group.execute(plan);

    // 方式 2: 使用 xCore SDK 执行
    robot.moveAbsJ(cmd, ec);
    robot.moveStart(ec);
}
```

### 11_moveit_cartesian.cpp

笛卡尔运动示例，演示：
- MoveL (直线运动)
- MoveC (圆弧运动)
- MoveSP (螺旋运动)
- MoveIt2 笛卡尔路径规划

---

## 架构说明

### 两种使用模式

#### 模式 1: 纯 xCore SDK (原版)

仅使用 xCore SDK，不涉及 MoveIt2：

```
用户代码 → xCore SDK → Gazebo 插件 → 仿真
```

**优点**：
- API 简单，与真实机器人完全一致
- 直接支持 MoveAbsJ, MoveJ, MoveL, MoveC, MoveSP
- 无需额外配置

**适用场景**：
- 简单的点对点运动
- 固定轨迹重复执行
- 与 SDK 手册完全对齐的教学

#### 模式 2: MoveIt2 规划 + xCore SDK 执行 (推荐)

使用 MoveIt2 进行规划，xCore SDK 执行：

```
用户代码 → MoveIt2 (规划) → xCore SDK (执行) → Gazebo 插件 → 仿真
```

**优点**：
- 复杂环境中的碰撞检测
- 多种规划算法选择
- 笛卡尔路径规划
- 支持多种运动学求解器
- 交互式 RViz 界面

**适用场景**：
- 有障碍物的环境
- 需要优化的路径
- 人机交互演示
- 研究和开发

---

## 常见问题

### Q: 编译时提示找不到 MoveIt2

确保 MoveIt2 已正确安装：

```bash
sudo apt install ros-humble-moveit
```

如果只安装了部分包，CMake 会自动跳过 MoveIt2 示例的编译。

### Q: MoveIt2 规划失败

常见原因：
1. 目标位置在碰撞中
2. 目标位置超出工作空间
3. 运动学求解失败

解决方法：
- 在 RViz 中查看碰撞可视化
- 尝试不同的规划器
- 调整目标位置

### Q: 如何使用 TRAC-IK 替代 KDL

1. 安装 TRAC-IK：

```bash
sudo apt install ros-humble-trac-ik
```

2. 修改 `kinematics.yaml`：

```yaml
xmate3_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.005
  solve_type: Speed
```

### Q: 如何提高规划速度

在 `ompl_planning.yaml` 中调整：

```yaml
xmate3_arm:
  default_planner_config: RRTConnect
  longest_valid_segment_fraction: 0.01  # 增大此值可以加快规划
```

或者设置更短的规划时间：

```cpp
move_group.setPlanningTime(2.0);  // 默认 5.0 秒
```

### Q: 如何添加碰撞物体

使用 PlanningSceneInterface：

```cpp
moveit::planning_interface::PlanningSceneInterface psi;

moveit_msgs::msg::CollisionObject box;
box.id = "table";
box.header.frame_id = "xMate3_base";

shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions = {1.0, 1.0, 0.05};

geometry_msgs::msg::Pose pose;
pose.position.x = 0.5;
pose.position.z = -0.025;

box.primitives.push_back(primitive);
box.primitive_poses.push_back(pose);
box.operation = box.ADD;

psi.applyCollisionObject(box);
```

### Q: MoveIt2 和 xCore SDK 可以同时使用吗

可以！这是推荐的使用方式：
- 用 MoveIt2 做规划和碰撞检测
- 用 xCore SDK 执行实际运动

参考 `example_moveit_integration.cpp` 了解如何混合使用。

### Q: 如何在真机上使用 MoveIt2

1. 用真实机器人驱动替代 Gazebo 插件
2. 确保 `joint_states` 话题正确发布
3. 配置 MoveIt2 控制器与真实控制器通信
4. 使用相同的 MoveIt2 配置

---

## 更多资源

- [MoveIt2 官方文档](https://moveit.picknik.ai/humble/)
- [MoveIt2 教程](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
- [xCore SDK C++ 使用手册](...)
- [ROKAE 官网](https://www.rokae.com)

---

**版本**: 1.0 | **最后更新**: 2024
