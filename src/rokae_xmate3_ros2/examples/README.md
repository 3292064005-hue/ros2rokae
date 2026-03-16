# 示例程序说明

> 本目录包含按 xCore SDK C++ 使用手册 章节组织的示例程序

---

## 目录结构

```
examples/
├── cpp/
│   ├── 01_basic_connect.cpp           # 第4.3节 - 基础连接
│   ├── 02_joint_cartesian_read.cpp    # 第4.3节 - 关节/位姿读取
│   ├── 03_kinematics.cpp              # 第4.3节 - 运动学计算
│   ├── 04_motion_basic.cpp            # 第4.4节 - 基础运动控制
│   ├── 05_motion_cartesian.cpp        # 第4.4节 - 笛卡尔空间运动
│   ├── 06_io_control.cpp              # 第4.6节 - IO控制
│   ├── 07_safety_collision.cpp        # 第4.3/4.8节 - 安全功能
│   ├── 08_path_record_replay.cpp      # 第4.8节 - 路径录制与回放
│   └── 99_complete_demo.cpp           # 综合演示
└── README.md                          # 本文件
```

---

## 示例索引

### 第4.3节 - 机器人基本操作及信息查询

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 1 | `01_basic_connect.cpp` | 连接机器人、获取信息、电源控制 |
| 2 | `02_joint_cartesian_read.cpp` | 读取关节位置、速度、力矩、笛卡尔位姿 |
| 3 | `03_kinematics.cpp` | 正运动学 (FK)、逆运动学 (IK) 计算 |

### 第4.4节 - 非实时运动控制接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 4 | `04_motion_basic.cpp` | MoveAbsJ 轴空间绝对运动 |
| 5 | `05_motion_cartesian.cpp` | MoveJ/MoveL/MoveC 笛卡尔空间运动 |

### 第4.6节 - IO与通信接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 6 | `06_io_control.cpp` | DI/DO/AI/AO 读写、仿真模式 |

### 第4.8节 - 协作机器人专属接口

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 7 | `07_safety_collision.cpp` | 碰撞检测、软限位、拖动示教 |
| 8 | `08_path_record_replay.cpp` | 路径录制、保存、回放 |

### 综合示例

| 示例 | 文件名 | 功能 |
|------|--------|------|
| 09 | `09_advanced_sdk_compat.cpp` | 高级 SDK 兼容能力（RT/RL/寄存器/动力学/奇异规避） |
| 99 | `99_complete_demo.cpp` | 完整功能演示流程 |

---

## 使用方法

### 1. 编译
所有示例在 `colcon build` 时会自动编译。

### 2. 运行
```bash
# 先启动仿真
ros2 launch rokae_xmate3_ros2 xmate3_simulation.launch.py

# 在新终端运行示例
ros2 run rokae_xmate3_ros2 example_01_basic_connect
```

### 3. 查看源代码
所有示例都包含详细的中文注释，可以直接阅读学习：
```bash
# 查看示例1的源代码
cat ~/ros2_ws0/src/rokae_xmate3_ros2/examples/cpp/01_basic_connect.cpp
```

---

## 快速参考

### 编译后的可执行文件
| 可执行文件名 | 说明 |
|------------|------|
| `example_01_basic_connect` | 示例1: 基础连接 |
| `example_02_joint_cartesian_read` | 示例2: 关节/位姿读取 |
| `example_03_kinematics` | 示例3: 运动学计算 |
| `example_04_motion_basic` | 示例4: 基础运动 |
| `example_05_motion_cartesian` | 示例5: 笛卡尔运动 |
| `example_06_io_control` | 示例6: IO控制 |
| `example_07_safety_collision` | 示例7: 安全功能 |
| `example_08_path_record_replay` | 示例8: 路径录制回放 |
| `example_99_complete_demo` | 综合演示 |

---

## 学习路径建议

### 初学者
1. `99_complete_demo.cpp` - 先看整体流程
2. `01_basic_connect.cpp` - 学习初始化
3. `04_motion_basic.cpp` - 学习基础运动

### 进阶学习
4. `02_joint_cartesian_read.cpp` - 状态读取
5. `05_motion_cartesian.cpp` - 笛卡尔运动
6. `03_kinematics.cpp` - 运动学

### 高级功能
7. `06_io_control.cpp` - IO控制
8. `07_safety_collision.cpp` - 安全功能
9. `08_path_record_replay.cpp` - 路径操作
10. `09_advanced_sdk_compat.cpp` - RT/RL/寄存器/动力学/奇异规避

---

## 注意事项

1. **仿真环境**: 所有示例都需要先启动 `xmate3_simulation.launch.py`（或 `simulation.launch.py`）
2. **错误处理**: 示例中包含完整的错误码 (std::error_code) 处理
3. **命名约定**: API完全遵循 xCore SDK C++ 使用手册 的命名
4. **章节对应**: 每个示例文件头都标明了对应的手册章节

---

## 更多资源

- [xCore SDK C++ 使用手册](../docs/)
- [API 参考](../docs/api/)
- [快速入门指南](../docs/QUICKSTART.md)
- [主 README](../README.md)
