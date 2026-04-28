# Target layering contract

本仓不拆多 ROS2 package，但运行与安装导出面按 target 分层冻结：

1. `rokae_xmate3_ros2_runtime_motion_core`
2. `rokae_xmate3_ros2_runtime_state`
3. `rokae_xmate3_ros2_runtime_facade`
4. `rokae_xmate3_ros2_runtime_ros_bridge`
5. `rokae_xmate3_ros2_runtime_control_bridge`
6. `rokae_xmate3_ros2_runtime_core`
7. `rokae_xmate3_ros2_runtime_test`
8. `xCoreSDK_static`
9. `xCoreSDK_shared`
10. `xCoreSDK_core`
11. `xCoreSDK_ros_bridge`

依赖方向：
- motion/state/facade/ros_bridge/control_bridge -> `runtime_core`
- `runtime_core` / `runtime_test` -> shared runtime assembly only
- `xCoreSDK_static` / `xCoreSDK_shared` -> compat facade + runtime object layers + sdk backend objects
- `xCoreSDK_core` -> SDK-shaped 最小消费面（公共头 + 模型/轨迹规划 core + Eigen），不再是 INTERFACE 空壳
- `xCoreSDK_ros_bridge` -> install-facing ROS2 runtime / action / service 桥接面
- install-facing public ABI 只导出 `include/rokae/*` 与 `xCoreSDK::*`
- backend/internal headers 继续受 `ROKAE_INSTALL_BACKEND_DEV_HEADERS` / `ROKAE_ENABLE_INTERNAL_SURFACE` 门控

该文件用于把 P1-01 的 target-level 分层显式化，避免再次被误判为“无实质分层”。
