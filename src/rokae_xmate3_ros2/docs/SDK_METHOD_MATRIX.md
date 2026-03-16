# SDK 方法覆盖矩阵

| 章节 | 方法 | 状态 | 说明 |
|---|---|---|---|
| 4.3 基本操作与信息查询 | `connectToRobot` | ✅ |  |
| 4.3 基本操作与信息查询 | `disconnectFromRobot` | ✅ |  |
| 4.3 基本操作与信息查询 | `robotInfo` | ✅ |  |
| 4.3 基本操作与信息查询 | `powerState` | ✅ |  |
| 4.3 基本操作与信息查询 | `setPowerState` | ✅ |  |
| 4.3 基本操作与信息查询 | `operateMode` | ✅ |  |
| 4.3 基本操作与信息查询 | `setOperateMode` | ✅ |  |
| 4.3 基本操作与信息查询 | `operationState` | ✅ |  |
| 4.3 基本操作与信息查询 | `posture` | ✅ |  |
| 4.3 基本操作与信息查询 | `cartPosture` | ✅ |  |
| 4.3 基本操作与信息查询 | `jointPos` | ✅ |  |
| 4.3 基本操作与信息查询 | `jointVel` | ✅ |  |
| 4.3 基本操作与信息查询 | `jointTorque` | ✅ |  |
| 4.3 基本操作与信息查询 | `baseFrame` | ✅ |  |
| 4.3 基本操作与信息查询 | `toolset` | ✅ |  |
| 4.3 基本操作与信息查询 | `setToolset` | ✅ |  |
| 4.3 基本操作与信息查询 | `calcIk` | ✅ |  |
| 4.3 基本操作与信息查询 | `calcFk` | ✅ |  |
| 4.3 基本操作与信息查询 | `clearServoAlarm` | ✅ |  |
| 4.3 基本操作与信息查询 | `queryControllerLog` | ✅ |  |
| 4.3 基本操作与信息查询 | `enableCollisionDetection` | ✅ |  |
| 4.3 基本操作与信息查询 | `disableCollisionDetection` | ✅ |  |
| 4.3 基本操作与信息查询 | `calibrateFrame` | ✅ | 已补齐 Gazebo 近似标定实现 |
| 4.3 基本操作与信息查询 | `getSoftLimit` | ✅ |  |
| 4.3 基本操作与信息查询 | `setSoftLimit` | ✅ |  |
| 4.3 基本操作与信息查询 | `sdkVersion` | ✅ |  |
| 4.4 非实时运动控制 | `setMotionControlMode` | ✅ |  |
| 4.4 非实时运动控制 | `moveStart` | ✅ |  |
| 4.4 非实时运动控制 | `moveReset` | ✅ |  |
| 4.4 非实时运动控制 | `stop` | ✅ |  |
| 4.4 非实时运动控制 | `moveAppend` | ✅ |  |
| 4.4 非实时运动控制 | `setDefaultSpeed` | ✅ |  |
| 4.4 非实时运动控制 | `setDefaultZone` | ✅ |  |
| 4.4 非实时运动控制 | `setDefaultConfOpt` | ✅ |  |
| 4.4 非实时运动控制 | `startJog` | ✅ |  |
| 4.4 非实时运动控制 | `adjustSpeedOnline` | ✅ |  |
| 4.4 非实时运动控制 | `setEventWatcher` | ✅ |  |
| 4.4 非实时运动控制 | `queryEventInfo` | ✅ |  |
| 4.4 非实时运动控制 | `executeCommand` | ✅ |  |
| 4.4 非实时运动控制 | `lastErrorCode` | ✅ |  |
| 4.5 实时运动控制 | `getRtMotionController` | ✅ |  |
| 4.5 实时运动控制 | `reconnectNetwork` | ✅ |  |
| 4.5 实时运动控制 | `disconnectNetwork` | ✅ | 会停止运动并清空状态接收缓存 |
| 4.5 实时运动控制 | `setControlLoop` | ✅ | 已修复 setFinished() 结束语义 |
| 4.5 实时运动控制 | `startLoop` | ✅ | 已修复非阻塞循环退出问题 |
| 4.5 实时运动控制 | `stopLoop` | ✅ |  |
| 4.5 实时运动控制 | `startMove` | ✅ |  |
| 4.5 实时运动控制 | `stopMove` | ✅ |  |
| 4.5 实时运动控制 | `startReceiveRobotState` | ✅ | 会预热一帧状态缓存 |
| 4.5 实时运动控制 | `stopReceiveRobotState` | ✅ |  |
| 4.5 实时运动控制 | `updateRobotState` | ✅ | 已补齐 jointAcc_c / tauVel_c 估计 |
| 4.5 实时运动控制 | `getStateData` | ✅ |  |
| 4.5 实时运动控制 | `MoveJ` | ✅ |  |
| 4.5 实时运动控制 | `MoveL` | ✅ |  |
| 4.5 实时运动控制 | `MoveC` | ✅ |  |
| 4.5 实时运动控制 | `setFilterLimit` | ✅ |  |
| 4.5 实时运动控制 | `setCartesianLimit` | ✅ |  |
| 4.5 实时运动控制 | `setJointImpedance` | ✅ |  |
| 4.5 实时运动控制 | `setCartesianImpedance` | ✅ |  |
| 4.5 实时运动控制 | `setCollisionBehaviour` | ✅ | 已补齐阈值->仿真碰撞灵敏度映射 |
| 4.5 实时运动控制 | `setEndEffectorFrame` | ✅ |  |
| 4.5 实时运动控制 | `setLoad` | ✅ |  |
| 4.5 实时运动控制 | `setFilterFrequency` | ✅ |  |
| 4.5 实时运动控制 | `setCartesianImpedanceDesiredTorque` | ✅ |  |
| 4.5 实时运动控制 | `setTorqueFilterCutOffFrequency` | ✅ |  |
| 4.5 实时运动控制 | `setFcCoor` | ✅ |  |
| 4.5 实时运动控制 | `automaticErrorRecovery` | ✅ | 已补齐兼容入口（已弃用） |
| 4.5 实时运动控制 | `setRtNetworkTolerance` | ✅ |  |
| 4.5 实时运动控制 | `useRciClient` | ✅ |  |
| 4.6 通信 | `getDI` | ✅ |  |
| 4.6 通信 | `setDI` | ✅ |  |
| 4.6 通信 | `getDO` | ✅ |  |
| 4.6 通信 | `setDO` | ✅ |  |
| 4.6 通信 | `getAI` | ✅ |  |
| 4.6 通信 | `setAO` | ✅ |  |
| 4.6 通信 | `setSimulationMode` | ✅ |  |
| 4.6 通信 | `readRegister` | ✅ |  |
| 4.6 通信 | `writeRegister` | ✅ |  |
| 4.6 通信 | `setxPanelVout` | ✅ |  |
| 4.7 RL 工程 | `projectsInfo` | ✅ |  |
| 4.7 RL 工程 | `loadProject` | ✅ |  |
| 4.7 RL 工程 | `ppToMain` | ✅ |  |
| 4.7 RL 工程 | `runProject` | ✅ |  |
| 4.7 RL 工程 | `pauseProject` | ✅ |  |
| 4.7 RL 工程 | `setProjectRunningOpt` | ✅ |  |
| 4.7 RL 工程 | `toolsInfo` | ✅ |  |
| 4.7 RL 工程 | `wobjsInfo` | ✅ |  |
| 4.8 协作相关 | `enableDrag` | ✅ |  |
| 4.8 协作相关 | `disableDrag` | ✅ |  |
| 4.8 协作相关 | `getEndTorque` | ✅ |  |
| 4.8 协作相关 | `startRecordPath` | ✅ |  |
| 4.8 协作相关 | `stopRecordPath` | ✅ |  |
| 4.8 协作相关 | `cancelRecordPath` | ✅ |  |
| 4.8 协作相关 | `saveRecordPath` | ✅ |  |
| 4.8 协作相关 | `replayPath` | ✅ |  |
| 4.8 协作相关 | `removePath` | ✅ |  |
| 4.8 协作相关 | `queryPathLists` | ✅ |  |
| 4.8 协作相关 | `setAvoidSingularity` | ✅ |  |
| 4.8 协作相关 | `getAvoidSingularity` | ✅ |  |