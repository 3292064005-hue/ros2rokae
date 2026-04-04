# xCoreSDK compatibility ABI lane

This document records the install-facing ABI contract added for the xMate 6-axis compatibility lane.

## Scope

- concrete consumer entries: `rokae::Robot_T<rokae::WorkType::collaborative, 6>`, `rokae::xMateRobot`
- public headers: `rokae/robot.h`, `rokae/model.h`, `rokae/motion_control_rt.h`, `rokae/planner.h`, `rokae/data_types.h`, `rokae/utility.h`
- install-facing targets: `xCoreSDK::xCoreSDK_static` (official-shaped native static lane) and `xCoreSDK::xCoreSDK_shared` (dynamic compat lane)
- backend headers such as `rokae_xmate3_ros2/*` and shim headers such as `rokae/sdk_shim*.hpp` are source-tree implementation details and are not installed as part of the public SDK surface
- install packages now default to `ROKAE_STRICT_PUBLIC_INSTALL=ON`, which keeps backend developer headers and native/private CMake exports out of the install-facing SDK unless the packager explicitly enables them
- `ROKAE_EXPORT_PRIVATE_SDK_TARGETS=OFF` by default still hides native/backend developer targets from the install tree, but no longer changes the semantics of `xCoreSDK::xCoreSDK_static` itself

## Consumer contract

A downstream CMake project should be able to consume the installed compatibility package with the install-facing target set. Internal backend targets are exported separately through a private helper file and are not part of the supported public contract.

The current package is still **ROS2/Gazebo-backed**. `xCoreSDK::xCoreSDK_static` and `xCoreSDK::xCoreSDK_shared` are both real install-facing targets; the static lane now packages the compatibility façade together with the internal runtime/backend object code. Downstream configuration therefore resolves the same ROS2/Gazebo dependency set up front, and runtime execution still requires the ROS2/Gazebo shared libraries that the compatibility implementation links against.

```cmake
find_package(xCoreSDK CONFIG REQUIRED)
add_executable(app main.cpp)
target_link_libraries(app PRIVATE xCoreSDK::xCoreSDK_static)
```

And a source file should only need the installed public headers, for example:

```cpp
#include <rokae/robot.h>
#include <rokae/model.h>
#include <rokae/motion_control_rt.h>
#include <rokae/planner.h>
```

The install-facing headers now also retain the official-shaped convenience surface for the xMate6 lane:

- `rokae::Robot_T<rokae::WorkType::collaborative, 6>` is publicly instantiable from `rokae/robot.h`
- `connectToRobot(remoteIP, localIP = "")` is available on the public wrapper without exposing ROS-backed types
- soft-limit reads/writes accept the official `std::array<double[2], 6>` contract, while the older nested-array overload remains available for in-tree compatibility; omitting the limit array on enable keeps the current numeric soft-limit values instead of zeroing them
- non-realtime `executeCommand(std::vector<MoveCCommand>, ec)` is part of the install-facing ABI
- compat-only RT metadata fields `rokae::RtCompatFields::samplePeriod_s` / `rokae::RtCompatFields::sampleFresh` are additive ABI extensions for the ROS2/Gazebo-backed lane so `getStateData<double/bool>()` has real six-axis field sources; they are not part of the official xCore RT field set

## Verification assets

- source-tree ABI pollution check: `test/harness/check_compat_public_abi.py`
- install-tree consumer skeleton: `test/compat/install_tree/`
- staged install/build harness: `test/harness/run_install_tree_consumer.py`
- install-tree consumer coverage now includes `minimal_state_and_motion.cpp`, `minimal_static_link_only.cpp`, and `minimal_shared_link_only.cpp`
- shared-library symbol leak check: `test/harness/check_exported_symbols.py`

## Notes

- The current lane intentionally targets **xMate 6-axis only**.
- The current install package remains **ROS2/Gazebo-backed**; the public config now resolves the required ROS2/Gazebo dependency set before loading the exported targets, and `xCoreSDK::xCoreSDK_static` is a native static library rather than a compatibility alias.
- The compatibility ABI split does **not** claim controller-grade RT fidelity; it only hardens the install/build/consumption boundary.


- `ROKAE_INSTALL_INTERNAL_BACKEND_EXAMPLES=ON` 可显式安装 internal/backend example 二进制；默认 **OFF**，避免把源码树内部验证入口混入 public 安装面。


## 2026-04 command-bridge hardening

- The install-facing façade and the source-tree shim now route RT command publication through the same internal `rt_command_bridge` helper. This is an implementation refactor only; public ABI names and signatures remain unchanged.
- Install-facing SDK targets now export include directories via explicit `BUILD_INTERFACE` / `INSTALL_INTERFACE` generator expressions and `INCLUDES DESTINATION`, tightening install-tree safety without changing target names.

## Out-of-scope modules on the public xMate6 lane

The install-facing compatibility package now freezes the public ABI to the xMate6 lane. IO/register, RL, and calibration entry points remain present only for source compatibility; the public lane returns deterministic `SdkError::not_implemented` results and no longer treats those subsystems as supported install-tree contracts.
