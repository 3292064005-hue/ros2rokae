# P0-1 implementation summary

This repository revision implements the install-facing ABI split for the xMate 6-axis compatibility lane.

## Landed items

- split backend SDK target from install-facing compatibility SDK targets
- added `xCoreSDK_static` / `xCoreSDK_shared` compatibility libraries
- replaced public `rokae/*.h` shim-based entrypoints with compiled-ABI declarations
- moved compatibility method bodies into `src/compat/*.cpp`
- made `rokae/data_types.h` and `rokae/utility.h` standalone public headers
- restricted installed public headers to `include/rokae/*`
- added `xCoreSDKConfig.cmake` package export
- added source-tree public ABI pollution check and install-tree consumer skeleton

## Explicit scope decisions

- target robot: xMate 6-axis only
- this change set does not claim controller-grade RT fidelity
- this change set does not harden RL semantics or calibration semantics

## Main files

- `CMakeLists.txt`
- `cmake/targets_sdk_backend.cmake`
- `cmake/targets_sdk_compat.cmake`
- `cmake/targets_packaging.cmake`
- `cmake/xCoreSDKConfig.cmake.in`
- `include/rokae/*.h`
- `src/compat/*.cpp`
- `test/harness/check_compat_public_abi.py`
- `test/compat/install_tree/*`
