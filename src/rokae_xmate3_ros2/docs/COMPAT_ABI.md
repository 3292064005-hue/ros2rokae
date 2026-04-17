# Compatibility ABI

> 状态：Compatibility Redirect  
> 角色：旧链接 / 旧脚本锚点 / 历史引用兼容入口  
> 当前主说明：[`COMPATIBILITY.md`](COMPATIBILITY.md)  
> 是否为当前主说明：否  
> 最后校验：2026-04-17

## 如何使用本页

- 需要当前有效规则时，直接跳转到 [`COMPATIBILITY.md`](COMPATIBILITY.md)。
- 只有在旧链接、旧脚本锚点或历史审计引用必须保留时，才继续保留此页。

## 保留锚点

- install-facing `xCoreSDK` consumer contract 只面向 xMate6 public lane
- `run_install_tree_consumer.py` 仍是 install-tree consumer 验证入口
- `check_exported_symbols.py` 仍是 exported symbol gate
- `MoveAppend` success means **queue accepted**
- `calibrateFrame()` remains `function_not_supported`
- single-source manifest
- strict 1kHz fail-fast RT profile
