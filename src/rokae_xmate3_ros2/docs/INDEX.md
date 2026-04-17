# 文档索引

> 状态：Active  
> 受众：使用者 / SDK 集成者 / Runtime 维护者 / 发布与审计人员  
> 作用：当前文档树唯一总入口  
> 最后校验：2026-04-17

## 0. 先理解这套文档怎么分层

当前**主文档**只有以下 7 份：

- [`QUICKSTART.md`](QUICKSTART.md)
- [`COMPATIBILITY.md`](COMPATIBILITY.md)
- [`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- [`ARCHITECTURE.md`](ARCHITECTURE.md)
- [`KINEMATICS_AND_MODEL.md`](KINEMATICS_AND_MODEL.md)
- [`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- [`EXAMPLES.md`](EXAMPLES.md)

除这 7 份之外，`docs/` 根目录里的其他旧文件名默认都属于以下三类之一：

- **Compatibility Redirect**：为旧链接、脚本锚点、历史引用保留的兼容跳转页
- **Maintenance Pointer**：维护/审计/硬化工作用入口，不属于用户主说明
- **Archived Pointer**：已归档历史阶段文档的保留入口

换言之：**未出现在“主文档清单”里的根目录文档，不应被视为当前主题的主说明。**

## 1. 我只想尽快跑起来

1. [`QUICKSTART.md`](QUICKSTART.md) — 环境、构建、启动、示例
2. [`EXAMPLES.md`](EXAMPLES.md) — public/internal 示例分层与运行方式
3. [`BUILD_RELEASE.md`](BUILD_RELEASE.md) — target environment、artifact、发布门禁

## 2. 我是 public SDK 集成者

1. [`COMPATIBILITY.md`](COMPATIBILITY.md) — xMate6 public lane 范围、ABI、对齐矩阵
2. [`QUICKSTART.md`](QUICKSTART.md) — 安装态 `xCoreSDK::xCoreSDK_static/shared` 使用方式
3. [`EXAMPLES.md`](EXAMPLES.md) — public 示例清单

## 3. 我是 runtime / 仿真维护者

1. [`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md) — profile、query authority、RT/NRT 规则
2. [`ARCHITECTURE.md`](ARCHITECTURE.md) — runtime 主链、扩展规则、fidelity 语义
3. [`KINEMATICS_AND_MODEL.md`](KINEMATICS_AND_MODEL.md) — kinematics/model 溯源与精度等级
4. [`BUILD_RELEASE.md`](BUILD_RELEASE.md) — source-tree 依赖与 release gate

## 4. 我是发布 / 审计人员

1. [`BUILD_RELEASE.md`](BUILD_RELEASE.md) — 环境锁定、artifact、验证分级
2. [`IMPLEMENTATION_AUDIT.md`](IMPLEMENTATION_AUDIT.md) — 当前实现审计（Maintenance Pointer）
3. [`HARDENING_BACKLOG.md`](HARDENING_BACKLOG.md) — 剩余硬化项（Maintenance Pointer）
4. [`archive/`](archive/) — 历史实施摘要与复核归档

## 5. 主文档清单

- [`COMPATIBILITY.md`](COMPATIBILITY.md)
- [`RUNTIME_PROFILES.md`](RUNTIME_PROFILES.md)
- [`ARCHITECTURE.md`](ARCHITECTURE.md)
- [`KINEMATICS_AND_MODEL.md`](KINEMATICS_AND_MODEL.md)
- [`BUILD_RELEASE.md`](BUILD_RELEASE.md)
- [`EXAMPLES.md`](EXAMPLES.md)
- [`QUICKSTART.md`](QUICKSTART.md)

## 6. 兼容跳转页（Compatibility Redirect）

以下旧文件名仍保留，但它们不再是主说明：
- `COMPAT_ABI.md`
- `API_ALIGNMENT_MATRIX.md`
- `XMATE6_OFFICIAL_ALIGNMENT_MATRIX.md`
- `PROFILE_CAPABILITY_MATRIX.md`
- `PROFILE_QUERY_POLICY.md`
- `RT_PROFILE_GUIDE.md`
- `RT_HARDENING_PROFILE.md`
- `EXTENSION_FRAMEWORK.md`
- `FIDELITY_POLICY.md`
- `RUNTIME_CATALOG_POLICY.md`
- `KINEMATICS_POLICY.md`
- `MODEL_TRACEABILITY.md`
- `ENVIRONMENT_LOCK.md`
- `PUBLIC_SDK_ARTIFACT.md`

## 7. 维护与归档入口

- Maintenance Pointer：`IMPLEMENTATION_AUDIT.md`、`HARDENING_BACKLOG.md`
- Archived Pointer：`P0_1_*`
- 历史正文：[`archive/`](archive/)
