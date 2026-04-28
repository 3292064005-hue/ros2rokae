# 文档索引

> 状态：Active  
> 受众：使用者 / SDK 集成者 / Runtime 维护者 / 发布与审计人员  
> 作用：当前文档树唯一总入口  
> 最后校验：2026-04-18

## 1. 文档分层

当前文档树只有四层：

- `public/`：使用者与集成者主说明
- `architecture/`：架构与扩展边界
- `release/`：构建、发布、门禁、环境锁定
- `reference/`：machine-readable manifest 与对齐参考

历史阶段材料统一放在：
- `archive/`

## 2. 我只想尽快跑起来

1. [`public/QUICKSTART.md`](public/QUICKSTART.md)
2. [`public/EXAMPLES.md`](public/EXAMPLES.md)
3. [`release/BUILD_RELEASE.md`](release/BUILD_RELEASE.md)

## 3. 我是 public SDK 集成者

1. [`public/COMPATIBILITY.md`](public/COMPATIBILITY.md)
2. [`public/PUBLIC_SDK_ARTIFACT.md`](public/PUBLIC_SDK_ARTIFACT.md)
3. [`reference/SDK_ALIGNMENT.md`](reference/SDK_ALIGNMENT.md)
4. [`reference/RECORDED_PATH_SCHEMA.md`](reference/RECORDED_PATH_SCHEMA.md)

## 4. 我是 runtime / 仿真维护者

1. [`architecture/ARCHITECTURE.md`](architecture/ARCHITECTURE.md)
2. [`architecture/PROVIDER_BOUNDARY.md`](architecture/PROVIDER_BOUNDARY.md)
3. [`public/RUNTIME_PROFILES.md`](public/RUNTIME_PROFILES.md)
4. [`public/KINEMATICS_AND_MODEL.md`](public/KINEMATICS_AND_MODEL.md)
5. [`reference/RUNTIME_STATE_MACHINE.md`](reference/RUNTIME_STATE_MACHINE.md)

## 5. 我是发布 / 审计人员

1. [`release/BUILD_RELEASE.md`](release/BUILD_RELEASE.md)
2. [`release/ENVIRONMENT_LOCK.md`](release/ENVIRONMENT_LOCK.md)
3. [`release/RELEASE_GATE.md`](release/RELEASE_GATE.md)
4. [`release/ACCEPTANCE_LAYERS.md`](release/ACCEPTANCE_LAYERS.md)
5. [`archive/audits/IMPLEMENTATION_AUDIT.md`](archive/audits/IMPLEMENTATION_AUDIT.md)
6. [`release/HARDENING_BACKLOG.md`](release/HARDENING_BACKLOG.md)

## 6. 单一事实源

- machine-readable manifest：[`reference/xmate_er3_alignment_manifest.json`](reference/xmate_er3_alignment_manifest.json)
- 对齐参考：[`reference/SDK_ALIGNMENT.md`](reference/SDK_ALIGNMENT.md)

## 7. 已删除的旧页面

以下旧页已合并或删除，不再作为主说明：

- `COMPAT_ABI.md`
- `API_ALIGNMENT_MATRIX.md`
- `XMATE_ER3_OFFICIAL_ALIGNMENT_MATRIX.md`
- `RT_PROFILE_GUIDE.md`
- `RT_HARDENING_PROFILE.md`
- `PROFILE_CAPABILITY_MATRIX.md`
- `PROFILE_QUERY_POLICY.md`
- `RUNTIME_CATALOG_POLICY.md`
- `KINEMATICS_POLICY.md`
- `FIDELITY_POLICY.md`
- `MODEL_TRACEABILITY.md`
- `EXTENSION_FRAMEWORK.md`
- `P0_1_*` 根目录过程页
- `examples/PUBLIC_SDK_README.md`
- `docs/maintenance/HARDENING_BACKLOG.md` 重复副本
