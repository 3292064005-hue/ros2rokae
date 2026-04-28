#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FAILURES: list[str] = []

hpp = (ROOT / 'src/runtime/runtime_snapshots.hpp').read_text(encoding='utf-8')
cpp = (ROOT / 'src/runtime/program_state.cpp').read_text(encoding='utf-8')
path_facade = (ROOT / 'src/runtime/path_facade.cpp').read_text(encoding='utf-8')
request_adapter = (ROOT / 'src/runtime/request_adapter.cpp').read_text(encoding='utf-8')
request_coordinator_text = (ROOT / 'src/runtime/request_coordinator.cpp').read_text(encoding='utf-8')
doc = (ROOT / 'docs/reference/RECORDED_PATH_SCHEMA.md').read_text(encoding='utf-8')
manifest = json.loads((ROOT / 'docs/reference/recorded_path_schema_manifest.json').read_text(encoding='utf-8'))

for token in [
    'kRecordedPathSchemaVersion = "v2"',
    'kRecordedPathLegacySchemaVersion = "v1"',
    'kRecordedPathRobotFamily = "xMateER3"',
    'kRecordedPathRobotModel = rokae_xmate3_ros2::spec::xmate_er3_truth::kRobotModelName',
    'kRecordedPathCanonicalIdentity = rokae_xmate3_ros2::spec::xmate_er3_truth::kCanonicalIdentity',
    'kRecordedPathMonotonicStepSec = 0.01',
    'kRecordedPathDefaultTaskPhase = "path_record"',
    'bool has_end_pose = false;',
    'bool has_contact_force = false;',
    'std::string image_frame_id;',
    'std::string task_phase{kRecordedPathDefaultTaskPhase};',
    'std::string source_id{"sdk_record"};',
    'isReplayPathSchemaVersionSupported',
    'normalizeReplayPathAssetForConsumption',
    'ReplayPathConsumptionTarget',
    'ReplayPathAnalysisInput',
    'ReplayPathReportSummary',
    'ReplayPathConsumptionReport',
    'buildReplayPathConsumptionReport',
    'buildReplayPathAnalysisInput',
    'buildReplayPathReportSummary',
    'validateReplayPathAssetForConsumption',
]:
    if token not in hpp:
        FAILURES.append(f'missing runtime_snapshots contract token: {token}')

for token in [
    'asset.metadata.version = kRecordedPathSchemaVersion;',
    'asset.metadata.robot = kRecordedPathRobotFamily;',
    'asset.metadata.robot_model = kRecordedPathRobotModel;',
    'asset.metadata.canonical_identity = kRecordedPathCanonicalIdentity;',
    'asset.metadata.monotonic_step_sec = kRecordedPathMonotonicStepSec;',
    'recorded_path_.back().time_from_start_sec + kRecordedPathMonotonicStepSec',
    'sample.task_phase = kRecordedPathDefaultTaskPhase;',
    'sample.source_id = record_source_;',
    'normalizeReplayPathAssetForConsumption(asset);',
    'buildReplayPathConsumptionReport(asset);',
    'ReplayPathConsumptionTarget::replay',
]:
    if token not in cpp:
        FAILURES.append(f'missing program_state contract token: {token}')

for token in [
    'Path schema version is not supported',
    'normalizeReplayPathAssetForConsumption(replay_asset);',
    'buildReplayPathConsumptionReport(replay_asset);',
    'buildReplayPathAnalysisInput(asset, analysis, &analysis_error)',
    'buildReplayPathReportSummary(asset, report_summary, &report_error)',
]:
    if token not in path_facade:
        FAILURES.append(f'path_facade missing schema-consumer token: {token}')

for token in [
    'isReplayPathSchemaVersionSupported(replay_asset.metadata.version)',
    'normalizeReplayPathAssetForConsumption(normalized_asset);',
    'buildReplayPathConsumptionReport(normalized_asset);',
]:
    if token not in request_adapter:
        FAILURES.append(f'request_adapter missing schema compatibility token: {token}')
for token in [
    'buildReplayPathReportSummary(replay_asset, replay_report, &replay_report_error)',
]:
    if token not in request_coordinator_text:
        FAILURES.append(f'request_coordinator missing schema-consumer token: {token}')

for token in [
    'version = v2',
    'v1 -> v2',
    'task_phase',
    'source_id',
    'ReplayPathConsumptionReport',
    'buildReplayPathAnalysisInput()',
    'buildReplayPathReportSummary()',
    'analysis readiness 只有在每个 sample 都具备',
    'core 层必须拒绝空 name、空 buffer 和 replay 契约校验失败的资产',
]:
    if token not in doc:
        FAILURES.append(f'docs/reference/RECORDED_PATH_SCHEMA.md missing token: {token}')

if manifest.get('canonical_identity') != 'xCoreSDK:xmate_er3':
    FAILURES.append('recorded_path_schema_manifest.json canonical_identity mismatch')
if manifest.get('metadata', {}).get('version') != 'v2':
    FAILURES.append('recorded_path_schema_manifest.json version must be v2')
if manifest.get('compatibility', {}).get('normalize_legacy_to') != 'v2':
    FAILURES.append('recorded_path_schema_manifest.json must declare v1->v2 normalization')
if manifest.get('sample', {}).get('task_phase') != 'required string':
    FAILURES.append('recorded_path_schema_manifest.json task_phase contract mismatch')
consumer_chain = manifest.get('consumer_chain', {})
expected_analysis = ['src/runtime/runtime_snapshots.hpp:buildReplayPathAnalysisInput', 'src/runtime/path_facade.cpp:build_consumption_suffix']
if consumer_chain.get('analysis') != expected_analysis:
    FAILURES.append('recorded_path_schema_manifest.json analysis consumer chain mismatch')
expected_report = ['src/runtime/runtime_snapshots.hpp:buildReplayPathReportSummary', 'src/runtime/request_coordinator.cpp:submitReplayPath', 'src/runtime/path_facade.cpp:build_consumption_suffix']
if consumer_chain.get('report') != expected_report:
    FAILURES.append('recorded_path_schema_manifest.json report consumer chain mismatch')
if manifest.get('consumption_targets') != ['replay', 'analysis', 'report', 'all']:
    FAILURES.append('recorded_path_schema_manifest.json consumption_targets mismatch')

if FAILURES:
    print('recorded path schema check failed:')
    for failure in FAILURES:
        print(f'- {failure}')
    sys.exit(1)

print('recorded path schema check passed')
