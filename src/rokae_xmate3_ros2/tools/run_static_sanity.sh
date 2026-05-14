#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TMP_PYCACHE="$(mktemp -d)"
trap 'rm -rf "${TMP_PYCACHE}"' EXIT

PYTHON_BIN="${ROKAE_PYTHON_EXECUTABLE:-$(command -v python3 || true)}"
if [ -z "${PYTHON_BIN}" ]; then
  echo "static_sanity: python3 not found" >&2
  exit 72
fi

export PYTHONDONTWRITEBYTECODE=1

PYTHONPYCACHEPREFIX="${TMP_PYCACHE}" "${PYTHON_BIN}" -B -m py_compile \
  "${PKG_ROOT}"/launch/*.py \
  "${PKG_ROOT}"/launch/_simulation_support.py \
  "${PKG_ROOT}"/tools/render_robot_description.py \
  "${PKG_ROOT}"/tools/check_gazebo_jtc_readiness.py \
  "${PKG_ROOT}"/test/harness/nrt_path_latency_probe.py \
  "${PKG_ROOT}"/test/harness/nrt_path_smoothness_probe.py \
  "${PKG_ROOT}"/test/harness/check_repo_contract.py \
  "${PKG_ROOT}"/test/harness/check_render_robot_description_install_tree.py \
  "${PKG_ROOT}"/test/harness/check_provider_boundary.py \
  "${PKG_ROOT}"/test/harness/check_public_internal_boundary.py \
  "${PKG_ROOT}"/test/harness/check_public_sdk_packaging_contract.py \
  "${PKG_ROOT}"/test/harness/check_docs_layout.py \
  "${PKG_ROOT}"/test/harness/check_recorded_path_schema.py \
  "${PKG_ROOT}"/test/harness/check_cpp_signature_sync.py \
  "${PKG_ROOT}"/test/harness/check_official_cpp_sdk_oracle.py \
  "${PKG_ROOT}"/test/harness/check_capability_matrix_contract.py \
  "${PKG_ROOT}"/test/harness/check_runtime_state_machine_contract.py \
  "${PKG_ROOT}"/test/harness/check_acceptance_layers.py \
  "${PKG_ROOT}"/test/harness/check_xmate_er3_alignment_behaviors.py \
  "${PKG_ROOT}"/test/harness/check_runtime_source_integrity.py \
  "${PKG_ROOT}"/test/harness/check_full_source_build_gate_contract.py \
  "${PKG_ROOT}"/test/harness/check_target_env_acceptance_report_contract.py \
  "${PKG_ROOT}"/test/harness/check_nrt_path_latency_gate_contract.py \
  "${PKG_ROOT}"/test/harness/check_nrt_path_smoothness_gate_contract.py \
  "${PKG_ROOT}"/tools/verify_target_env_acceptance_report.py

for script in "${PKG_ROOT}"/tools/*.sh; do
  bash -n "$script"
done

"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_repo_contract.py" "${PKG_ROOT}"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_render_robot_description_install_tree.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_provider_boundary.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_public_internal_boundary.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_public_sdk_packaging_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_docs_layout.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_recorded_path_schema.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_cpp_signature_sync.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_official_cpp_sdk_oracle.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_capability_matrix_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_runtime_state_machine_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_acceptance_layers.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_xmate_er3_alignment_behaviors.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_runtime_source_integrity.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_full_source_build_gate_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_target_env_acceptance_report_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_nrt_path_latency_gate_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_nrt_path_smoothness_gate_contract.py"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_release_gate_workflow.py"

"${PYTHON_BIN}" - "${PKG_ROOT}" <<'PY'
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
text_suffixes = {
    '.cpp', '.hpp', '.h', '.ipp', '.py', '.sh', '.cmake', '.md', '.srv', '.msg', '.action', '.xacro', '.xml', '.yaml', '.yml'
}
for path in root.rglob('*'):
    if not path.is_file():
        continue
    name = path.name
    if name == '.gitkeep':
        continue
    should_check = path.suffix in text_suffixes or name.endswith('.launch.py')
    if not should_check:
        continue
    payload = path.read_bytes()
    if b'\x00' in payload:
        raise SystemExit(f'embedded NUL byte detected in {path}')
PY
