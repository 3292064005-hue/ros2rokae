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

PYTHONPYCACHEPREFIX="${TMP_PYCACHE}" "${PYTHON_BIN}" -B -m py_compile \
  "${PKG_ROOT}"/launch/*.py \
  "${PKG_ROOT}"/launch/_simulation_support.py \
  "${PKG_ROOT}"/tools/render_robot_description.py \
  "${PKG_ROOT}"/test/harness/check_repo_contract.py \
  "${PKG_ROOT}"/test/harness/check_render_robot_description_install_tree.py

for script in "${PKG_ROOT}"/tools/*.sh; do
  bash -n "$script"
done

"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_repo_contract.py" "${PKG_ROOT}"
"${PYTHON_BIN}" "${PKG_ROOT}/test/harness/check_render_robot_description_install_tree.py"

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
