#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TMP_PYCACHE="$(mktemp -d)"
trap 'rm -rf "${TMP_PYCACHE}"' EXIT
PYTHONPYCACHEPREFIX="${TMP_PYCACHE}" python3 -B -m py_compile "${PKG_ROOT}"/launch/*.py "${PKG_ROOT}"/launch/_simulation_support.py
for script in "${PKG_ROOT}"/tools/*.sh; do
  bash -n "$script"
done
python3 "${PKG_ROOT}/test/harness/check_repo_contract.py"
