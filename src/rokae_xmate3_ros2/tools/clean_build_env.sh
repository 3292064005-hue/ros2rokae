#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <command> [args...]" >&2
  exit 64
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

filter_path_list() {
  local raw_value="${1:-}"
  if [ -z "${raw_value}" ]; then
    return 0
  fi

  local filtered=()
  local entry=""
  local old_ifs="${IFS}"
  IFS=':'
  read -r -a entries <<< "${raw_value}"
  IFS="${old_ifs}"

  for entry in "${entries[@]}"; do
    if [ -z "${entry}" ]; then
      continue
    fi
    case "${entry}" in
      *conda*|*mamba*|*miniconda*|*anaconda*)
        continue
        ;;
    esac
    filtered+=("${entry}")
  done

  local joined=""
  local part=""
  for part in "${filtered[@]}"; do
    if [ -n "${joined}" ]; then
      joined="${joined}:"
    fi
    joined="${joined}${part}"
  done
  printf '%s' "${joined}"
}

unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_PROMPT_MODIFIER
unset CONDA_EXE
unset CONDA_PYTHON_EXE
unset _CONDA_EXE
unset _CONDA_ROOT
unset PYTHONHOME
unset PYTHONSTARTUP
unset PYTHONUSERBASE
unset CMAKE_PREFIX_PATH
unset CMAKE_LIBRARY_PATH
unset LIBRARY_PATH
unset CPATH
unset PKG_CONFIG_PATH

export PYTHONPATH="$(filter_path_list "${PYTHONPATH:-}")"
FILTERED_LD_LIBRARY_PATH="$(filter_path_list "${LD_LIBRARY_PATH:-}")"
export LD_LIBRARY_PATH="${FILTERED_LD_LIBRARY_PATH}"

SYSTEM_PATH="/usr/bin:/bin:/usr/sbin:/sbin"
FILTERED_PATH="$(filter_path_list "${PATH:-}")"
if [ -n "${FILTERED_PATH}" ]; then
  export PATH="${SYSTEM_PATH}:${FILTERED_PATH}"
else
  export PATH="${SYSTEM_PATH}"
fi

export Python3_EXECUTABLE="/usr/bin/python3"
export Python_EXECUTABLE="/usr/bin/python3"
export PYTHON_EXECUTABLE="/usr/bin/python3"
export AMENT_PYTHON_EXECUTABLE="/usr/bin/python3"
export ROKAE_CLEAN_ENV_ACTIVE="1"

exec "$@"
