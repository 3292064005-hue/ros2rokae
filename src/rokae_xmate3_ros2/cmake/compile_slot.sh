#!/usr/bin/env bash
set -euo pipefail

slots="${ROKAE_EXAMPLE_COMPILE_SLOTS:-4}"
if [[ ! "$slots" =~ ^[0-9]+$ ]] || [[ "$slots" -lt 1 ]]; then
  slots=4
fi

lock_dir="${TMPDIR:-/tmp}/rokae_example_compile_slots_${USER:-user}"
mkdir -p "$lock_dir"

while true; do
  for ((i = 1; i <= slots; ++i)); do
    lock_file="$lock_dir/slot_${i}.lock"
    exec {fd}>"$lock_file"
    if flock -n "$fd"; then
      exec "$@"
    fi
    eval "exec ${fd}>&-"
  done
  sleep 0.05
done
