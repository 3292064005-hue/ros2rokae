#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOUSAGE'
usage: check_target_environment.sh [--quiet]

Validate the locked target environment for this package.
Expected baseline:
  - Ubuntu 22.04
  - ROS 2 Humble
  - gazebo / gazebo_ros available from the sourced ROS environment
  - colcon, rosdep, ros2, xacro, gazebo available on PATH

Set ROKAE_IGNORE_ENV_LOCK=1 to bypass this check intentionally.
EOUSAGE
}

QUIET=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --quiet)
      QUIET=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      exit 64
      ;;
  esac
done

if [ "${ROKAE_IGNORE_ENV_LOCK:-0}" = "1" ]; then
  [ "$QUIET" -eq 1 ] || echo "environment-lock: bypassed by ROKAE_IGNORE_ENV_LOCK=1"
  exit 0
fi

fail() {
  echo "environment-lock: $1" >&2
  exit 78
}

if [ -r /etc/os-release ]; then
  . /etc/os-release
else
  fail "/etc/os-release not found"
fi

if [ "${ID:-}" != "ubuntu" ] || [ "${VERSION_ID:-}" != "22.04" ]; then
  fail "expected Ubuntu 22.04, got ${ID:-unknown} ${VERSION_ID:-unknown}"
fi

if [ "${ROS_DISTRO:-}" != "humble" ]; then
  fail "expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}"
fi

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  fail "missing /opt/ros/humble/setup.bash"
fi

for cmd in colcon rosdep ros2 xacro gazebo; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    fail "required command '$cmd' is not available on PATH"
  fi
done

if ! python3 - <<'EOPY' >/dev/null 2>&1
import importlib.util
mods = ['ament_index_python', 'launch', 'xacro']
missing = [m for m in mods if importlib.util.find_spec(m) is None]
raise SystemExit(1 if missing else 0)
EOPY
then
  fail "required Python ROS tooling modules are not importable (ament_index_python, launch, xacro)"
fi

if ! pkg-config --exists gazebo; then
  fail "gazebo pkg-config metadata is unavailable"
fi

if ! rosdep db >/dev/null 2>&1; then
  fail "rosdep database is not initialized (run rosdep init/update)"
fi

if ! ros2 pkg prefix gazebo_ros >/dev/null 2>&1; then
  fail "required ROS package 'gazebo_ros' is not discoverable from the sourced environment"
fi

[ "$QUIET" -eq 1 ] || echo "environment-lock: OK (Ubuntu 22.04, ROS 2 Humble, Gazebo available)"
