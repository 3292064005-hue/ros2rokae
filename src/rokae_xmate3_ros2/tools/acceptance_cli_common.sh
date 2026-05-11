#!/usr/bin/env bash
set -euo pipefail

rokae_acceptance_source_file() {
  local setup_file="$1"
  local had_nounset=0
  case "$-" in
    *u*) had_nounset=1 ;;
  esac
  set +u
  # shellcheck disable=SC1090
  . "${setup_file}"
  if [ "${had_nounset}" -eq 1 ]; then
    set -u
  fi
}

rokae_acceptance_source_env() {
  local workspace_root="$1"
  rokae_acceptance_source_ros_env
  if [ -f "${workspace_root}/install/setup.bash" ]; then
    rokae_acceptance_source_file "${workspace_root}/install/setup.bash"
  fi
}

rokae_acceptance_source_ros_env() {
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    rokae_acceptance_source_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [ -f "/opt/ros/humble/setup.bash" ]; then
    rokae_acceptance_source_file /opt/ros/humble/setup.bash
  else
    echo "acceptance: ROS environment is not available" >&2
    return 78
  fi
}

rokae_acceptance_wait_for_service() {
  local service_name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if ros2 service list | grep -Fxq "${service_name}"; then
      return 0
    fi
    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "acceptance: timeout waiting for service ${service_name}" >&2
      return 1
    fi
    sleep 1
  done
}

rokae_acceptance_call_service() {
  local service_name="$1"
  local service_type="$2"
  local payload="$3"
  local response
  if ! response="$(ros2 service call "${service_name}" "${service_type}" "${payload}" 2>&1)"; then
    echo "${response}" >&2
    return 1
  fi
  echo "${response}"
  if ! grep -q "success=True" <<<"${response}"; then
    echo "acceptance: ${service_name} returned non-success response" >&2
    return 1
  fi
}

rokae_acceptance_wait_for_topic() {
  local topic_name="$1"
  local timeout_s="$2"
  local start_s
  start_s="$(date +%s)"
  while true; do
    if ros2 topic list | grep -Fxq "${topic_name}"; then
      return 0
    fi
    if (( "$(date +%s)" - start_s >= timeout_s )); then
      echo "acceptance: timeout waiting for topic ${topic_name}" >&2
      return 1
    fi
    sleep 1
  done
}


rokae_acceptance_package_root() {
  local script_dir="$1"
  (cd "${script_dir}/.." && pwd)
}

rokae_acceptance_tool_path() {
  local script_dir="$1"
  local workspace_root="$2"
  local tool_name="$3"
  local candidate="${script_dir}/${tool_name}"
  if [ -f "${candidate}" ]; then
    printf '%s
' "${candidate}"
    return 0
  fi
  echo "acceptance: missing sibling tool ${tool_name} for workspace ${workspace_root}" >&2
  return 1
}

rokae_acceptance_package_share() {
  local script_dir="$1"
  local workspace_root="$2"
  local pkg_name="${3:-rokae_xmate3_ros2}"
  local install_share="${workspace_root}/install/${pkg_name}/share/${pkg_name}"
  if [ -d "${install_share}" ]; then
    printf '%s\n' "${install_share}"
    return 0
  fi
  local package_root
  package_root="$(rokae_acceptance_package_root "${script_dir}")"
  if [ -d "${package_root}" ]; then
    printf '%s\n' "${package_root}"
    return 0
  fi
  return 1
}
