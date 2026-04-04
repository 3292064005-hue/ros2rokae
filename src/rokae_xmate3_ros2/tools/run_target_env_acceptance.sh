#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOUSAGE'
usage: run_target_env_acceptance.sh [--engine docker|podman] [--image-tag TAG] [--release-gate] [--launch-smoke] [--local-target-env] [--report-dir DIR]

Build the locked Ubuntu 22.04 / ROS 2 Humble / Gazebo 11 image and execute the package gates
inside an isolated target-like container, or run the same bundle directly when already on the
locked target environment. A machine-readable acceptance report is always emitted into --report-dir,
including environment-check failures, image-build failures, and gate failures.
EOUSAGE
}

ENGINE="${ROKAE_CONTAINER_ENGINE:-docker}"
IMAGE_TAG="${ROKAE_ACCEPTANCE_IMAGE_TAG:-rokae_xmate3_ros2:humble-gazebo11}"
RUN_RELEASE_GATE=0
RUN_LAUNCH_SMOKE=0
RUN_LOCAL_TARGET_ENV=0
REPORT_DIR=""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --engine)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      ENGINE="$2"
      shift 2
      ;;
    --image-tag)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      IMAGE_TAG="$2"
      shift 2
      ;;
    --release-gate)
      RUN_RELEASE_GATE=1
      shift
      ;;
    --launch-smoke)
      RUN_LAUNCH_SMOKE=1
      shift
      ;;
    --local-target-env)
      RUN_LOCAL_TARGET_ENV=1
      shift
      ;;
    --report-dir)
      [ "$#" -ge 2 ] || { usage; exit 64; }
      REPORT_DIR="$2"
      shift 2
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

if [ -z "${REPORT_DIR}" ]; then
  REPORT_DIR="${PKG_ROOT}/artifacts/target_env_acceptance"
fi
mkdir -p "${REPORT_DIR}"

MODE="container"
REPORT_NAME="acceptance_report_container.json"
ENVIRONMENT_CHECK_STATUS="not_run"
ROSDEP_INSTALL_STATUS="not_run"
QUICK_GATE_STATUS="not_run"
RELEASE_GATE_STATUS="not_requested"
LAUNCH_SMOKE_STATUS="not_requested"
IMAGE_BUILD_STATUS="not_run"
ACCEPTANCE_STATUS="failed"
FAILURE_REASON=""

run_logged_step() {
  local log_path="$1"
  shift
  mkdir -p "$(dirname "${log_path}")"
  set +e
  "$@" >"${log_path}" 2>&1
  local status=$?
  set -e
  return ${status}
}

emit_report_host() {
  local workspace_root="$1"
  local package_root="$2"
  local output_path="$3"
  local report_args=(
    --workspace-root "${workspace_root}"
    --package-root "${package_root}"
    --output "${output_path}"
    --mode "${MODE}"
    --environment-check-status "${ENVIRONMENT_CHECK_STATUS}"
    --rosdep-install-status "${ROSDEP_INSTALL_STATUS}"
    --quick-gate-status "${QUICK_GATE_STATUS}"
    --release-gate-status "${RELEASE_GATE_STATUS}"
    --launch-smoke-status "${LAUNCH_SMOKE_STATUS}"
    --image-build-status "${IMAGE_BUILD_STATUS}"
    --acceptance-status "${ACCEPTANCE_STATUS}"
    --failure-reason "${FAILURE_REASON}"
  )
  if [ "${RUN_RELEASE_GATE}" = "1" ]; then
    report_args+=(--release-gate)
  fi
  if [ "${RUN_LAUNCH_SMOKE}" = "1" ]; then
    report_args+=(--launch-smoke)
  fi
  python3 "${PKG_ROOT}/tools/write_target_env_report.py" "${report_args[@]}"
}

run_local_bundle() {
  local tmp_ws="/tmp/rokae_acceptance_ws_local"
  local local_report="${REPORT_DIR}/acceptance_report_local.json"
  MODE="local"
  REPORT_NAME="acceptance_report_local.json"
  IMAGE_BUILD_STATUS="not_applicable"

  cleanup_local_bundle() {
    rm -rf "${tmp_ws}"
  }
  trap cleanup_local_bundle EXIT

  if [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck disable=SC1091
    . /opt/ros/humble/setup.bash
  fi

  if run_logged_step "${REPORT_DIR}/environment_check_local.log" \
      "${PKG_ROOT}/tools/check_target_environment.sh" --quiet; then
    ENVIRONMENT_CHECK_STATUS="passed"
  else
    ENVIRONMENT_CHECK_STATUS="failed"
    FAILURE_REASON="environment_check_failed"
    emit_report_host "${tmp_ws}" "${PKG_ROOT}" "${local_report}"
    return 1
  fi

  rm -rf "${tmp_ws}"
  mkdir -p "${tmp_ws}/src"
  cp -R "${PKG_ROOT}" "${tmp_ws}/src/rokae_xmate3_ros2"

  if run_logged_step "${REPORT_DIR}/rosdep_install_local.log" \
      rosdep install --from-paths "${tmp_ws}/src" --ignore-src -r -y; then
    ROSDEP_INSTALL_STATUS="passed"
  else
    ROSDEP_INSTALL_STATUS="failed"
    FAILURE_REASON="rosdep_install_failed"
    emit_report_host "${tmp_ws}" "${tmp_ws}/src/rokae_xmate3_ros2" "${local_report}"
    return 1
  fi

  if run_logged_step "${REPORT_DIR}/quick_gate_local.log" \
      "${tmp_ws}/src/rokae_xmate3_ros2/tools/run_quick_gate.sh" "${tmp_ws}"; then
    QUICK_GATE_STATUS="passed"
  else
    QUICK_GATE_STATUS="failed"
    FAILURE_REASON="quick_gate_failed"
    emit_report_host "${tmp_ws}" "${tmp_ws}/src/rokae_xmate3_ros2" "${local_report}"
    return 1
  fi

  if [ "${RUN_RELEASE_GATE}" = "1" ]; then
    if run_logged_step "${REPORT_DIR}/release_gate_local.log" \
        "${tmp_ws}/src/rokae_xmate3_ros2/tools/run_release_gate.sh" "${tmp_ws}"; then
      RELEASE_GATE_STATUS="passed"
    else
      RELEASE_GATE_STATUS="failed"
      FAILURE_REASON="release_gate_failed"
      emit_report_host "${tmp_ws}" "${tmp_ws}/src/rokae_xmate3_ros2" "${local_report}"
      return 1
    fi
  fi

  if [ "${RUN_LAUNCH_SMOKE}" = "1" ]; then
    if run_logged_step "${REPORT_DIR}/launch_smoke_local.log" \
        "${tmp_ws}/src/rokae_xmate3_ros2/tools/run_launch_smoke.sh" "${tmp_ws}"; then
      LAUNCH_SMOKE_STATUS="passed"
    else
      LAUNCH_SMOKE_STATUS="failed"
      FAILURE_REASON="launch_smoke_failed"
      emit_report_host "${tmp_ws}" "${tmp_ws}/src/rokae_xmate3_ros2" "${local_report}"
      return 1
    fi
  fi

  ACCEPTANCE_STATUS="passed"
  FAILURE_REASON=""
  emit_report_host "${tmp_ws}" "${tmp_ws}/src/rokae_xmate3_ros2" "${local_report}"
}

if [ "${RUN_LOCAL_TARGET_ENV}" = "1" ]; then
  run_local_bundle
  exit $?
fi

if ! command -v "$ENGINE" >/dev/null 2>&1; then
  FAILURE_REASON="container_engine_missing"
  emit_report_host "${PKG_ROOT}" "${PKG_ROOT}" "${REPORT_DIR}/${REPORT_NAME}"
  echo "acceptance: container engine '$ENGINE' is not available" >&2
  exit 69
fi

if run_logged_step "${REPORT_DIR}/image_build.log" \
    "$ENGINE" build -f "${PKG_ROOT}/docker/Dockerfile.humble-gazebo11" -t "${IMAGE_TAG}" "${PKG_ROOT}"; then
  IMAGE_BUILD_STATUS="passed"
else
  IMAGE_BUILD_STATUS="failed"
  FAILURE_REASON="image_build_failed"
  emit_report_host "${PKG_ROOT}" "${PKG_ROOT}" "${REPORT_DIR}/${REPORT_NAME}"
  exit 1
fi

CONTAINER_SCRIPT=$(cat <<'EOSCRIPT'
set -uo pipefail
. /opt/ros/humble/setup.bash
mkdir -p /artifacts
ENVIRONMENT_CHECK_STATUS="not_run"
ROSDEP_INSTALL_STATUS="not_run"
QUICK_GATE_STATUS="not_run"
RELEASE_GATE_STATUS="not_requested"
LAUNCH_SMOKE_STATUS="not_requested"
ACCEPTANCE_STATUS="failed"
FAILURE_REASON=""
TMP_WS="/tmp/rokae_acceptance_ws"
rm -rf "${TMP_WS}"
mkdir -p "${TMP_WS}/src"
cp -R /package "${TMP_WS}/src/rokae_xmate3_ros2"
run_logged_step() {
  local log_path="$1"
  shift
  set +e
  "$@" >"${log_path}" 2>&1
  local status=$?
  set -e
  return ${status}
}
emit_report() {
  report_args=(
    --workspace-root "${TMP_WS}"
    --package-root "${TMP_WS}/src/rokae_xmate3_ros2"
    --output /artifacts/acceptance_report_container.json
    --mode container
    --environment-check-status "${ENVIRONMENT_CHECK_STATUS}"
    --rosdep-install-status "${ROSDEP_INSTALL_STATUS}"
    --quick-gate-status "${QUICK_GATE_STATUS}"
    --release-gate-status "${RELEASE_GATE_STATUS}"
    --launch-smoke-status "${LAUNCH_SMOKE_STATUS}"
    --image-build-status passed
    --acceptance-status "${ACCEPTANCE_STATUS}"
    --failure-reason "${FAILURE_REASON}"
  )
  if [ "${RUN_RELEASE_GATE}" = "1" ]; then
    report_args+=(--release-gate)
  fi
  if [ "${RUN_LAUNCH_SMOKE}" = "1" ]; then
    report_args+=(--launch-smoke)
  fi
  python3 "${TMP_WS}/src/rokae_xmate3_ros2/tools/write_target_env_report.py" "${report_args[@]}"
}
if run_logged_step /artifacts/environment_check_container.log \
    "${TMP_WS}/src/rokae_xmate3_ros2/tools/check_target_environment.sh" --quiet; then
  ENVIRONMENT_CHECK_STATUS="passed"
else
  ENVIRONMENT_CHECK_STATUS="failed"
  FAILURE_REASON="environment_check_failed"
  emit_report
  exit 1
fi
if run_logged_step /artifacts/rosdep_install_container.log \
    rosdep install --from-paths "${TMP_WS}/src" --ignore-src -r -y; then
  ROSDEP_INSTALL_STATUS="passed"
else
  ROSDEP_INSTALL_STATUS="failed"
  FAILURE_REASON="rosdep_install_failed"
  emit_report
  exit 1
fi
if run_logged_step /artifacts/quick_gate_container.log \
    "${TMP_WS}/src/rokae_xmate3_ros2/tools/run_quick_gate.sh" "${TMP_WS}"; then
  QUICK_GATE_STATUS="passed"
else
  QUICK_GATE_STATUS="failed"
  FAILURE_REASON="quick_gate_failed"
  emit_report
  exit 1
fi
if [ "${RUN_RELEASE_GATE}" = "1" ]; then
  if run_logged_step /artifacts/release_gate_container.log \
      "${TMP_WS}/src/rokae_xmate3_ros2/tools/run_release_gate.sh" "${TMP_WS}"; then
    RELEASE_GATE_STATUS="passed"
  else
    RELEASE_GATE_STATUS="failed"
    FAILURE_REASON="release_gate_failed"
    emit_report
    exit 1
  fi
fi
if [ "${RUN_LAUNCH_SMOKE}" = "1" ]; then
  if run_logged_step /artifacts/launch_smoke_container.log \
      "${TMP_WS}/src/rokae_xmate3_ros2/tools/run_launch_smoke.sh" "${TMP_WS}"; then
    LAUNCH_SMOKE_STATUS="passed"
  else
    LAUNCH_SMOKE_STATUS="failed"
    FAILURE_REASON="launch_smoke_failed"
    emit_report
    exit 1
  fi
fi
ACCEPTANCE_STATUS="passed"
FAILURE_REASON=""
emit_report
EOSCRIPT
)

set +e
"$ENGINE" run --rm \
  -e RUN_RELEASE_GATE="${RUN_RELEASE_GATE}" \
  -e RUN_LAUNCH_SMOKE="${RUN_LAUNCH_SMOKE}" \
  -v "${PKG_ROOT}:/package:ro" \
  -v "${REPORT_DIR}:/artifacts" \
  "${IMAGE_TAG}" \
  bash -lc "${CONTAINER_SCRIPT}"
CONTAINER_STATUS=$?
set -e
if [ ${CONTAINER_STATUS} -ne 0 ] && [ ! -f "${REPORT_DIR}/${REPORT_NAME}" ]; then
  FAILURE_REASON="container_bundle_failed_before_report"
  emit_report_host "${PKG_ROOT}" "${PKG_ROOT}" "${REPORT_DIR}/${REPORT_NAME}"
fi
exit ${CONTAINER_STATUS}
