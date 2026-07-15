#!/usr/bin/env bash

# Strict mode only when executed directly; when sourced, the caller's shell
# options must be left untouched.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  MAPEVERYTHING_MSGS_SOURCED=0
  set -euo pipefail
else
  MAPEVERYTHING_MSGS_SOURCED=1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${REPO_ROOT}/ros2"
MSG_PACKAGE="${ROS_WS}/mapeverything_msgs"
INSTALL_SETUP="${ROS_WS}/install/setup.bash"

source_setup() {
  local setup_file="$1"
  local had_nounset=0
  if [[ "$-" == *u* ]]; then
    had_nounset=1
  fi
  set +u
  # ROS setup scripts may reference unset environment variables.
  source "${setup_file}"
  local setup_status=$?
  if [[ "${had_nounset}" -eq 1 ]]; then
    set -u
  fi
  return "${setup_status}"
}

if [[ ! -d "${MSG_PACKAGE}" ]]; then
  echo "error: missing message package: ${MSG_PACKAGE}" >&2
  return 2 2>/dev/null || exit 2
fi

if [[ -n "${ROS_SETUP:-}" ]]; then
  source_setup "${ROS_SETUP}" || {
    echo "error: failed to source ${ROS_SETUP}" >&2
    return 2 2>/dev/null || exit 2
  }
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash" || {
    echo "error: failed to source /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    return 2 2>/dev/null || exit 2
  }
else
  for distro in rolling kilted jazzy humble iron; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      source_setup "/opt/ros/${distro}/setup.bash" || {
        echo "error: failed to source /opt/ros/${distro}/setup.bash" >&2
        return 2 2>/dev/null || exit 2
      }
      break
    fi
  done
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon is not available. Source ROS 2 first or install python3-colcon-common-extensions." >&2
  return 2 2>/dev/null || exit 2
fi

colcon --log-base "${ROS_WS}/log" build \
  --base-paths "${MSG_PACKAGE}" \
  --build-base "${ROS_WS}/build" \
  --install-base "${ROS_WS}/install" \
  --packages-select mapeverything_msgs \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 || {
  echo "error: colcon build failed for mapeverything_msgs" >&2
  return 2 2>/dev/null || exit 2
}

source_setup "${INSTALL_SETUP}" || {
  echo "error: failed to source ${INSTALL_SETUP}" >&2
  return 2 2>/dev/null || exit 2
}

if [[ "${MAPEVERYTHING_MSGS_SOURCED}" -eq 0 ]]; then
  if [[ "$#" -gt 0 ]]; then
    exec "$@"
  fi
  echo "Built and sourced mapeverything_msgs for this process."
  echo "To update your current shell, run: source ${BASH_SOURCE[0]}"
else
  echo "Built and sourced mapeverything_msgs."
fi
