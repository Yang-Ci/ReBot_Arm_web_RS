#!/usr/bin/env bash

set -eo pipefail

REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REBOTARM_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REBOTARM_ROS_SETUP="/opt/ros/${REBOTARM_ROS_DISTRO}/setup.bash"
REBOTARM_WS_SETUP="${REBOTARM_RS_ROOT}/rebotarm_ros2/install/setup.bash"
REBOTARM_VENV_ACTIVATE="${REBOTARM_RS_ROOT}/rebotarm_ros2/.venv/bin/activate"

if [[ ! -f "${REBOTARM_ROS_SETUP}" ]]; then
  echo "ROS 2 environment not found: ${REBOTARM_ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${REBOTARM_WS_SETUP}" ]]; then
  echo "RS workspace has not been built: ${REBOTARM_WS_SETUP}" >&2
  echo "Run scripts/setup_rs_workspace.sh first." >&2
  exit 1
fi

if [[ ! -f "${REBOTARM_VENV_ACTIVATE}" ]]; then
  echo "RS Python environment has not been created: ${REBOTARM_VENV_ACTIVATE}" >&2
  echo "Run scripts/setup_rs_workspace.sh first." >&2
  exit 1
fi

REBOTARM_RESTORE_NOUNSET=false
if [[ $- == *u* ]]; then
  REBOTARM_RESTORE_NOUNSET=true
  set +u
fi

source "${REBOTARM_ROS_SETUP}"
source "${REBOTARM_VENV_ACTIVATE}"
REBOTARM_VENV_SITE_PACKAGES="$(python3 -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
REBOTARM_PYTHON_ABI="$(python3 -c 'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"
REBOTARM_CMEEL_SITE_PACKAGES="${REBOTARM_VENV_SITE_PACKAGES}/cmeel.prefix/lib/${REBOTARM_PYTHON_ABI}/site-packages"
export PYTHONPATH="${REBOTARM_CMEEL_SITE_PACKAGES}:${REBOTARM_VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
source "${REBOTARM_WS_SETUP}"

if [[ "${REBOTARM_RESTORE_NOUNSET}" == true ]]; then
  set -u
fi
unset REBOTARM_RESTORE_NOUNSET
unset REBOTARM_CMEEL_SITE_PACKAGES
unset REBOTARM_PYTHON_ABI
unset REBOTARM_VENV_SITE_PACKAGES
