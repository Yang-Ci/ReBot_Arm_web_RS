#!/usr/bin/env bash

set -euo pipefail

REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REBOTARM_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REBOTARM_ROS_SETUP="/opt/ros/${REBOTARM_ROS_DISTRO}/setup.bash"
REBOTARM_WS="${REBOTARM_RS_ROOT}/rebotarm_ros2"
REBOTARM_VENV="${REBOTARM_WS}/.venv"
REBOTARM_SDK_DIR="${REBOTARM_WS}/third_party/reBotArm_control_py"
REBOTARM_MUJOCO_DIR="${REBOTARM_WS}/third_party/reBot-B601-RS-for-mujoco_sim"

if [[ ! -f "${REBOTARM_ROS_SETUP}" ]]; then
  echo "ROS 2 environment not found: ${REBOTARM_ROS_SETUP}" >&2
  exit 1
fi

REBOTARM_RESTORE_NOUNSET=false
if [[ $- == *u* ]]; then
  REBOTARM_RESTORE_NOUNSET=true
  set +u
fi
source "${REBOTARM_ROS_SETUP}"

if [[ ! -x "${REBOTARM_VENV}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${REBOTARM_VENV}"
fi
source "${REBOTARM_VENV}/bin/activate"

if [[ "${REBOTARM_RESTORE_NOUNSET}" == true ]]; then
  set -u
fi
unset REBOTARM_RESTORE_NOUNSET

if [[ ! -f "${REBOTARM_SDK_DIR}/reBotArm_control_py/__init__.py" ]]; then
  echo "Vendored reBotArm control SDK is missing: ${REBOTARM_SDK_DIR}" >&2
  exit 1
fi

if [[ ! -f "${REBOTARM_MUJOCO_DIR}/assets/00_arm_rs_asm_v3/scene.xml" ]]; then
  echo "Vendored RS MuJoCo source is missing: ${REBOTARM_MUJOCO_DIR}" >&2
  exit 1
fi

if find "${REBOTARM_WS}/third_party" -mindepth 2 -maxdepth 2 -name .git -print -quit | grep -q .; then
  echo "Nested Git metadata found under rebotarm_ros2/third_party." >&2
  echo "Vendor sources must be ordinary files owned by the main repository." >&2
  exit 1
fi

"${REBOTARM_VENV}/bin/python" -m pip install --upgrade pip
"${REBOTARM_VENV}/bin/python" -m pip install \
  -r "${REBOTARM_RS_ROOT}/requirements-rs-hardware.txt" \
  -r "${REBOTARM_RS_ROOT}/requirements-rs-mujoco.txt"

if rosdep db >/dev/null 2>&1; then
  rosdep install --from-paths "${REBOTARM_WS}/src" --ignore-src -r -y
else
  echo "rosdep is not initialized; skipping system dependency installation." >&2
  echo "Optional setup: sudo rosdep init && rosdep update" >&2
fi
cd "${REBOTARM_WS}"
colcon build --symlink-install

echo "RS ROS 2 workspace ready."
echo "Run: source ${REBOTARM_WS}/install/setup.bash"
