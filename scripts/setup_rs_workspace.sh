#!/usr/bin/env bash

set -euo pipefail

REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REBOTARM_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
REBOTARM_ROS_SETUP="/opt/ros/${REBOTARM_ROS_DISTRO}/setup.bash"
REBOTARM_WS="${REBOTARM_RS_ROOT}/rebotarm_ros2"
REBOTARM_VENV="${REBOTARM_WS}/.venv"
REBOTARM_SDK_DIR="${REBOTARM_WS}/third_party/reBotArm_control_py"
REBOTARM_MUJOCO_DIR="${REBOTARM_WS}/third_party/reBot-B601-RS-for-mujoco_sim"
REBOTARM_SDK_REF="40ab6ce58fec3c58cb603efb3f30240d6f5849e4"
REBOTARM_MUJOCO_REF="1249cb6efdf393ba636056fc41df30dc6ba389aa"
REBOTARM_SDK_PATCH="${REBOTARM_RS_ROOT}/patches/rebotarm_control_py_rs.patch"
REBOTARM_MUJOCO_OVERRIDE="${REBOTARM_RS_ROOT}/vendor_overrides/reBot-B601-RS-for-mujoco_sim/assets/00_arm_rs_asm_v3"

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

if [[ ! -d "${REBOTARM_SDK_DIR}/reBotArm_control_py" ]]; then
  mkdir -p "${REBOTARM_WS}/third_party"
  git clone \
    https://github.com/vectorBH6/reBotArm_control_py.git \
    "${REBOTARM_SDK_DIR}"
  git -C "${REBOTARM_SDK_DIR}" checkout --detach "${REBOTARM_SDK_REF}"
fi

if [[ "$(git -C "${REBOTARM_SDK_DIR}" rev-parse HEAD)" != "${REBOTARM_SDK_REF}" ]]; then
  echo "Warning: reBotArm SDK is not at the validated revision ${REBOTARM_SDK_REF}." >&2
fi

if [[ ! -f "${REBOTARM_MUJOCO_DIR}/assets/00_arm_rs_asm_v3/scene.xml" ]]; then
  git clone \
    https://github.com/LAN-GER/reBot-B601-RS-for-mujoco_sim.git \
    "${REBOTARM_MUJOCO_DIR}"
  git -C "${REBOTARM_MUJOCO_DIR}" checkout --detach "${REBOTARM_MUJOCO_REF}"
fi

if git -C "${REBOTARM_SDK_DIR}" apply --check "${REBOTARM_SDK_PATCH}"; then
  git -C "${REBOTARM_SDK_DIR}" apply "${REBOTARM_SDK_PATCH}"
elif ! git -C "${REBOTARM_SDK_DIR}" apply --reverse --check "${REBOTARM_SDK_PATCH}"; then
  echo "RS SDK patch cannot be applied cleanly: ${REBOTARM_SDK_PATCH}" >&2
  exit 1
fi

if [[ "$(git -C "${REBOTARM_MUJOCO_DIR}" rev-parse HEAD)" != "${REBOTARM_MUJOCO_REF}" ]]; then
  echo "Warning: RS MuJoCo source is not at the validated revision ${REBOTARM_MUJOCO_REF}." >&2
fi

cp "${REBOTARM_MUJOCO_OVERRIDE}/00_arm_rs_asm_v3.xml" \
  "${REBOTARM_MUJOCO_DIR}/assets/00_arm_rs_asm_v3/00_arm_rs_asm_v3.xml"
cp "${REBOTARM_MUJOCO_OVERRIDE}/meshes/"*.STL \
  "${REBOTARM_MUJOCO_DIR}/assets/00_arm_rs_asm_v3/meshes/"

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
