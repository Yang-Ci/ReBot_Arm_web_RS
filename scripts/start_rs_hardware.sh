#!/usr/bin/env bash

set -euo pipefail

if [[ "${REBOTARM_RS_HARDWARE_CONFIRM:-}" != "I_UNDERSTAND_RS_WILL_MOVE" ]]; then
  echo "Hardware launch blocked." >&2
  echo "Confirm the work area is clear and a physical emergency stop is available." >&2
  echo "Then set REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE." >&2
  exit 2
fi

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"

if ! ip link show can0 >/dev/null 2>&1; then
  echo "SocketCAN interface can0 does not exist." >&2
  exit 1
fi

ip -details link show can0
exec ros2 launch rebotarm_bringup bringup.launch.py \
  model:=rs \
  channel:=can0 \
  arm_namespace:=rebotarm \
  ee_frame_id:=gripper_end \
  use_rviz:="${REBOTARM_USE_RVIZ:-false}"
