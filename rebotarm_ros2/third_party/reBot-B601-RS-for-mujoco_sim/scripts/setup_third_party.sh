#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SDK_DIR="${REPO_ROOT}/../reBotArm_control_py"

if [[ ! -f "${SDK_DIR}/reBotArm_control_py/__init__.py" ]]; then
  echo "Vendored SDK missing at ${SDK_DIR}." >&2
  echo "Restore rebotarm_ros2/third_party from the parent repository." >&2
  exit 1
fi

if [[ -e "${SDK_DIR}/.git" ]]; then
  echo "Nested Git metadata is not allowed at ${SDK_DIR}/.git" >&2
  exit 1
fi

echo "Vendored SDK ready: ${SDK_DIR}"
