#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_DIR="${ROOT_DIR}/rebotarm_ros2"
WEB_DIR="${ROOT_DIR}/reBotArm_simulator-RS"
VENV_DIR="${WS_DIR}/.venv"
SDK_DIR="${WS_DIR}/third_party/reBotArm_control_py"
MUJOCO_SOURCE_DIR="${WS_DIR}/third_party/reBot-B601-RS-for-mujoco_sim"
SDK_REF="40ab6ce58fec3c58cb603efb3f30240d6f5849e4"
MUJOCO_REF="1249cb6efdf393ba636056fc41df30dc6ba389aa"
CHECK_ONLY=0
ASSUME_YES=0

INSTALLED=()
SKIPPED=()
MISMATCH=()
FAILED=()

usage() {
  cat <<'EOF'
Usage: ./setup.sh [--check] [--yes]

  --check  Inspect the environment only; make no changes.
  --yes    Non-interactive package installation (sudo may request a password).
  -h       Show this help.

The installer preserves existing configuration and third-party working trees.
It installs only missing system dependencies, then delegates the reproducible
SDK/model checkout, Python environment, rosdep, and colcon build to
scripts/setup_rs_workspace.sh.
EOF
}

while (($#)); do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    --yes) ASSUME_YES=1 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

log() { printf '\n[rebotarm-rs-setup] %s\n' "$*"; }
have() { command -v "$1" >/dev/null 2>&1; }
record_installed() { INSTALLED+=("$1"); }
record_skipped() { SKIPPED+=("$1"); }
record_mismatch() { MISMATCH+=("$1"); }
record_failed() { FAILED+=("$1"); }

run_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

print_group() {
  local title="$1"
  shift
  local -a values=("$@")
  printf '\n%s (%d)\n' "${title}" "${#values[@]}"
  if ((${#values[@]} == 0)); then
    printf '  - none\n'
  else
    printf '  - %s\n' "${values[@]}"
  fi
}

finish() {
  local status=$?
  if ((status != 0)) && ((${#FAILED[@]} == 0)); then
    record_failed "setup exited with status ${status}"
  fi
  print_group 'Installed/updated' "${INSTALLED[@]}"
  print_group 'Already usable; skipped' "${SKIPPED[@]}"
  print_group 'Version/platform mismatches' "${MISMATCH[@]}"
  print_group 'Failed or still missing' "${FAILED[@]}"
  if ((CHECK_ONLY)); then
    printf '\nCheck-only mode made no changes.\n'
  elif ((${#FAILED[@]} == 0)); then
    printf '\nRS setup complete. Next:\n'
    printf '  ./rebotarm doctor\n'
    printf '  ./rebotarm start rs_sim\n'
    printf '  ./rebotarm start web\n'
  else
    printf '\nSetup is incomplete. Fix the failed items and rerun ./setup.sh.\n'
  fi
}
trap finish EXIT

if [[ ! -d "${WS_DIR}/src" || ! -d "${WEB_DIR}/public" ]]; then
  record_failed "repository layout is incomplete under ${ROOT_DIR}"
  exit 1
fi

log 'Checking supported platform'
PY_SITE=''
if [[ -r /etc/os-release ]]; then
  # shellcheck source=/dev/null
  source /etc/os-release
  case "${ID:-}:${VERSION_ID:-}" in
    ubuntu:24.04)
      DETECTED_ROS_DISTRO=jazzy
      PY_SITE=python3.12
      record_skipped "Ubuntu ${VERSION_ID} supported"
      ;;
    ubuntu:22.04)
      DETECTED_ROS_DISTRO=humble
      PY_SITE=python3.10
      record_skipped "Ubuntu ${VERSION_ID} supported"
      ;;
    *)
      DETECTED_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
      PY_SITE="python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null || printf '3')"
      record_mismatch "expected Ubuntu 24.04 or 22.04; found ${PRETTY_NAME:-unknown}"
      ;;
  esac
else
  DETECTED_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
  PY_SITE=python3.12
  record_mismatch 'cannot identify operating system'
fi
ROS_DISTRO="${ROS_DISTRO:-${DETECTED_ROS_DISTRO}}"
export ROS_DISTRO

if [[ "${ROS_DISTRO}" != "${DETECTED_ROS_DISTRO}" ]]; then
  record_mismatch "ROS_DISTRO=${ROS_DISTRO}, platform default is ${DETECTED_ROS_DISTRO}"
fi
case "$(uname -m)" in
  x86_64|aarch64) record_skipped "architecture $(uname -m) supported" ;;
  *) record_mismatch "untested architecture $(uname -m)" ;;
esac

APT_PACKAGES=(
  git curl ca-certificates software-properties-common build-essential pkg-config
  python3 python3-venv python3-pip python3-pytest
  nodejs npm
  iproute2 can-utils util-linux
  libgl1 libegl1 libx11-6 libxrandr2 libxinerama1 libxcursor1 libxi6
  ros-dev-tools
  ros-${ROS_DISTRO}-desktop
  ros-${ROS_DISTRO}-rosbridge-suite
  ros-${ROS_DISTRO}-moveit
  ros-${ROS_DISTRO}-tf-transformations
)

apt_capability_available() {
  case "$1" in
    pkg-config) have pkg-config ;;
    nodejs) have node ;;
    npm) have npm ;;
    iproute2) have ip ;;
    can-utils) have candump ;;
    util-linux) have flock ;;
    ros-dev-tools) have colcon && have rosdep ;;
    ros-${ROS_DISTRO}-desktop) [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] ;;
    ros-${ROS_DISTRO}-rosbridge-suite) [[ -d "/opt/ros/${ROS_DISTRO}/share/rosbridge_server" ]] ;;
    ros-${ROS_DISTRO}-moveit) [[ -d "/opt/ros/${ROS_DISTRO}/share/moveit_ros_move_group" ]] ;;
    ros-${ROS_DISTRO}-tf-transformations)
      [[ -d "/opt/ros/${ROS_DISTRO}/lib/${PY_SITE}/site-packages/tf_transformations" ||
         -d "${VENV_DIR}/lib/${PY_SITE}/site-packages/tf_transformations" ]]
      ;;
    *) return 1 ;;
  esac
}

MISSING_APT=()
for package in "${APT_PACKAGES[@]}"; do
  if dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'install ok installed'; then
    version="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
    record_skipped "apt ${package} ${version}"
  elif apt_capability_available "${package}"; then
    record_skipped "${package} capability already available"
  else
    MISSING_APT+=("${package}")
  fi
done

if ((${#MISSING_APT[@]})); then
  if ((CHECK_ONLY)); then
    for package in "${MISSING_APT[@]}"; do record_failed "missing apt package ${package}"; done
  else
    log "Installing missing system packages: ${MISSING_APT[*]}"
    if ! apt-cache show "ros-${ROS_DISTRO}-desktop" >/dev/null 2>&1; then
      log 'Adding the official ROS 2 apt source'
      run_sudo apt-get update || record_mismatch 'apt update reported an error before ROS source setup'
      run_sudo apt-get install -y software-properties-common curl ca-certificates
      run_sudo add-apt-repository -y universe
      ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n1)"
      if [[ -z "${ros_apt_version}" ]]; then
        record_failed 'could not resolve the latest ros2-apt-source version'
        exit 1
      fi
      codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-noble}}"
      ros_apt_deb="$(mktemp /tmp/ros2-apt-source.XXXXXX.deb)"
      curl -fL -o "${ros_apt_deb}" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${codename}_all.deb"
      run_sudo dpkg -i "${ros_apt_deb}"
      record_installed "official ROS 2 apt source ${ros_apt_version}"
    fi
    run_sudo apt-get update || record_mismatch 'apt update reported an error; continuing with available indexes'
    if run_sudo apt-get install -y "${MISSING_APT[@]}"; then
      for package in "${MISSING_APT[@]}"; do record_installed "apt ${package}"; done
    else
      for package in "${MISSING_APT[@]}"; do
        if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'install ok installed' &&
           ! apt_capability_available "${package}"; then
          record_failed "apt ${package}"
        fi
      done
    fi
  fi
fi

log 'Checking runtime versions'
if have python3; then
  py_version="$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')"
  if python3 -c 'import sys; raise SystemExit(0 if sys.version_info[:2] in ((3, 12), (3, 10)) else 1)'; then
    record_skipped "Python ${py_version} compatible"
  else
    record_mismatch "expected Python 3.12 or 3.10; found ${py_version}"
  fi
else
  record_failed 'python3 command missing'
fi

if have node; then
  node_version="$(node -p 'process.versions.node')"
  node_major="${node_version%%.*}"
  if ((node_major >= 18)); then
    record_skipped "Node.js ${node_version} compatible"
  else
    record_mismatch "Node.js >=18 required; found ${node_version}"
  fi
else
  record_failed 'node command missing'
fi

log 'Checking pinned RS third-party sources'
check_git_revision() {
  local label="$1" path="$2" expected="$3"
  if [[ ! -d "${path}/.git" ]]; then
    if ((CHECK_ONLY)); then record_failed "${label} missing: ${path}"; fi
    return
  fi
  local actual
  actual="$(git -C "${path}" rev-parse HEAD 2>/dev/null || true)"
  if [[ "${actual}" == "${expected}" ]]; then
    record_skipped "${label} revision ${actual} validated"
  else
    record_mismatch "${label} revision ${actual:-unknown}; validated ${expected}; existing tree preserved"
  fi
}
check_git_revision 'RS control SDK' "${SDK_DIR}" "${SDK_REF}"
check_git_revision 'RS MuJoCo source' "${MUJOCO_SOURCE_DIR}" "${MUJOCO_REF}"

log 'Checking rosdep database'
if have rosdep && rosdep db >/dev/null 2>&1; then
  record_skipped 'rosdep database initialized'
elif ((CHECK_ONLY)); then
  record_mismatch 'rosdep database is not initialized; runtime can work, ./setup.sh --yes will initialize it'
else
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run_sudo rosdep init
    record_installed 'rosdep system sources initialized'
  fi
  rosdep update
  record_installed 'rosdep user database updated'
fi

if ((CHECK_ONLY)); then
  log 'Checking generated environment and build'
  if [[ -x "${VENV_DIR}/bin/python" ]]; then
    record_skipped "virtual environment ${VENV_DIR}"
    PYTHON_IMPORTS=(numpy scipy mujoco pinocchio motorbridge fastmcp)
    for module in "${PYTHON_IMPORTS[@]}"; do
      if "${VENV_DIR}/bin/python" -c "import ${module}" >/dev/null 2>&1; then
        record_skipped "Python import ${module}"
      else
        record_failed "Python import ${module} failed in project venv"
      fi
    done
  else
    record_failed "missing virtual environment ${VENV_DIR}"
  fi
  if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    record_skipped 'ROS workspace build/install exists'
  else
    record_failed "missing ${WS_DIR}/install/setup.bash"
  fi
else
  log 'Creating the reproducible Python/SDK/model workspace and building ROS packages'
  if ROS_DISTRO="${ROS_DISTRO}" "${ROOT_DIR}/scripts/setup_rs_workspace.sh"; then
    record_installed 'RS workspace dependencies and colcon build checked/updated'
  else
    record_failed 'scripts/setup_rs_workspace.sh failed'
    exit 1
  fi
fi

log 'Checking web application configuration'
if [[ -f "${WEB_DIR}/.env" ]]; then
  record_skipped 'existing web .env preserved'
elif ((CHECK_ONLY)); then
  record_failed "missing ${WEB_DIR}/.env (copy from .env.example)"
else
  cp "${WEB_DIR}/.env.example" "${WEB_DIR}/.env"
  record_installed 'web .env created from .env.example'
fi

if [[ -f "${WEB_DIR}/package.json" ]] && have node; then
  if node -e 'JSON.parse(require("fs").readFileSync(process.argv[1], "utf8"))' "${WEB_DIR}/package.json"; then
    record_skipped 'web package.json is valid'
  else
    record_failed 'web package.json is invalid'
  fi
  if ((!CHECK_ONLY)); then
    if (cd "${WEB_DIR}" && npm install --ignore-scripts --no-audit --no-fund); then
      record_installed 'web npm dependencies checked/updated'
    else
      record_failed 'web npm install failed'
    fi
  fi
fi

if [[ -e /sys/class/net/can0 ]]; then
  can_state="$(cat /sys/class/net/can0/operstate 2>/dev/null || printf unknown)"
  record_skipped "SocketCAN can0 exists (state=${can_state})"
else
  record_mismatch 'SocketCAN can0 is absent; simulation works, hardware requires configuring can0'
fi

if ((${#FAILED[@]})); then
  exit 1
fi
