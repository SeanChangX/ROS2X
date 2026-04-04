#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./install_micro_ros.sh [options]

Options:
  --skip-rosdep      Skip apt/rosdep dependency installation.
  --skip-agent-ws    Skip create_agent_ws.sh step.
  --skip-agent-build Skip build_agent.sh step.
  --force            Force reinstall/rebuild even if already bootstrapped.
  -h, --help         Show this help message.
EOF
}

log() {
  printf '[micro-ROS][%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

ensure_rosdep_available() {
  if command -v rosdep >/dev/null 2>&1; then
    return
  fi

  log "rosdep not found. Installing python3-rosdep ..."
  sudo apt-get update
  sudo apt-get install -y --no-install-recommends python3-rosdep
}

retry() {
  local attempts="$1"
  local delay_s="$2"
  shift 2

  local attempt=1
  while true; do
    if "$@"; then
      return 0
    fi

    if ((attempt >= attempts)); then
      return 1
    fi

    log "Command failed (attempt ${attempt}/${attempts}), retrying in ${delay_s}s: $*"
    sleep "${delay_s}"
    attempt=$((attempt + 1))
  done
}

SKIP_ROSDEP=0
SKIP_AGENT_WS=0
SKIP_AGENT_BUILD=0
FORCE_REBUILD=0

while (($# > 0)); do
  case "$1" in
    --skip-rosdep)
      SKIP_ROSDEP=1
      ;;
    --skip-agent-ws)
      SKIP_AGENT_WS=1
      ;;
    --skip-agent-build)
      SKIP_AGENT_BUILD=1
      ;;
    --force)
      FORCE_REBUILD=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

ROS_DISTRO="${ROS_DISTRO:-humble}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${WS_DIR:-${ROS2X_WS:-${REPO_ROOT}/ros2_ws}}"
SRC_DIR="${WS_DIR}/src"
MICRO_ROS_SETUP_DIR="${SRC_DIR}/micro_ros_setup"
AGENT_EXEC="${WS_DIR}/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
BOOTSTRAP_MARKER="${WS_DIR}/.micro_ros_bootstrapped_${ROS_DISTRO}"
BOOTSTRAP_LOCK="${WS_DIR}/.micro_ros_bootstrap.lock"

if command -v flock >/dev/null 2>&1; then
  exec 9>"${BOOTSTRAP_LOCK}"
  if ! flock -n 9; then
    log "Another bootstrap is running, waiting for lock ..."
    flock 9
  fi
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS environment not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if ((FORCE_REBUILD == 0)) && [[ -f "${BOOTSTRAP_MARKER}" ]] && [[ -x "${AGENT_EXEC}" ]]; then
  log "Bootstrap marker found and micro_ros_agent is ready. Nothing to do."
  exit 0
fi

mkdir -p "${SRC_DIR}"
cd "${WS_DIR}"
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

if [[ -d "${MICRO_ROS_SETUP_DIR}/.git" ]]; then
  log "Updating micro_ros_setup (${ROS_DISTRO}) ..."
  retry 3 2 git -C "${MICRO_ROS_SETUP_DIR}" fetch origin "${ROS_DISTRO}" --depth 1
  git -C "${MICRO_ROS_SETUP_DIR}" checkout "${ROS_DISTRO}"
  retry 3 2 git -C "${MICRO_ROS_SETUP_DIR}" pull --ff-only origin "${ROS_DISTRO}"
else
  log "Cloning micro_ros_setup (${ROS_DISTRO}) ..."
  retry 3 2 git clone --depth 1 -b "${ROS_DISTRO}" \
    https://github.com/micro-ROS/micro_ros_setup.git \
    "${MICRO_ROS_SETUP_DIR}"
fi

if ((SKIP_ROSDEP == 0)); then
  log "Installing dependencies with rosdep ..."
  ensure_rosdep_available
  sudo apt-get update
  if ! rosdep db >/dev/null 2>&1; then
    sudo rosdep init || true
  fi
  if ! retry 3 3 rosdep update --rosdistro "${ROS_DISTRO}"; then
    log "rosdep update failed after retries, checking local cache fallback ..."
    if ! rosdep db >/dev/null 2>&1; then
      echo "rosdep update failed and no usable local cache is available." >&2
      exit 1
    fi
    log "Using existing rosdep cache."
  fi
  rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -y
else
  log "Skipping rosdep dependency installation."
fi

log "Building workspace ..."
colcon build --symlink-install --cmake-clean-cache --event-handlers console_direct+
set +u
# shellcheck disable=SC1091
source "${WS_DIR}/install/local_setup.bash"
set -u

if ((SKIP_AGENT_WS == 0)); then
  if [[ -d "${SRC_DIR}/uros/micro-ROS-Agent" ]]; then
    log "Agent workspace sources already exist; skipping create_agent_ws.sh"
  else
    log "Creating micro-ROS agent workspace ..."
    ros2 run micro_ros_setup create_agent_ws.sh
  fi
else
  log "Skipping create_agent_ws.sh."
fi

if ((SKIP_AGENT_BUILD == 0)); then
  if ((FORCE_REBUILD == 0)) && [[ -x "${AGENT_EXEC}" ]]; then
    log "micro_ros_agent binary already exists; skipping build_agent.sh"
  else
    log "Building micro-ROS agent workspace ..."
    # Keep build mode aligned with the workspace build to avoid symlink collisions
    # when entrypoint later runs `colcon build --symlink-install`.
    ros2 run micro_ros_setup build_agent.sh --symlink-install
  fi
else
  log "Skipping build_agent.sh."
fi

set +u
# shellcheck disable=SC1091
source "${WS_DIR}/install/local_setup.bash"
set -u

if [[ ! -x "${AGENT_EXEC}" ]]; then
  echo "micro_ros_agent binary not found after bootstrap: ${AGENT_EXEC}" >&2
  exit 1
fi

printf 'ROS_DISTRO=%s\nUPDATED_AT=%s\n' "${ROS_DISTRO}" "$(date -Iseconds)" > "${BOOTSTRAP_MARKER}"
log "micro-ROS workspace is ready."
