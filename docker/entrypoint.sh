#!/usr/bin/env bash

set -euo pipefail

USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}
USERNAME=${USERNAME:-ros}
PROJECT_NAME=${PROJECT_NAME:-ros2x}
ROS_DISTRO=${ROS_DISTRO:-humble}
ROS2X_ROOT=${ROS2X_ROOT:-/home/${USERNAME}/ROS2X}
ROS2X_WS=${ROS2X_WS:-/home/${USERNAME}/ROS2X/ros2_ws}
AUTO_BUILD=${AUTO_BUILD:-false}
AUTO_RUN=${AUTO_RUN:-false}
INSTALL_MICRO_ROS=${INSTALL_MICRO_ROS:-false}
LAUNCH_COMMAND=${LAUNCH_COMMAND:-}

log() {
    echo "[ROS2X entrypoint] $*"
}

source_ros_underlay() {
    local underlay="/opt/ros/${ROS_DISTRO}/setup.bash"
    if [[ ! -f "${underlay}" ]]; then
        log "ROS underlay not found: ${underlay}"
        return 1
    fi
    set +u
    # shellcheck disable=SC1090
    source "${underlay}"
    set -u
}

source_ros_overlay_if_exists() {
    local overlay="${ROS2X_WS}/install/setup.bash"
    if [[ -f "${overlay}" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "${overlay}"
        set -u
    fi
}

ensure_workspace_dirs() {
    mkdir -p "${ROS2X_WS}/src"
}

bootstrap_micro_ros_if_enabled() {
    if [[ "${INSTALL_MICRO_ROS}" != "true" ]]; then
        return
    fi

    local installer="${ROS2X_ROOT}/scripts/install_micro_ros.sh"
    if [[ ! -f "${installer}" ]]; then
        log "micro-ROS installer not found: ${installer}"
        return 1
    fi

    log "Bootstrapping micro-ROS via ${installer}..."
    ROS_DISTRO="${ROS_DISTRO}" WS_DIR="${ROS2X_WS}" bash "${installer}"
}

build_workspace() {
    log "Running workspace build..."
    source_ros_underlay
    source_ros_overlay_if_exists
    cd "${ROS2X_WS}"
    if [[ "${INSTALL_MICRO_ROS}" == "true" ]]; then
        bootstrap_micro_ros_if_enabled
        # Bootstrap may update workspace content and overlay state.
        source_ros_overlay_if_exists
    fi
    colcon build --symlink-install
}

run_launch_command() {
    if [[ -z "${LAUNCH_COMMAND}" ]]; then
        log "LAUNCH_COMMAND is empty. Nothing to run."
        return 1
    fi

    source_ros_underlay
    source_ros_overlay_if_exists

    log "Launching command: ${LAUNCH_COMMAND}"
    bash -lc "${LAUNCH_COMMAND}"
}

if [[ "$(id -u)" -eq 0 ]]; then
    if [[ "$(id -u ${USERNAME})" != "${USER_ID}" || "$(id -g ${USERNAME})" != "${GROUP_ID}" ]]; then
        log "Updating ${USERNAME} UID/GID to ${USER_ID}:${GROUP_ID}"
        groupmod -g "${GROUP_ID}" "${USERNAME}"
        usermod -u "${USER_ID}" "${USERNAME}"
        chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}"
    fi
fi

mkdir -p "${ROS2X_WS}"
cd "${ROS2X_WS}"
ensure_workspace_dirs

if [[ "$(id -u)" -eq 0 ]]; then
    log "Switching to user ${USERNAME}"
    exec gosu "${USERNAME}" "$0" "$@"
fi

if [[ "${AUTO_BUILD}" == "true" ]]; then
    build_workspace
fi

if [[ "${AUTO_RUN}" == "true" ]]; then
    bootstrap_micro_ros_if_enabled
    if [[ ! -f "${ROS2X_WS}/install/setup.bash" ]]; then
        log "Workspace not built yet. Building before run..."
        build_workspace
    fi
    run_launch_command
fi

# For one-shot actions (build/run), exit after completing action when command is default shell.
if [[ "${AUTO_BUILD}" == "true" || "${AUTO_RUN}" == "true" ]]; then
    if [[ $# -eq 0 || ( $# -eq 1 && "$1" == "/bin/bash" ) ]]; then
        log "Action finished. Exiting container."
        exit 0
    fi
fi

# Keep the container alive for interactive workflows.
if [[ $# -eq 0 || ( $# -eq 1 && "$1" == "/bin/bash" ) ]]; then
    log "No explicit command provided. Keeping container alive for enter/exec workflows..."
    while true; do sleep 60; done
fi

log "Executing command as ${USERNAME}: $*"
exec "$@"
