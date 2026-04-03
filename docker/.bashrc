# ==================== ROS ==================== #
# ==== Description: ROS environment setup ===== #
# =================== BEGIN =================== #

source_ros_environment() {
    alias rosdep-check='rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO:-humble} -y'
    alias build='colcon build --symlink-install'
    local ws="${ROS2X_WS:-}"

    if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    fi

    if [ -n "${ws}" ] && [ -f "${ws}/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "${ws}/install/setup.bash"
    fi

    if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
        # shellcheck disable=SC1091
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    fi

    if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
        # shellcheck disable=SC1091
        source /usr/share/colcon_cd/function/colcon_cd.sh
    fi

    if [ -n "${ws}" ]; then
        export _colcon_cd_root="${ws}"
    fi
}

groot() {
    if [ -f "${GROOT_APPIMAGE:-}" ]; then
        alias groot='${GROOT_APPIMAGE}'
    fi
}

# ==================== END ==================== #

source_ros_environment
groot
