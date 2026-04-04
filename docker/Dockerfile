ARG ROS_DISTRO=humble
ARG ROS_INSTALL_TYPE=ros-base
FROM ubuntu:22.04 AS base

LABEL org.opencontainers.image.title="ROS2X - ROS Toolbox"
LABEL org.opencontainers.image.description="ROS2X is a toolbox for ROS2 development"
LABEL org.opencontainers.image.authors="scx@gapp.nthu.edu.tw"
LABEL org.opencontainers.image.licenses="MIT"

ARG ROS_DISTRO
ARG ROS_INSTALL_TYPE
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei
ENV SHELL=/bin/bash
ENV TERM=xterm-256color
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS2X_ROOT=/home/${USERNAME}/ROS2X
ENV ROS2X_WS=/home/${USERNAME}/ROS2X/ros2_ws

# Improve apt resilience for flaky mirrors/networks.
RUN printf 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\n' > /etc/apt/apt.conf.d/80-ros2x-retries

# Base tools required before ROS installation.
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    gosu \
    curl \
    wget \
    git \
    unzip \
    usbutils \
    build-essential \
    cmake \
    python3-pip \
    bash-completion \
    software-properties-common \
    ca-certificates \
    gnupg2 \
    lsb-release \
    locales \
    tzdata \
    tmux \
    htop \
    tree \
    vim \
    libfuse2 \
    fuse3 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libqt5gui5 \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user (UID/GID will still be remapped by entrypoint at runtime).
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -a -G dialout ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Remove modemmanager to avoid serial-port conflicts for QGroundControl.
RUN apt-get update \
    && (apt-get remove -y modemmanager || true) \
    && rm -rf /var/lib/apt/lists/*

# Install ROS using the repo script (works in Docker build and native host use cases).
COPY scripts/install_ros2.sh /tmp/install_ros2.sh
RUN chmod +x /tmp/install_ros2.sh \
    && ROS_DISTRO=${ROS_DISTRO} INSTALL_TYPE=${ROS_INSTALL_TYPE} NON_INTERACTIVE=true CONFIGURE_BASHRC=no ALLOW_ROOT=true SKIP_UPGRADE=true NO_INSTALL_RECOMMENDS=true \
       /tmp/install_ros2.sh \
       --allow-root \
       --non-interactive \
       --configure-bashrc no \
       --skip-upgrade \
       --no-install-recommends \
       --ros-distro ${ROS_DISTRO} \
       --install-type ${ROS_INSTALL_TYPE} \
    && rm -f /tmp/install_ros2.sh

# Install ROS-related optional packages when available for the selected distro.
RUN apt-get update \
    && for pkg in \
        "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
        "ros-${ROS_DISTRO}-foxglove-bridge" \
        "ros-${ROS_DISTRO}-rosbridge-server" \
        "ros-${ROS_DISTRO}-ament-cmake" \
        "ros-${ROS_DISTRO}-rviz2" \
        "ros-${ROS_DISTRO}-ros-gz" \
        "ros-${ROS_DISTRO}-ros-gz-sim" \
        "ros-${ROS_DISTRO}-ros-gz-bridge" \
        "ros-${ROS_DISTRO}-rqt-plot" \
        "ros-${ROS_DISTRO}-rqt-tf-tree" \
        "ros-${ROS_DISTRO}-nav2-msgs" \
        "ros-${ROS_DISTRO}-nav2-util" \
        "ros-${ROS_DISTRO}-behaviortree-cpp" \
        "ros-${ROS_DISTRO}-generate-parameter-library"; do \
          if apt-cache show "$pkg" >/dev/null 2>&1; then \
              apt-get install -y --no-install-recommends "$pkg"; \
          else \
              echo "[ROS2X] Optional package not found for ${ROS_DISTRO}: $pkg"; \
          fi; \
       done \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

USER root
WORKDIR /home/${USERNAME}

# Container shell customizations.
COPY docker/.bashrc /tmp/ros2x.bashrc
RUN cat /tmp/ros2x.bashrc >> /home/${USERNAME}/.bashrc \
    && rm -f /tmp/ros2x.bashrc \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

# Runtime bootstrap.
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
