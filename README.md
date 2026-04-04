# ROS2X

[English](README.md) | [繁體中文](README.zh-TW.md)

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20(default)-22314E.svg)](https://docs.ros.org/en/humble/index.html)
[![Docker](https://img.shields.io/badge/Docker-Compose-2496ED.svg)](https://docs.docker.com/compose/)
[![Arch](https://img.shields.io/badge/Arch-x86__64%20%7C%20aarch64-6f42c1.svg)](#compatibility)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25.svg)](ROS2X)

ROS2X is a Docker-first ROS 2 development toolbox focused on reproducible,
robotics-friendly workflows. It provides one entrypoint script for image build,
workspace build, launch/run, micro-ROS bootstrap, and built-in app launchers.

<table>
  <tr>
    <td align="center" width="220">
      <a href="https://foxglove.dev">
        <img src="docs/assets/foxglove-logo.svg" alt="Foxglove" height="50" />
      </a>
    </td>
    <td align="center" width="220">
      <a href="https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html">
        <img src="docs/assets/rviz-logo.png" alt="RViz2" height="50" />
      </a>
    </td>
    <td align="center" width="220">
      <a href="https://gazebosim.org/docs/fortress/">
        <img src="docs/assets/gazebo-logo.png" alt="Gazebo Fortress" height="50" />
      </a>
    </td>
    <td align="center" width="220">
      <a href="https://www.behaviortree.dev/groot/">
        <img src="docs/assets/groot2-logo.png" alt="Groot2" height="50" />
      </a>
    </td>
    <td align="center" width="220">
      <a href="https://qgroundcontrol.com/">
        <img src="docs/assets/qgroundcontrol-logo.svg" alt="QGroundControl" height="50" />
      </a>
    </td>
  </tr>
  <tr>
    <td align="center"><code>./ROS2X bridge</code></td>
    <td align="center"><code>./ROS2X rviz</code></td>
    <td align="center"><code>./ROS2X gazebo</code></td>
    <td align="center"><code>./ROS2X app groot</code></td>
    <td align="center"><code>./ROS2X app qgc</code></td>
  </tr>
</table>

## Highlights

- Ubuntu 22.04 base image (`docker/Dockerfile`)
- Host UID/GID mapping for correct file ownership
- X11 GUI support for RViz and AppImage tools
- Integrated micro-ROS bootstrap flow (`scripts/install_micro_ros.sh`)
- Built-in app launchers for Foxglove Bridge, RViz2, Gazebo, Groot2, and QGroundControl
- Persistent project config via `config/ros2x.conf`

## Quick Start

```bash
./ROS2X --help
./ROS2X config init
./ROS2X build
./ROS2X enter
```

## Command Reference

| Command | Purpose |
|---|---|
| `./ROS2X build` | Build workspace in container (`colcon build --symlink-install`) |
| `./ROS2X run` | Run `LAUNCH_COMMAND` (auto-build if workspace not ready) |
| `./ROS2X bridge` | Run Foxglove Bridge |
| `./ROS2X rviz` | Launch RViz2 |
| `./ROS2X gazebo` | Launch Gazebo (`ros2 launch ros_gz_sim gz_sim.launch.py`) |
| `./ROS2X app groot` | Launch Groot2 AppImage |
| `./ROS2X app qgroundcontrol` | Launch QGroundControl AppImage |
| `./ROS2X enter` | Enter running container shell |
| `./ROS2X --command "<cmd>"` | Execute one command in container |
| `./ROS2X image-build` | Build/update Docker image only |
| `./ROS2X close` | Stop container |
| `./ROS2X config ...` | Show or manage persistent config |

## Configuration

`./ROS2X` writes `docker/.env` on every invocation.
Do not edit `docker/.env` directly.

Recommended ways to configure values:

1. One-off command:
   `ROS_DISTRO=humble ROS_INSTALL_TYPE=desktop ./ROS2X build`
2. Persistent project defaults:
   `./ROS2X config set <KEY> <VALUE>`

Precedence order:
`inline env vars > config/ros2x.conf > script defaults`

Useful config commands:

```bash
./ROS2X config init
./ROS2X config list
./ROS2X config keys
./ROS2X config get ROS_DISTRO
./ROS2X config set INSTALL_MICRO_ROS true
./ROS2X config unset GROOT_SHA256
```

Main keys:

- `ROS_DISTRO` (default: `humble`)
- `ROS_INSTALL_TYPE` (`ros-base|desktop|development`, default: `ros-base`)
- `ROS_DOMAIN_ID` (default: `0`)
- `PROJECT_NAME` (default: `ros2x`)
- `INSTALL_MICRO_ROS` (default: `false`)
- `LAUNCH_COMMAND`
- `IMAGE_NAME` (default: `ghcr.io/seanchangx/ros2x:${ROS_DISTRO}-<flavor>`)
  - flavor mapping: `ros-base -> base`, `desktop -> desktop`, `development -> dev`
- `IMAGE_STRATEGY` (`auto|pull|build`, default: `auto`)
- `GROOT_VERSION`, `GROOT_FALLBACK_VERSIONS`, `GROOT_APPIMAGE_URL`, `GROOT_SHA256`
- `QGC_VERSION` (`latest` or e.g. `5.0.8`), `QGC_APPIMAGE_URL`, `QGC_SHA256`
- `FOXGLOVE_PORT` (default: `8765`), `FOXGLOVE_ADDRESS` (default: `0.0.0.0`)

## Utility Tooling Example

Run each utility independently with `--command`:

```bash
./ROS2X --command "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
```

```bash
./ROS2X --command "ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox joy_vel:=cmd_vel"
```

```bash
./ROS2X --command "ros2 run image_tools cam2image --ros-args --log-level WARN -p video_device:=/dev/video0"
```

## micro-ROS Workflow

Enable micro-ROS bootstrap and build:

```bash
./ROS2X config set INSTALL_MICRO_ROS true
./ROS2X build
```

Run agent example:

```bash
./ROS2X --command "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200 -v6"
```

## Foxglove Bridge

```bash
./ROS2X bridge
```

Custom bind:

```bash
./ROS2X config set FOXGLOVE_ADDRESS 0.0.0.0
./ROS2X config set FOXGLOVE_PORT 8765
./ROS2X bridge
```

## RViz2

```bash
./ROS2X rviz
```

## Gazebo

```bash
./ROS2X gazebo
```

## Groot2

```bash
./ROS2X app groot
```

AppImage target path:
`apps/groot/groot.AppImage`

## QGroundControl

```bash
./ROS2X app qgroundcontrol
```

AppImage target path:
`apps/qgroundcontrol/qgroundcontrol.AppImage`

## Runtime Behavior

1. `ROS2X` resolves config and writes compose env.
2. `image-build` builds image only.
3. `build` runs workspace build inside container.
   - If `INSTALL_MICRO_ROS=true`, bootstrap runs first, then workspace build still runs.
4. `run` auto-builds only when workspace is not built, then executes `LAUNCH_COMMAND`.
5. `bridge`, `rviz`, `gazebo`, `app groot`, and `app qgroundcontrol` keep the container running after the GUI/process exits.
   Use `./ROS2X close` when you want to stop it.

## Compatibility

- Host OS: Linux (Docker Engine + Docker Compose v2 required)
- CPU arch: `x86_64`, `aarch64`
- ROS distro:
  - Script supports configurable distro values.
  - Docker image base is Ubuntu 22.04, so Jammy-aligned distros are the primary path.
- macOS and Windows hosts are not first-class targets for this repo's default setup.

## Native Installer (Optional)

Use the same installer script outside Docker on Ubuntu:

```bash
chmod +x scripts/install_ros2.sh
./scripts/install_ros2.sh --ros-distro humble --install-type development
```

Common flags:

- `--non-interactive`
- `--configure-bashrc yes|no|ask`
- `--skip-upgrade`
- `--no-install-recommends`
- `--allow-root`
- `--force-unsupported`

## Repository Layout

```text
ROS2X
├── ROS2X
├── docker
│   ├── Dockerfile
│   ├── docker-compose.yaml
│   ├── entrypoint.sh
│   └── .bashrc
├── apps
│   ├── gazebo
│   ├── groot
│   └── qgroundcontrol
├── ros2_ws
│   └── src
├── config
│   └── ros2x.conf.example
└── scripts
    ├── install_ros2.sh
    ├── install_micro_ros.sh
    ├── install_groot2.sh
    └── install_qgroundcontrol.sh
```

## Notes

- `network_mode: host` and `privileged: true` are enabled for robotics use cases.
- If hardware passthrough is not required, tighten container privileges accordingly.
- For multi-robot LAN scenarios, assign distinct `ROS_DOMAIN_ID` values.
