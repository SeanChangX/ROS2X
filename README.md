# ROS2X

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20(default)-22314E.svg)](https://docs.ros.org/en/humble/index.html)
[![Docker](https://img.shields.io/badge/Docker-Compose-2496ED.svg)](https://docs.docker.com/compose/)
[![Arch](https://img.shields.io/badge/Arch-x86__64%20%7C%20aarch64-6f42c1.svg)](#compatibility)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25.svg)](ROS2X)

ROS2X is a Docker-first ROS 2 development toolbox focused on reproducible,
robotics-friendly workflows. It provides one entrypoint script for image build,
workspace build, launch/run, micro-ROS bootstrap, Foxglove Bridge, and Groot2.

## Highlights

- Ubuntu 22.04 base image (`docker/Dockerfile`)
- Host UID/GID mapping for correct file ownership
- X11 GUI support for RViz and AppImage tools
- Integrated micro-ROS bootstrap flow (`scripts/install_micro_ros.sh`)
- Groot2 AppImage launcher (`./ROS2X app groot`)
- Foxglove Bridge launcher (`./ROS2X bridge`)
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
| `./ROS2X app groot` | Launch Groot2 AppImage |
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
- `FOXGLOVE_PORT` (default: `8765`), `FOXGLOVE_ADDRESS` (default: `0.0.0.0`)

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

## Groot2

```bash
./ROS2X app groot
```

AppImage target path:
`apps/groot/groot.AppImage`

## Runtime Behavior

1. `ROS2X` resolves config and writes compose env.
2. `image-build` builds image only.
3. `build` runs workspace build inside container.
   - If `INSTALL_MICRO_ROS=true`, bootstrap runs first, then workspace build still runs.
4. `run` auto-builds only when workspace is not built, then executes `LAUNCH_COMMAND`.
5. `app groot` and `bridge` only stop the container if they started it in that invocation.
   Existing running containers are kept alive.

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
chmod +x scripts/install_ros2_humble.sh
./scripts/install_ros2_humble.sh --ros-distro humble --install-type development
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
│   └── groot
├── ros2_ws
│   └── src
├── config
│   └── ros2x.conf.example
└── scripts
    ├── install_ros2_humble.sh
    ├── install_micro_ros.sh
    └── install_groot2.sh
```

## Notes

- `network_mode: host` and `privileged: true` are enabled for robotics use cases.
- If hardware passthrough is not required, tighten container privileges accordingly.
- For multi-robot LAN scenarios, assign distinct `ROS_DOMAIN_ID` values.
