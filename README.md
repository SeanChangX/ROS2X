## ROS2X - ROS 2 Development Toolbox
```
        ____  ____  ________  _  __
       / __ \/ __ \/ ___/__ \| |/ /
      / /_/ / / / /\__ \__/ /|   / 
     / _, _/ /_/ /___/ / __//   |  
    /_/ |_|\____//____/____/_/|_|  
                               
[ ROS2X is an all-in-one Docker toolbox for ROS 2 development ]
```

### Key Features
- **Reproducible dev environment**: Docker image based on `osrf/ros:<distro>-desktop`
- **Host user mapping**: Files written in the container are owned by your host user
- **X11 GUI support**: Run RViz and other GUI apps from inside the container
- **micro-ROS**: Auto-clone and build micro-ROS workspace
- **Groot2 integration**: One command to download and run Groot2 AppImage

## Repository Layout
```
[ ROS2X ]
├── ROS2X                        # Main script
├── docker
│   ├── Dockerfile               # Multi-stage image with base tools and entrypoint
│   ├── docker-compose.yaml      # Container config (host networking, volumes, X11)
│   └── entrypoint.sh            # UID/GID mapping, micro-ROS install, build/run hooks
├── src                          # ROS 2 workspace source (mounted to /home/ros/workspace)
│   └── micro_ros/               # (optional) micro-ROS setup
├── groot
│   └── install_groot2.sh        # Downloads Groot2 AppImage on demand
└── scripts
    └── install_ros2_humble.sh   # Native install helper (Ubuntu 22.04)

```

## Prerequisites
- Linux with Docker Engine and Docker Compose v2
- Working X11 on the host (the script handles `xhost +local:` automatically)

## Quick Start
```
❯ ./ROS2X --help
----- [ ROS2X Usage ] -----------------------
|   build           - Build the ROS workspace
|   run             - Run the main program
|   groot           - Launch Groot2
|   enter           - Enter the container
|   close           - Stop the container
|   config          - Show current configuration
|   --command <cmd> - Run a custom command
|   --help          - Show this help message
---------------------------------------------
```

## Common Workflows
- **Develop**: put your ROS 2 packages under `src/`, then build with `./ROS2X build`.
- **Iterate inside container**: `./ROS2X enter`, then use `colcon build --symlink-install`.
- **Run GUI tools**: RViz and other GUI apps are supported via X11 mapping.
- **Use Groot2**: download and run the AppImage automatically:

```bash
./ROS2X groot
```

## Configuration
Run the following to see current settings:

```bash
./ROS2X config
```

You can customize these variables. The script writes `docker/.env` on each run.

- `ROS_DISTRO` (default: `humble`): ROS 2 distribution
- `ROS_DOMAIN_ID` (default: `100`): DDS domain isolation
- `PROJECT_NAME` (default: `ros2x`): container name
- `INSTALL_MICRO_ROS` (default: `true`): auto-install micro-ROS into `src/micro_ros`
- `LAUNCH_COMMAND`: command evaluated when using `./ROS2X run`

Ways to set variables:
- Edit the top of `ROS2X` and set your preferred defaults
- Or prefix for a single invocation (values are captured into `docker/.env`):

```bash
ROS_DISTRO=humble ROS_DOMAIN_ID=12 INSTALL_MICRO_ROS=false ./ROS2X build
```

## What the Container Does
- Uses host networking (`network_mode: host`) and privileged mode to access `/dev`
- Mounts the repository root to `/home/ros/workspace` inside the container
- Maps your host user ID and group ID for correct file ownership
- Optionally clones `micro_ros_setup` into `src/micro_ros` (if not present) and builds
- Runs your `LAUNCH_COMMAND` when `./ROS2X run` is used

## micro-ROS Notes
- Set `INSTALL_MICRO_ROS=false` to skip micro-ROS.
- If `src/micro_ros` already exists, it will be used and built.
- If you have a custom `scripts/build_micro_ros.sh`, it will be executed automatically after the base build.

## Native (Non-Docker) Install (Optional)
If you prefer a native Ubuntu 22.04 setup, use `scripts/install_ros2_humble.sh`.

```bash
chmod +x scripts/install_ros2_humble.sh
./scripts/install_ros2_humble.sh
```

This interactive script configures locales, apt sources, and installs `ros-humble-*` with development tools. It is intended for Ubuntu 22.04 (Jammy) on x86_64/aarch64.

## Troubleshooting
- **No GUI/Display**: Ensure X is running on host. The script runs `xhost +local:` automatically. On Wayland, you may need additional configuration.
- **Permission issues**: The entrypoint remaps user IDs to match your host. If you previously created files as root, fix ownership on the host: `sudo chown -R "$USER:$USER" .`.
- **DDS conflicts**: Change `ROS_DOMAIN_ID` to isolate networks when multiple ROS graphs are present.
- **Port conflicts**: Because host networking is used, ensure required ports aren’t blocked on your host.
