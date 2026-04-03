#!/usr/bin/env bash

# ROS 2 Installation Script
# Default target: ROS 2 Humble on Ubuntu 22.04 (Jammy)
# Can be reused for other ROS distros when OS compatibility matches.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ROS_DISTRO="${ROS_DISTRO:-humble}"
INSTALL_TYPE="${INSTALL_TYPE:-development}"
NON_INTERACTIVE="${NON_INTERACTIVE:-false}"
SKIP_UPGRADE="${SKIP_UPGRADE:-false}"
CONFIGURE_BASHRC="${CONFIGURE_BASHRC:-ask}"
ALLOW_ROOT="${ALLOW_ROOT:-false}"
FORCE_UNSUPPORTED="${FORCE_UNSUPPORTED:-false}"
NO_INSTALL_RECOMMENDS="${NO_INSTALL_RECOMMENDS:-false}"

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

usage() {
    cat <<USAGE
Usage: $0 [options]

Options:
  --ros-distro <name>         ROS distro to install (default: ${ROS_DISTRO})
  --install-type <type>       desktop | ros-base | development (default: ${INSTALL_TYPE})
  --non-interactive           Run without prompts
  --configure-bashrc <mode>   yes | no | ask (default: ${CONFIGURE_BASHRC})
  --skip-upgrade              Skip apt upgrade step
  --no-install-recommends     Install apt packages without recommends
  --allow-root                Allow running as root (useful in Docker build)
  --force-unsupported         Continue even if distro/OS codename mismatch
  -h, --help                  Show this help
USAGE
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --ros-distro)
                ROS_DISTRO="$2"
                shift 2
                ;;
            --install-type)
                INSTALL_TYPE="$2"
                shift 2
                ;;
            --non-interactive)
                NON_INTERACTIVE="true"
                shift
                ;;
            --configure-bashrc)
                CONFIGURE_BASHRC="$2"
                shift 2
                ;;
            --skip-upgrade)
                SKIP_UPGRADE="true"
                shift
                ;;
            --no-install-recommends)
                NO_INSTALL_RECOMMENDS="true"
                shift
                ;;
            --allow-root)
                ALLOW_ROOT="true"
                shift
                ;;
            --force-unsupported)
                FORCE_UNSUPPORTED="true"
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done
}

run_as_root() {
    if [[ $EUID -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

apt_install() {
    if [[ "${NO_INSTALL_RECOMMENDS}" == "true" ]]; then
        run_as_root apt-get install -y --no-install-recommends "$@"
    else
        run_as_root apt-get install -y "$@"
    fi
}

check_root() {
    if [[ $EUID -eq 0 && "${ALLOW_ROOT}" != "true" ]]; then
        print_error "Running as root is blocked by default. Use --allow-root if needed."
        exit 1
    fi
}

check_ubuntu() {
    print_status "Checking OS compatibility..."

    if ! grep -q "Ubuntu" /etc/os-release; then
        print_error "This script only supports Ubuntu."
        exit 1
    fi

    UBUNTU_VERSION=$(grep "VERSION_ID" /etc/os-release | cut -d'"' -f2)
    UBUNTU_CODENAME=$(grep "VERSION_CODENAME" /etc/os-release | cut -d'=' -f2)
    ARCH=$(uname -m)

    print_status "Detected Ubuntu ${UBUNTU_VERSION} (${UBUNTU_CODENAME}), arch=${ARCH}"

    if [[ "${ARCH}" != "x86_64" && "${ARCH}" != "aarch64" ]]; then
        print_error "Unsupported architecture: ${ARCH}. Supported: x86_64, aarch64"
        exit 1
    fi

    expected_codename=""
    case "${ROS_DISTRO}" in
        humble|iron)
            expected_codename="jammy"
            ;;
        jazzy)
            expected_codename="noble"
            ;;
        rolling)
            expected_codename="${UBUNTU_CODENAME}"
            ;;
        *)
            print_warning "Unknown ROS_DISTRO=${ROS_DISTRO}. Skipping strict codename validation."
            expected_codename="${UBUNTU_CODENAME}"
            ;;
    esac

    if [[ "${UBUNTU_CODENAME}" != "${expected_codename}" ]]; then
        if [[ "${FORCE_UNSUPPORTED}" == "true" ]]; then
            print_warning "Codename mismatch: ${ROS_DISTRO} typically targets ${expected_codename}, current OS is ${UBUNTU_CODENAME}. Continuing due to --force-unsupported."
        else
            print_error "Codename mismatch: ${ROS_DISTRO} typically targets Ubuntu ${expected_codename}, current is ${UBUNTU_CODENAME}."
            print_error "Use --force-unsupported to continue anyway."
            exit 1
        fi
    fi

    print_success "OS compatibility check passed"
}

setup_locale() {
    print_status "Configuring locale..."

    if locale | grep -q "UTF-8"; then
        print_success "UTF-8 locale already configured"
        return
    fi

    run_as_root apt-get update -y
    apt_install locales
    run_as_root locale-gen en_US en_US.UTF-8
    run_as_root update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    print_success "Locale configured"
}

setup_ros2_sources() {
    print_status "Setting up ROS 2 apt sources..."

    run_as_root apt-get update -y
    apt_install \
        software-properties-common \
        curl \
        ca-certificates \
        gnupg2 \
        lsb-release

    run_as_root add-apt-repository universe -y || true

    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
        curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            | run_as_root gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi

    arch="$(dpkg --print-architecture)"
    codename="$(. /etc/os-release && echo "$VERSION_CODENAME")"
    echo "deb [arch=${arch} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" \
        | run_as_root tee /etc/apt/sources.list.d/ros2.list >/dev/null

    run_as_root apt-get update -y

    print_success "ROS 2 apt sources configured"
}

upgrade_system() {
    if [[ "${SKIP_UPGRADE}" == "true" ]]; then
        print_status "Skipping apt upgrade (--skip-upgrade)."
        return
    fi

    print_status "Upgrading system packages..."
    run_as_root apt-get upgrade -y
    print_success "System packages upgraded"
}

install_ros2() {
    print_status "Installing ROS 2 packages (${ROS_DISTRO}, ${INSTALL_TYPE})..."

    case "${INSTALL_TYPE}" in
        desktop)
            apt_install "ros-${ROS_DISTRO}-desktop"
            ;;
        ros-base)
            apt_install "ros-${ROS_DISTRO}-ros-base"
            ;;
        development)
            apt_install "ros-${ROS_DISTRO}-desktop" ros-dev-tools
            ;;
        *)
            print_error "Invalid install type: ${INSTALL_TYPE}. Use desktop|ros-base|development."
            exit 1
            ;;
    esac

    apt_install python3-colcon-common-extensions

    print_success "ROS 2 ${ROS_DISTRO} installation completed"
}

configure_shell() {
    case "${CONFIGURE_BASHRC}" in
        yes)
            should_add="yes"
            ;;
        no)
            should_add="no"
            ;;
        ask)
            if [[ "${NON_INTERACTIVE}" == "true" ]]; then
                should_add="no"
            else
                read -r -p "Add ROS environment to ~/.bashrc? (y/n): " answer
                if [[ "${answer}" == "y" || "${answer}" == "Y" ]]; then
                    should_add="yes"
                else
                    should_add="no"
                fi
            fi
            ;;
        *)
            print_error "Invalid CONFIGURE_BASHRC=${CONFIGURE_BASHRC}. Use yes|no|ask."
            exit 1
            ;;
    esac

    if [[ "${should_add}" != "yes" ]]; then
        print_status "Skipping ~/.bashrc modification"
        return
    fi

    target_home="${HOME}"
    if [[ $EUID -eq 0 && -n "${SUDO_USER:-}" ]]; then
        target_home="$(getent passwd "${SUDO_USER}" | cut -d: -f6)"
    fi

    bashrc_path="${target_home}/.bashrc"
    source_line="source /opt/ros/${ROS_DISTRO}/setup.bash"

    if [[ ! -f "${bashrc_path}" ]]; then
        touch "${bashrc_path}"
    fi

    if ! grep -q "${source_line}" "${bashrc_path}"; then
        {
            echo ""
            echo "# ROS 2 ${ROS_DISTRO}"
            echo "${source_line}"
        } >> "${bashrc_path}"
        print_success "Added ROS source line to ${bashrc_path}"
    else
        print_success "ROS source line already present in ${bashrc_path}"
    fi
}

test_installation() {
    print_status "Testing ROS 2 installation..."

    # ROS setup files may reference unset vars; keep strict mode for script logic,
    # but relax nounset only while sourcing underlay.
    set +u
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u

    if command -v ros2 >/dev/null 2>&1; then
        print_success "ros2 command is available"
        ros2 --help | head -5
    else
        print_error "ros2 command not found after install"
        exit 1
    fi

    print_success "Installation test completed"
}

cleanup_apt() {
    print_status "Cleaning apt cache..."
    run_as_root apt-get autoremove -y || true
    run_as_root apt-get clean -y || true
    run_as_root rm -rf /var/lib/apt/lists/*
}

main() {
    parse_args "$@"

    echo "=========================================="
    echo "ROS 2 Installation Script"
    echo "=========================================="
    echo "ROS_DISTRO      : ${ROS_DISTRO}"
    echo "INSTALL_TYPE    : ${INSTALL_TYPE}"
    echo "NON_INTERACTIVE : ${NON_INTERACTIVE}"
    echo "NO_RECOMMENDS   : ${NO_INSTALL_RECOMMENDS}"
    echo "=========================================="

    check_root
    check_ubuntu
    setup_locale
    setup_ros2_sources
    upgrade_system
    install_ros2
    configure_shell
    test_installation
    cleanup_apt

    print_success "All done"
}

main "$@"
