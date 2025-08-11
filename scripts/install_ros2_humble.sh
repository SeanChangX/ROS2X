#!/bin/bash

# ROS 2 Humble Installation Script
# Based on official documentation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# Maintainer: SeanChangX <scx@gapp.nthu.edu.tw>
# Version: 2025-08-07

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
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

# Function to check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        print_error "This script should not be run as root. Please run as a regular user."
        exit 1
    fi
}

# Function to check Ubuntu version
check_ubuntu_version() {
    print_status "Checking Ubuntu version..."
    
    # Check if running Ubuntu
    if ! grep -q "Ubuntu" /etc/os-release; then
        print_error "This script is designed for Ubuntu systems only."
        exit 1
    fi
    
    # Get Ubuntu version
    UBUNTU_VERSION=$(grep "VERSION_ID" /etc/os-release | cut -d'"' -f2)
    UBUNTU_CODENAME=$(grep "VERSION_CODENAME" /etc/os-release | cut -d'=' -f2)
    
    print_status "Detected Ubuntu version: $UBUNTU_VERSION ($UBUNTU_CODENAME)"
    
    # Check if version is supported (Ubuntu 22.04 Jammy)
    if [[ "$UBUNTU_CODENAME" != "jammy" ]]; then
        print_error "ROS 2 Humble is only supported on Ubuntu 22.04 (Jammy Jellyfish)."
        print_error "Current version: $UBUNTU_VERSION ($UBUNTU_CODENAME)"
        print_error "Please upgrade to Ubuntu 22.04 or use a different ROS 2 distribution."
        exit 1
    fi
    
    print_success "Ubuntu version is compatible with ROS 2 Humble"
}

# Function to check system architecture
check_architecture() {
    print_status "Checking system architecture..."
    
    ARCH=$(uname -m)
    print_status "Detected architecture: $ARCH"
    
    # Check if architecture is supported
    if [[ "$ARCH" != "x86_64" && "$ARCH" != "aarch64" ]]; then
        print_error "Unsupported architecture: $ARCH"
        print_error "ROS 2 Humble supports: x86_64, aarch64"
        exit 1
    fi
    
    print_success "Architecture is supported"
}

# Function to set locale
setup_locale() {
    print_status "Setting up locale..."
    
    # Check current locale
    if ! locale | grep -q "UTF-8"; then
        print_warning "UTF-8 locale not detected. Setting up proper locale..."
        
        sudo apt update && sudo apt install -y locales
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
        
        print_success "Locale configured"
    else
        print_success "UTF-8 locale already configured"
    fi
}

# Function to setup ROS 2 sources
setup_sources() {
    print_status "Setting up ROS 2 apt sources..."
    
    # Enable Universe repository
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe -y
    
    # Install curl if not present
    sudo apt update && sudo apt install -y curl
    
    # Get latest ros-apt-source version
    print_status "Downloading latest ros-apt-source package..."
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    
    # Download and install ros-apt-source
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    
    # Update package lists
    sudo apt update
    
    print_success "ROS 2 apt sources configured"
}

# Function to upgrade system packages
upgrade_system() {
    print_status "Upgrading system packages..."
    print_warning "This step is important to avoid conflicts with system packages"
    
    sudo apt upgrade -y
    
    print_success "System packages upgraded"
}

# Function to install ROS 2 packages
install_ros2() {
    local install_type=$1
    
    print_status "Installing ROS 2 Humble $install_type..."
    
    case $install_type in
        "desktop")
            sudo apt install -y ros-humble-desktop
            print_success "ROS 2 Humble Desktop installed"
            ;;
        "ros-base")
            sudo apt install -y ros-humble-ros-base
            print_success "ROS 2 Humble ROS-Base installed"
            ;;
        "development")
            sudo apt install -y ros-humble-desktop ros-dev-tools
            print_success "ROS 2 Humble Desktop with development tools installed"
            ;;
        *)
            print_error "Invalid installation type: $install_type"
            exit 1
            ;;
    esac
}

# Function to setup environment
setup_environment() {
    print_status "Setting up ROS 2 environment..."
    sudo apt install -y python3-colcon-common-extensions
    
    # Ask user if they want to add ROS 2 environment to .bashrc
    read -p "Do you want to add ROS 2 environment to .bashrc? (y/n): " add_to_bashrc
    
    if [[ "$add_to_bashrc" == "y" ]]; then
        # Add to .bashrc if not already present
        if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
            echo "" >> ~/.bashrc
            echo "# ROS 2 Humble" >> ~/.bashrc
            echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
            print_success "ROS 2 environment added to .bashrc"
        else
            print_success "ROS 2 environment already in .bashrc"
        fi
    fi
    
    print_success "Environment setup completed"
}

# Function to test installation
test_installation() {
    print_status "Testing ROS 2 installation..."
    
    # Source ROS 2 environment
    source /opt/ros/humble/setup.bash
    
    # Test ROS 2 command
    if command -v ros2 &> /dev/null; then
        print_success "ROS 2 command is available"
        ros2 --help | head -5
    else
        print_error "ROS 2 command not found"
        exit 1
    fi
    
    print_success "Installation test completed"
}

# Function to show installation options
show_installation_options() {
    echo ""
    echo "=========================================="
    echo "ROS 2 Humble Installation Options"
    echo "=========================================="
    echo "1. Desktop Install (Recommended)"
    echo "   - Includes: ROS, RViz, demos, tutorials"
    echo "   - Best for: Most users, GUI applications"
    echo ""
    echo "2. ROS-Base Install (Bare Bones)"
    echo "   - Includes: Communication libraries, message packages, command line tools"
    echo "   - Best for: Headless systems, minimal installations"
    echo ""
    echo "3. Development Install"
    echo "   - Includes: Desktop + development tools (compilers, etc.)"
    echo "   - Best for: Developers who want to build ROS packages"
    echo ""
    echo "4. Exit"
    echo ""
}

# Function to get user choice
get_user_choice() {
    while true; do
        read -p "Please select an installation option (1-4): " choice
        case $choice in
            1)
                echo "desktop"
                return
                ;;
            2)
                echo "ros-base"
                return
                ;;
            3)
                echo "development"
                return
                ;;
            4)
                echo "exit"
                return
                ;;
            *)
                print_error "Invalid option. Please select 1-4." >&2
                ;;
        esac
    done
}

# Function to show next steps
show_next_steps() {
    echo ""
    echo "=========================================="
    echo "Installation Complete!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "1. Open a new terminal or run: source ~/.bashrc"
    echo "2. Test the installation: ros2 --help"
    echo "3. Try the demo:"
    echo "   Terminal 1: ros2 run demo_nodes_cpp talker"
    echo "   Terminal 2: ros2 run demo_nodes_py listener"
    echo ""
    echo "Useful commands:"
    echo "- ros2 --help          : Show all available commands"
    echo "- ros2 node list       : List running nodes"
    echo "- ros2 topic list      : List available topics"
    echo "- ros2 service list    : List available services"
    echo ""
    echo "Documentation:"
    echo "- Tutorials: https://docs.ros.org/en/humble/Tutorials.html"
    echo "- API Reference: https://docs.ros.org/en/humble/"
    echo ""
    echo "To uninstall ROS 2 Humble:"
    echo "sudo apt remove ~nros-humble-* && sudo apt autoremove"
    echo ""
}

# Main installation function
main() {
    echo "=========================================="
    echo "ROS 2 Humble Installation Script"
    echo "=========================================="
    echo "Based on official documentation"
    echo "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    echo ""
    
    # Check if not running as root
    check_root
    
    # Check system requirements
    check_ubuntu_version
    check_architecture
    
    # Show installation options
    show_installation_options
    install_type=$(get_user_choice)
    
    # Check if user chose to exit
    if [[ "$install_type" == "exit" ]]; then
        print_status "Installation cancelled by user"
        exit 0
    fi
    
    echo ""
    print_status "Starting ROS 2 Humble $install_type installation..."
    echo ""
    
    # Perform installation steps
    setup_locale
    setup_sources
    upgrade_system
    install_ros2 "$install_type"
    setup_environment
    test_installation
    
    # Show next steps
    show_next_steps
}

# Run main function
main "$@" 