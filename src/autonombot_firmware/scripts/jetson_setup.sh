#!/bin/bash

# Jetson Xavier Setup Script for Autonombot
# This script prepares the Jetson Xavier for running the autonombot firmware

set -e  # Exit on any error

echo "=========================================="
echo "Autonombot Jetson Xavier Setup Script"
echo "=========================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Jetson
check_jetson() {
    if [ ! -f "/etc/nv_tegra_release" ]; then
        log_warn "This script is designed for NVIDIA Jetson devices."
        log_warn "Continuing anyway, but some features may not work."
    else
        log_info "Detected NVIDIA Jetson device"
        cat /etc/nv_tegra_release
    fi
}

# Update system packages
update_system() {
    log_info "Updating system packages..."
    sudo apt update
    sudo apt upgrade -y
}

# Install essential packages
install_essentials() {
    log_info "Installing essential packages..."
    sudo apt install -y \
        python3-pip \
        python3-dev \
        build-essential \
        cmake \
        git \
        curl \
        wget \
        nano \
        htop \
        screen \
        usbutils \
        can-utils
}

# Install GPIO libraries
install_gpio() {
    log_info "Installing GPIO libraries..."
    
    # Install Jetson.GPIO for newer JetPack versions
    sudo apt install -y python3-jetson-gpio
    
    # Also install RPi.GPIO for compatibility
    pip3 install RPi.GPIO --user
    
    # Install modern GPIO library
    sudo apt install -y libgpiod-dev python3-gpiod
    
    # Add user to gpio group
    sudo usermod -a -G gpio $USER
    
    log_info "GPIO libraries installed. You may need to logout/login for group changes to take effect."
}

# Install ROS 2 dependencies
install_ros_deps() {
    log_info "Installing ROS 2 dependencies..."
    sudo apt install -y \
        python3-serial \
        python3-smbus \
        ros-humble-hardware-interface \
        ros-humble-controller-manager \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        ros-humble-diagnostic-msgs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs
}

# Install sensor drivers
install_sensor_drivers() {
    log_info "Installing sensor drivers..."
    
    # RPLiDAR driver
    sudo apt install -y ros-humble-rplidar-ros
    
    # RealSense camera driver
    sudo apt install -y \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
        librealsense2-dev \
        librealsense2-utils
    
    # Serial communication
    sudo apt install -y libserial-dev
}

# Configure permissions
setup_permissions() {
    log_info "Setting up device permissions..."
    
    # Add user to dialout group (for serial communication)
    sudo usermod -a -G dialout $USER
    
    # Add user to video group (for camera access)
    sudo usermod -a -G video $USER
    
    # Create udev rules for consistent device naming
    sudo tee /etc/udev/rules.d/99-autonombot.rules > /dev/null <<EOF
# RPLiDAR
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"

# Arduino (adjust VID/PID as needed)
KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino_uno"
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="arduino_nano"

# USB Camera (if using USB camera instead of RealSense)
KERNEL=="video*", ATTRS{idVendor}=="046d", SYMLINK+="usb_camera"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    log_info "Device permissions configured. Please logout and login for group changes to take effect."
}

# Configure GPIO permissions
setup_gpio_permissions() {
    log_info "Setting up GPIO permissions..."
    
    # Create gpio group if it doesn't exist
    sudo groupadd -f gpio
    
    # Add user to gpio group
    sudo usermod -a -G gpio $USER
    
    # Set up GPIO permissions
    sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null <<EOF
# GPIO permissions for Jetson
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0664"
KERNEL=="gpiochip*", GROUP="gpio", MODE="0664"
EOF
    
    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
}

# Set up environment
setup_environment() {
    log_info "Setting up ROS 2 environment..."
    
    # Add ROS 2 setup to bashrc if not already there
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        log_info "Added ROS 2 Humble to ~/.bashrc"
    fi
    
    # Create convenient aliases
    if ! grep -q "# Autonombot aliases" ~/.bashrc; then
        cat >> ~/.bashrc << 'EOF'

# Autonombot aliases
alias ab_build='cd ~/autonombot_ws && colcon build --symlink-install'
alias ab_source='source ~/autonombot_ws/install/setup.bash'
alias ab_launch='ros2 launch autonombot_firmware jetson_hardware.launch.py'
alias ab_status='ros2 topic echo /sensor_manager/status'
alias ab_diagnostics='ros2 topic echo /diagnostics'
EOF
        log_info "Added convenience aliases to ~/.bashrc"
    fi
}

# Performance optimizations for Jetson
optimize_jetson() {
    log_info "Applying Jetson performance optimizations..."
    
    # Set Jetson to maximum performance mode
    sudo nvpmodel -m 0  # Max performance mode
    sudo jetson_clocks  # Enable max clocks
    
    # Increase swap space for build processes
    if [ ! -f /swapfile ]; then
        log_info "Creating swap file for build processes..."
        sudo fallocate -l 4G /swapfile
        sudo chmod 600 /swapfile
        sudo mkswap /swapfile
        sudo swapon /swapfile
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    
    log_info "Jetson optimizations applied"
}

# Create workspace
setup_workspace() {
    log_info "Setting up autonombot workspace..."
    
    WORKSPACE_DIR="$HOME/autonombot_ws"
    
    if [ ! -d "$WORKSPACE_DIR" ]; then
        mkdir -p "$WORKSPACE_DIR/src"
        log_info "Created workspace at $WORKSPACE_DIR"
    else
        log_info "Workspace already exists at $WORKSPACE_DIR"
    fi
    
    # Set workspace permissions
    chown -R $USER:$USER "$WORKSPACE_DIR"
}

# Test installations
test_installation() {
    log_info "Testing installations..."
    
    # Test GPIO
    if python3 -c "import RPi.GPIO" 2>/dev/null; then
        log_info "✓ RPi.GPIO library working"
    else
        log_warn "✗ RPi.GPIO library not working"
    fi
    
    if python3 -c "import Jetson.GPIO" 2>/dev/null; then
        log_info "✓ Jetson.GPIO library working"
    else
        log_warn "✗ Jetson.GPIO library not working"
    fi
    
    # Test ROS 2
    if command -v ros2 &> /dev/null; then
        log_info "✓ ROS 2 command available"
    else
        log_error "✗ ROS 2 not found in PATH"
    fi
    
    # Test hardware detection
    log_info "Hardware detection:"
    echo "USB devices:"
    lsusb | grep -E "(Intel|RealSense|Arduino|CP210|CH340)" || log_warn "No recognized devices found"
    
    echo "GPIO chips:"
    ls /dev/gpiochip* 2>/dev/null || log_warn "No GPIO chips found"
}

# Main installation flow
main() {
    log_info "Starting Jetson Xavier setup for Autonombot..."
    
    check_jetson
    update_system
    install_essentials
    install_gpio
    install_ros_deps
    install_sensor_drivers
    setup_permissions
    setup_gpio_permissions
    setup_environment
    optimize_jetson
    setup_workspace
    
    log_info "Installation completed!"
    log_info ""
    log_info "Next steps:"
    log_info "1. Logout and login again for group changes to take effect"
    log_info "2. Clone your autonombot code to ~/autonombot_ws/src/"
    log_info "3. Build the workspace: cd ~/autonombot_ws && colcon build"
    log_info "4. Source the workspace: source ~/autonombot_ws/install/setup.bash"
    log_info "5. Launch the robot: ros2 launch autonombot_firmware jetson_hardware.launch.py"
    log_info ""
    
    test_installation
    
    log_info "Setup script completed successfully!"
}

# Run main function
main "$@"
