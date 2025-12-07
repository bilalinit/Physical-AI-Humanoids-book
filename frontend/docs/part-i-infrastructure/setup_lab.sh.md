---
sidebar_position: 10
title: 'Setup Lab Script'
description: 'Complete setup script for Physical AI development environment'
---

# Setup Lab Script

## Overview

This script automates the setup of a complete Physical AI development environment on Ubuntu 22.04 with NVIDIA hardware support. The script will install and configure:

- NVIDIA drivers and CUDA toolkit
- ROS 2 Humble Hawksbill
- JetPack 6.0 (for Jetson platforms)
- Isaac Sim dependencies
- Required Python packages
- Git configuration for robotics development

## Prerequisites

- Ubuntu 22.04 LTS
- NVIDIA GPU (RTX 4070 Ti or better recommended)
- Internet connection
- At least 50GB free disk space

## Usage

```bash
# Make the script executable
chmod +x setup_lab.sh

# Run the script (with optional parameters)
./setup_lab.sh --cuda-version 12.4 --ros-distro humble --isaac-sim-version 4.2.0
```

## Script Contents

```bash
#!/bin/bash

# setup_lab.sh - Physical AI Development Environment Setup Script
# Author: Physical AI Team
# Date: $(date +%Y-%m-%d)
# Description: Complete setup for Physical AI development with ROS 2, NVIDIA tools, and AI frameworks

set -e  # Exit on any error

# Default values
CUDA_VERSION="12.4"
ROS_DISTRO="humble"
ISAAC_SIM_VERSION="4.2.0"
WORKSPACE_DIR="$HOME/physical_ai_ws"
NVIDIA_DRIVER_VERSION="550"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${GREEN}================================${NC}"
    echo -e "${GREEN}Physical AI Development Setup${NC}"
    echo -e "${GREEN}================================${NC}"
    echo
}

print_step() {
    echo -e "${YELLOW}>>> $1${NC}"
}

confirm_action() {
    read -p "Continue with setup? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 1
    fi
}

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "Please do not run as root"
    exit 1
fi

# Check Ubuntu version
if ! lsb_release -d | grep -q "Ubuntu 22.04"; then
    echo "This script requires Ubuntu 22.04"
    exit 1
fi

print_header
confirm_action

# Update system
print_step "Updating system packages"
sudo apt update && sudo apt upgrade -y

# Install basic development tools
print_step "Installing basic development tools"
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    htop \
    tmux \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-setuptools \
    software-properties-common \
    gnupg \
    lsb-release \
    apt-transport-https \
    ca-certificates \
    gnupg-agent

# Install NVIDIA drivers
print_step "Installing NVIDIA drivers (version $NVIDIA_DRIVER_VERSION)"
sudo apt install -y nvidia-driver-$NVIDIA_DRIVER_VERSION
sudo apt install -y nvidia-utils-$NVIDIA_DRIVER_VERSION

# Install CUDA
print_step "Installing CUDA $CUDA_VERSION"
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-toolkit-$CUDA_VERSION

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Install ROS 2 Humble
print_step "Installing ROS 2 Humble Hawksbill"
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-$ROS_DISTRO-desktop
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
sudo rosdep init
rosdep update

# Install ROS 2 dependencies
sudo apt install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-tf2-geometry-msgs \
    ros-$ROS_DISTRO-tf2-eigen \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-geometry2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-isaac-sim-ros-bridge

# Install Python packages for AI development
print_step "Installing Python packages for AI development"
pip3 install --user --upgrade pip
pip3 install --user \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu124 \
    numpy scipy matplotlib pandas \
    jupyter notebook ipykernel \
    transformers accelerate diffusers \
    openai anthropic \
    opencv-python \
    mediapipe \
    pyquaternion transforms3d \
    pygame

# Install additional tools
print_step "Installing additional tools"
sudo apt install -y \
    v4l-utils \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer-plugins-base1.0-dev \
    libeigen3-dev \
    libboost-all-dev \
    libopencv-dev \
    python3-opencv

# Install Jetson-specific tools (if on Jetson platform)
if [ -f /etc/nv_tegra_release ]; then
    print_step "Installing Jetson-specific tools"
    sudo apt install -y nvidia-jetpack
    sudo apt install -y jtop
    sudo systemctl enable jtop.service
fi

# Create workspace directory
print_step "Creating workspace directory at $WORKSPACE_DIR"
mkdir -p $WORKSPACE_DIR/src
cd $WORKSPACE_DIR

# Source ROS environment
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc

# Install colcon for building ROS packages
pip3 install --user -U colcon-common-extensions

# Install additional ROS tools
sudo apt install -y \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-slam-toolbox

# Install Isaac Sim prerequisites
print_step "Installing Isaac Sim prerequisites"
sudo apt install -y \
    mesa-utils \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libxrender1 \
    libxrandr2 \
    libxss1 \
    libgtk-3-0 \
    libgconf-2-4 \
    libasound2

# Install Docker (for Isaac Sim container)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Create common directories
mkdir -p ~/physical_ai_ws/{src,models,logs,experiments,scripts}

# Install monitoring utilities
print_step "Installing monitoring utilities"
sudo apt install -y htop nmon iotop

# Install jtop (for Jetson platforms)
if [ -f /etc/nv_tegra_release ]; then
    pip3 install --user jetson-stats
    sudo -H jetson_config
fi

# Setup git configuration for robotics development
print_step "Configuring Git for robotics development"
git config --global user.name "Physical AI Developer"
git config --global user.email "developer@physical-ai.org"
git config --global core.editor "vim"
git config --global push.default simple

# Create a sample ROS 2 workspace with basic packages
print_step "Creating sample ROS 2 workspace"
cd $WORKSPACE_DIR
colcon build --symlink-install

# Add aliases to .bashrc
echo "" >> ~/.bashrc
echo "# Physical AI aliases" >> ~/.bashrc
echo "alias cw='cd $WORKSPACE_DIR'" >> ~/.bashrc
echo "alias cs='cd $WORKSPACE_DIR/src'" >> ~/.bashrc
echo "alias cb='cd $WORKSPACE_DIR && colcon build --symlink-install'" >> ~/.bashrc
echo "alias sb='source $WORKSPACE_DIR/install/setup.bash'" >> ~/.bashrc
echo "alias rgs='ros2 run gazebo_ros gazebo'" >> ~/.bashrc
echo "alias rns='ros2 run nav2_bringup nav2_bringup_launch.py'" >> ~/.bashrc

# Create a verification script
cat > $WORKSPACE_DIR/verify_setup.sh << 'EOF'
#!/bin/bash
echo "=== Physical AI Development Environment Verification ==="
echo

echo "1. Checking NVIDIA GPU..."
nvidia-smi || echo "NVIDIA GPU not detected or drivers not working"

echo
echo "2. Checking CUDA..."
nvcc --version || echo "CUDA not installed properly"

echo
echo "3. Checking ROS 2..."
source /opt/ros/humble/setup.bash
ros2 --version || echo "ROS 2 not installed properly"

echo
echo "4. Checking Python packages..."
python3 -c "import torch; print(f'PyTorch: {torch.__version__}')"
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
python3 -c "import numpy; print(f'NumPy: {numpy.__version__}')"

echo
echo "5. Checking workspace..."
if [ -d "$HOME/physical_ai_ws/install" ]; then
    echo "ROS workspace exists and is built"
else
    echo "ROS workspace not built properly"
fi

echo
echo "=== Verification Complete ==="
EOF

chmod +x $WORKSPACE_DIR/verify_setup.sh

print_step "Setup complete!"
echo
echo "To complete the installation:"
echo "1. Reboot your system: sudo reboot"
echo "2. After reboot, run: $WORKSPACE_DIR/verify_setup.sh"
echo "3. Start developing: cd $WORKSPACE_DIR && source install/setup.bash"
echo
echo "For NVIDIA Jetson platforms, jtop service is now enabled and will auto-start"
echo "You can monitor your Jetson with: jtop"