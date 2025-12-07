---
sidebar_position: 2
title: 'Chapter 1: Hardware & OS Configuration'
---

# Chapter 1: Hardware & OS Configuration

## Learning Objectives

By the end of this chapter, you should be able to:
- Identify the hardware requirements for Physical AI and humanoid robotics projects
- Configure Ubuntu 22.04 for NVIDIA hardware compatibility
- Install and configure NVIDIA drivers and CUDA toolkit
- Set up the basic development environment

## Prerequisites

Before starting this chapter, you should:
- Have access to compatible NVIDIA hardware (Jetson, GPU, etc.)
- Basic familiarity with Linux command line
- Understanding of hardware architecture concepts

## Introduction

This chapter covers the foundational hardware and operating system configuration needed for Physical AI and humanoid robotics projects using NVIDIA's ecosystem. We'll focus on setting up Ubuntu 22.04 with proper NVIDIA hardware support, which forms the backbone of our development environment.

## Hardware Requirements

### NVIDIA Jetson Platforms (Recommended for Edge Robotics)

- **Jetson Orin NX (16GB)**: Most powerful option for edge AI on robots
- **Jetson Orin Nano (8GB)**: Cost-effective alternative with good performance
- **Jetson AGX Orin**: For compute-intensive applications

### Workstation Options (For Simulation and Heavy Computation)

- **NVIDIA RTX 4090**: Top-tier for training and simulation
- **NVIDIA RTX 4080**: Good balance of performance and cost
- **NVIDIA RTX A5000**: Professional workstation GPU

### Additional Hardware

- **Robots**: 12-DOF bipedal robot kit or equivalent
- **Sensors**: RGB-D cameras, LiDAR, IMU
- **Networking**: Reliable WiFi 6 or Ethernet connection

## OS Installation: Ubuntu 22.04 LTS

### Base Installation

1. Download Ubuntu 22.04 LTS ISO from the official website
2. Create a bootable USB using tools like Rufus (Windows) or `dd` (Linux/Mac)
3. Boot from USB and follow standard installation process
4. During installation, ensure you select:
   - Minimal installation (to avoid unnecessary packages)
   - Third-party software for graphics and Wi-Fi hardware
   - Encryption disabled (for compatibility with some robotics tools)

### Initial Configuration

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install essential development tools
sudo apt install build-essential cmake git curl wget vim htop -y

# Install Python development tools
sudo apt install python3-dev python3-pip python3-venv -y
```

## NVIDIA Driver Installation

### For Jetson Platforms

Jetson platforms come with pre-installed NVIDIA drivers. Verify your installation:

```bash
# Check JetPack version
cat /etc/nv_tegra_release

# Verify GPU status
sudo nvpmodel -q
jtop
```

### For Workstation GPUs

1. **Disable Nouveau driver** (if present):

```bash
# Check if nouveau is loaded
lsmod | grep nouveau

# If loaded, create blacklist file
sudo bash -c "echo 'blacklist nouveau' >> /etc/modprobe.d/blacklist-nouveau.conf"
sudo bash -c "echo 'options nouveau modeset=0' >> /etc/modprobe.d/blacklist-nouveau.conf"
sudo update-initramfs -u
sudo reboot
```

2. **Download and install NVIDIA driver**:

```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install latest NVIDIA driver
sudo apt install nvidia-driver-535  # Or latest available version
sudo reboot
```

3. **Verify installation**:

```bash
nvidia-smi
```

## CUDA Toolkit Installation

### For Jetson Platforms

CUDA is pre-installed with JetPack. Verify with:

```bash
nvcc --version
```

### For Workstation GPUs

```bash
# Download CUDA toolkit (replace with latest version)
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run

# Make executable and run
chmod +x cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run
```

During installation, select:
- CUDA Toolkit
- CUDA Samples
- CUDA Documentation

### Configure Environment Variables

Add to your `~/.bashrc`:

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

Then source the file:

```bash
source ~/.bashrc
```

## ROS 2 Humble Installation

ROS 2 Humble Hawksbill is our recommended version for compatibility with NVIDIA platforms.

```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## Development Environment Setup

### Create Workspace

```bash
# Create ROS workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws
colcon build
source install/setup.bash
```

### Install Additional Tools

```bash
# Install Docker for containerized development
sudo apt install docker.io
sudo usermod -aG docker $USER  # Log out and back in for changes to take effect

# Install VS Code for development
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64 signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# Install additional Python packages
pip3 install numpy scipy matplotlib pyyaml
```

## Workstation vs Cloud Comparison

| Aspect | Workstation | Cloud |
|--------|-------------|-------|
| Performance | High, dedicated | Variable, shared |
| Cost | High upfront | Pay-as-you-go |
| Availability | Always available | Internet dependent |
| Maintenance | Your responsibility | Provider responsibility |
| Security | Your control | Provider control |

## Summary

In this chapter, we've covered the essential hardware and OS configuration needed for Physical AI and humanoid robotics projects. We've set up Ubuntu 22.04 with NVIDIA hardware support, installed CUDA, and prepared the ROS 2 development environment.

## Next Steps

Now that your hardware and OS are configured, continue to [Chapter 2: Edge Ecosystem (Jetson Setup)](./chapter-2-edge-ecosystem.md) to learn about setting up your edge computing platform.

## Exercises

1. Verify your NVIDIA driver installation with `nvidia-smi`
2. Confirm CUDA installation with `nvcc --version`
3. Test ROS 2 installation by creating a simple publisher/subscriber node