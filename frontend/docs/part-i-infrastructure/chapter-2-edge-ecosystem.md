---
sidebar_position: 3
title: 'Chapter 2: Edge Ecosystem (Jetson Setup)'
---

# Chapter 2: Edge Ecosystem (Jetson Setup)

## Learning Objectives

By the end of this chapter, you should be able to:
- Configure JetPack 6.0 for NVIDIA Jetson platforms
- Set up containerized environments for robotics applications
- Install and configure jtop for system monitoring
- Configure power modes and thermal management
- Deploy basic ROS 2 applications to Jetson platforms

## Prerequisites

Before starting this chapter, you should:
- Have a NVIDIA Jetson platform (Orin NX, Orin Nano, or AGX Orin)
- Complete Chapter 1: Hardware & OS Configuration
- Basic understanding of containerization concepts

## Introduction

This chapter focuses on setting up the NVIDIA Jetson platform as an edge computing solution for humanoid robotics. We'll explore JetPack 6.0, which provides the complete software stack for Jetson platforms, including CUDA, cuDNN, TensorRT, and support for containerized applications.

## JetPack 6.0 Installation and Setup

### Initial Setup

JetPack 6.0 comes pre-installed on new Jetson platforms. To verify your installation:

```bash
# Check JetPack version
cat /etc/nv_tegra_release

# Verify system information
sudo /opt/nvidia/jetson-io/configure-jetson.sh --query

# Check available memory and storage
free -h
df -h
```

### System Configuration

1. **Configure power mode**:

```bash
# Check available power modes
sudo nvpmodel -q

# Set to MAXN mode for maximum performance (default)
sudo nvpmodel -m 0

# For thermal management, set to 15W mode
sudo nvpmodel -m 1
```

2. **Configure Jetson Clocks**:

```bash
# Set clocks to maximum performance
sudo jetson_clocks

# Note: This should only be used for performance testing
# For normal operation, let the system manage clocks automatically
```

## jtop Installation and Configuration

jtop is an essential tool for monitoring Jetson system status.

```bash
# Install jtop
sudo -H pip3 install -U jetson-stats

# Enable jtop service
sudo systemctl restart jtop.service

# Launch jtop in terminal
jtop
```

jtop provides real-time monitoring of:
- GPU utilization and temperature
- CPU usage and frequency
- Memory usage
- Power consumption
- Fan speed (if applicable)

## Containerized Development Environment

### Docker Setup

Docker is pre-installed on Jetson platforms, but we need to configure it for GPU access:

```bash
# Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in, or run:
newgrp docker

# Test Docker installation
docker run --rm hello-world
```

### NVIDIA Container Runtime

Configure Docker to work with NVIDIA GPUs:

```bash
# Install nvidia-container-toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure the container runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker daemon
sudo systemctl restart docker

# Test GPU access in container
docker run --rm --gpus all nvidia/cuda:12.3.0-devel-ubuntu22.04 nvidia-smi
```

### Docker Compose Setup

```bash
# Install Docker Compose
sudo apt install docker-compose-plugin

# Or install via pip
sudo -H pip3 install docker-compose
```

## ROS 2 on Jetson

### Installing ROS 2 Humble

ROS 2 Humble is compatible with JetPack 6.0:

```bash
# Update package lists
sudo apt update

# Install ROS 2 base
sudo apt install ros-humble-ros-base

# Install additional packages for robotics
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-rosbridge-suite
sudo apt install ros-humble-robot-localization
```

### Environment Setup

Add to your `~/.bashrc`:

```bash
# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Additional ROS 2 workspace setup (if applicable)
# source ~/physical_ai_ws/install/setup.bash
```

## Containerized ROS 2 Applications

### Creating a ROS 2 Container

Create a Dockerfile for your ROS 2 application:

```dockerfile
FROM nvcr.io/nvidia/l4t-ml:r36.2.0-py3

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-ros-core \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Build command
CMD ["bash"]
```

### Running ROS 2 in Container

```bash
# Build the image
docker build -t ros2-jetson-app .

# Run with GPU access
docker run -it --gpus all --rm ros2-jetson-app

# For interactive development
docker run -it --gpus all --network host --rm ros2-jetson-app
```

## Power Management and Thermal Considerations

### Power Modes

Jetson platforms support multiple power modes that affect performance and thermal characteristics:

```bash
# Check current power mode
sudo nvpmodel -q

# List all available power modes
sudo nvpmodel -l
```

Common power modes:
- **MAXN (Mode 0)**: Maximum performance, higher power consumption
- **15W (Mode 1)**: Balanced performance and power
- **10W (Mode 2)**: Lower power, reduced performance
- **5W (Mode 3)**: Minimum power, lowest performance

### Thermal Management

Monitor thermal status:

```bash
# Using jtop
jtop

# Command line thermal info
cat /sys/class/thermal/thermal_zone*/temp

# Check thermal throttling
sudo tegrastats
```

## Performance Optimization

### GPU Memory Management

Configure GPU memory allocation:

```bash
# Check GPU memory
nvidia-smi

# For applications requiring large GPU memory, consider:
# - Using TensorRT for optimized inference
# - Implementing memory pooling strategies
# - Using mixed precision (FP16) where possible
```

### CPU Affinity and Scheduling

For real-time performance:

```bash
# Check CPU information
lscpu

# Use taskset to bind processes to specific CPUs
taskset -c 0-3 your_application
```

## Deployment Strategies

### APTOS (Adaptive Platform for Trustworthy and Optimized Systems)

For production deployment on Jetson:

```bash
# Install additional tools for deployment
sudo apt install ros-humble-ros-environment

# Create a deployment script
cat > deploy.sh << 'EOF'
#!/bin/bash

# Setup environment
source /opt/ros/humble/setup.bash
source ~/physical_ai_ws/install/setup.bash

# Launch main application
ros2 launch robot_bringup robot.launch.py
EOF

chmod +x deploy.sh
```

### Container-based Deployment

For consistent deployment across multiple Jetson devices:

```yaml
# docker-compose.yml
version: '3.8'
services:
  ros2-core:
    image: ros2-jetson-app
    container_name: ros2_core
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - ROS_DOMAIN_ID=0
    network_mode: host
    volumes:
      - ./workspace:/workspace
    command: ["ros2", "launch", "robot_bringup", "robot.launch.py"]
```

## Troubleshooting Common Issues

### GPU Memory Issues

```bash
# Check available GPU memory
nvidia-smi

# Clear GPU memory cache (if using PyTorch)
# In Python:
# import torch
# torch.cuda.empty_cache()
```

### Power Limitations

If experiencing performance issues:

```bash
# Check if system is thermally throttled
sudo tegrastats

# Switch to higher power mode
sudo nvpmodel -m 0
```

## Summary

This chapter covered setting up the NVIDIA Jetson platform as an edge computing solution for humanoid robotics. We explored JetPack 6.0, configured containerized environments, installed monitoring tools, and prepared the platform for ROS 2 applications.

## Next Steps

Now that your Jetson platform is configured, continue to [Chapter 3: ROS 2 Architecture](../part-ii-ros-nervous-system/chapter-3-ros2-architecture.md) to learn about the Robot Operating System architecture that will serve as the nervous system for your humanoid robot.

## Exercises

1. Configure your Jetson to run in MAXN mode and verify with `sudo nvpmodel -q`
2. Install and run jtop to monitor system resources
3. Create and run a simple Docker container with GPU access
4. Verify ROS 2 installation by running `ros2 topic list`