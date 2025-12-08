---
title: ROS 2 Hardware Setup Guide
---

# ROS 2 Hardware Setup Guide

## Overview

This guide provides instructions for setting up the hardware required for the ROS 2 module of the Physical AI & Humanoid Robotics course. The required hardware varies based on whether you're running ROS 2 in simulation or on physical platforms.

## Equipment List

### Minimum Requirements
- Computer with 8GB RAM (16GB recommended)
- 64-bit processor with 4+ cores
- 20GB free disk space
- Ubuntu 22.04 LTS or Windows 10/11 with WSL2

### Recommended Hardware for Physical Robotics
- Single-board computer (e.g., Raspberry Pi 4, NVIDIA Jetson Nano)
- Robot platform with ROS-compatible drivers
- USB camera or depth sensor
- Network router for local communication

## Software Installation

### Option 1: Native Installation (Ubuntu)
1. Update system packages:
   ```bash
   sudo apt update
   ```

2. Set locale:
   ```bash
   locale  # check for UTF-8
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US.UTF-8
   ```

3. Add ROS 2 repository:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. Install ROS 2 Humble Hawksbill:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. Install additional packages:
   ```bash
   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-rosdep
   sudo apt install python3-argcomplete
   ```

6. Source the ROS 2 setup:
   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Option 2: Windows with WSL2
1. Install WSL2 with Ubuntu 22.04:
   ```cmd
   wsl --install -d Ubuntu-22.04
   ```

2. Follow the native installation steps within the WSL environment.

### Option 3: Docker Container
1. Install Docker Desktop
2. Pull the ROS 2 container:
   ```bash
   docker pull osrf/ros:humble-desktop
   ```
3. Run the container:
   ```bash
   docker run -it --rm --name ros2_env osrf/ros:humble-desktop
   ```

## Hardware Configuration

### Setting Up a Single Board Computer (Raspberry Pi)

1. Flash the OS:
   - Download Raspberry Pi OS (64-bit) or Ubuntu 22.04
   - Use Raspberry Pi Imager to flash the SD card

2. Install ROS 2 on the device:
   - Follow the ROS 2 installation guide for ARM64
   - Configure network settings to communicate with your main development machine

3. Test the setup:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 run demo_nodes_cpp talker
   ```

### Connecting Sensors

1. For USB cameras:
   - Connect to the robot's computer
   - Verify detection: `lsusb`
   - Install appropriate drivers if needed

2. For depth cameras (e.g., Intel RealSense):
   - Install camera drivers
   - Test with provided tools: `realsense-viewer`

## Network Configuration

### Setting Up Robot Communication

1. Ensure both development machine and robot are on the same network
2. Configure ROS 2 environment variables:
   ```bash
   export ROS_DOMAIN_ID=1
   export ROS_LOCALHOST_ONLY=0
   ```
3. Test communication between machines using ROS 2 tools

## Troubleshooting

### Common Issues and Solutions

1. **Permission Denied Errors**
   - Add user to dialout group: `sudo usermod -a -G dialout $USER`
   - Log out and back in for changes to take effect

2. **Package Installation Failures**
   - Update package lists: `sudo apt update`
   - Fix broken dependencies: `sudo apt --fix-broken install`

3. **Network Communication Issues**
   - Check firewall settings
   - Verify both machines are on the same ROS_DOMAIN_ID
   - Use `ros2 topic list` to verify communication

4. **Performance Issues**
   - Ensure adequate RAM and processor resources
   - Close unnecessary applications during intensive robotics tasks
   - Consider using lightweight ROS packages for resource-constrained systems

## Compatibility Notes

- ROS 2 Humble Hawksbill requires Ubuntu 22.04 or equivalent
- Some hardware drivers may require additional configuration
- Always check hardware compatibility with the ROS distribution before purchasing

## Recommended Next Steps

After completing this setup:
1. Complete the ROS 2 tutorials to familiarize yourself with the system
2. Test communication between your development machine and any robot hardware
3. Proceed to Lab Exercise 1 in the ROS 2 module