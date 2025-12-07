---
title: Gazebo/Unity Hardware Setup Guide
---

# Gazebo/Unity Hardware Setup Guide

## Overview

This guide provides instructions for setting up the hardware required for the Gazebo/Unity simulation module of the Physical AI & Humanoid Robotics course. While the module primarily focuses on simulation, understanding the corresponding real-world hardware is essential for effective sim-to-real transfer.

## Equipment List

### Minimum Requirements for Simulation
- Computer with 8GB RAM (16GB recommended)
- 64-bit processor with 4+ cores
- 40GB free disk space
- Dedicated GPU with at least 4GB VRAM (recommended for Unity)
- Ubuntu 22.04 LTS or Windows 10/11

### Recommended Hardware for Sim-to-Real Validation
- Mobile robot platform (e.g., TurtleBot3, Clearpath Jackal, or equivalent)
- RGB-D camera (e.g., Intel RealSense D435, Kinect)
- 2D LiDAR (e.g., Hokuyo URG-04LX, RPLIDAR A1)
- Computing platform (NVIDIA Jetson, UP Board, or laptop with ROS 2 support)

## Software Installation

### Installing Gazebo
1. For Ubuntu 22.04 with ROS Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
   ```

2. For additional Gazebo features:
   ```bash
   sudo apt install gazebo libgazebo-dev
   ```

3. Verify installation:
   ```bash
   gazebo --version
   ```

### Installing Unity for Robotics
1. Download and install Unity Hub from unity.com
2. Install Unity 2021.3 LTS or newer
3. In Unity Hub, go to the Packages tab and install:
   - Unity Perception package
   - Robotics package
   - ROS# (for ROS communication)

### Setting up ROS 2 Bridge
1. Clone the ROS# repository:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/siemens/ros-sharp.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Real Robot Hardware Configuration

### TurtleBot3 Setup (Example)
1. Install TurtleBot3 packages:
   ```bash
   sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
   ```

2. Set environment variables:
   ```bash
   echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
   source ~/.bashrc
   ```

3. Verify real robot connection (if available):
   ```bash
   ro2 topic list | grep /camera
   ros2 topic list | grep /scan
   ```

### Sensor Hardware Setup

#### LiDAR Connection
1. Connect LiDAR to USB port
2. Check connection: `ls /dev/ttyUSB*`
3. Test with: `ros2 run hls_lfcd_lds_driver hlds_laser_publisher`

#### Camera Connection
1. Connect RGB-D camera to USB port
2. Test with appropriate ROS 2 driver:
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```

## Network Configuration for Hardware-in-the-Loop

### Setting Up ROS 2 Communication
1. Configure network settings:
   ```bash
   export ROS_DOMAIN_ID=1
   export ROS_LOCALHOST_ONLY=0
   ```

2. Test communication between simulation and hardware:
   ```bash
   ros2 topic list
   ros2 node list
   ```

## Simulation-Specific Configuration

### Optimizing Gazebo Performance
1. Adjust physics engine parameters in world files:
   ```xml
   <physics type="ode">
     <max_step_size>0.001</max_step_size>
     <real_time_factor>1.0</real_time_factor>
     <real_time_update_rate>1000.0</real_time_update_rate>
   </physics>
   ```

2. Reduce rendering quality during intensive simulations:
   - Set `OGRE_RESOURCEMANAGER_STRICT` to 0
   - Reduce GUI rendering frequency

### Unity Performance Optimization
1. Adjust Quality Settings in Unity:
   - Edit > Project Settings > Quality
   - Lower shadow resolution and anti-aliasing during rapid testing
   - Use fewer real-time lights

2. Configure Perception package for performance:
   - Reduce capture frequency during development
   - Use lower resolution for faster iteration

## Troubleshooting

### Common Gazebo Issues
1. **Gazebo crashes on startup**
   - Check graphics drivers and ensure GPU is properly configured
   - Try running with software rendering: `gazebo --verbose --lockstep`

2. **Robot models not appearing**
   - Verify model paths in Gazebo and ROS 2 environment variables
   - Check that model files are in correct directories

3. **Slow simulation performance**
   - Disable unnecessary rendering
   - Check for collision mesh complexity
   - Verify adequate system resources

### Common Unity Issues
1. **Perception package not detecting objects**
   - Ensure all objects implement ISegmentable interface
   - Check that semantic segmentation is enabled

2. **High memory usage during data capture**
   - Reduce capture frequency
   - Limit scene complexity during capture
   - Implement automatic cleanup of old data

## Compatibility Notes

- Unity 2021.3 LTS is recommended for best compatibility with Perception package
- Gazebo Fortress or Garden may have different plugin APIs than older versions
- Ensure matching ROS 2 distribution versions across all components
- Some features may require specific GPU drivers or compute capabilities

## Recommended Next Steps

After completing this setup:
1. Verify Gazebo installation with a simple empty world launch
2. Test Unity Perception with a simple scene
3. Run through the Gazebo/Unity lab exercises
4. If available, connect to real hardware to validate sim-to-real transfer