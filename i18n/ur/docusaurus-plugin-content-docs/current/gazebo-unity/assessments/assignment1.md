---
title: Gazebo/Unity Assignment 1
---

# Gazebo/Unity Assignment 1

## Description

The purpose of this assignment is to create a simple robot simulation environment in Gazebo or Unity. You will create a ROS 2 node that controls the simulated robot and connects its sensors to the simulation.

## Learning Objectives

- Create a simple robot environment in Gazebo or Unity
- Control simulation using ROS 2 nodes
- Use simulated sensors
- Read perception data through ROS 2 topics

## Requirements

- For Gazebo simulation, Ubuntu 20.04 and ROS 2 Humble
- For Unity simulation, Unity 2021.3 LTS or newer
- Unity Robotics Package (if using Unity)

## Instructions

### 1. Environment Creation (5 points)

Create an environment in Gazebo or Unity that includes:

- A ground plane
- 2-3 objects as obstacles
- A camera or LiDAR sensor

### 2. Robot Model (5 points)

Create a simple robot model with:

- A body
- 2 wheels
- A camera sensor
- A LiDAR sensor

### 3. ROS 2 Control (10 points)

Create ROS 2 nodes:

- A node that publishes commands
- A node that subsribes to sensor data
- A node that moves the robot based on camera or LiDAR data

### 4. Documentation (5 points)

- Brief description of your solution
- Which platform did you use and why?
- What challenges you faced and how you solved them?

## Grading Rubric

- 10: Simulation environment created properly
- 10: Robot model created correctly
- 10: ROS 2 control implemented successfully
- 5: Adequate documentation provided
- 5: Code quality and comments

## Submission Requirements

1. Push your code to your GitHub repository
2. Write a report detailing:
   - Your simulation setup
   - ROS 2 control implementation
   - Challenges and their solutions
   - Suggestions for future improvements

### Example Solutions

- [Gazebo completion example](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

### Deadline

This assignment is due on January 15, 2025.