---
title: Assignment 1 - Node Implementation and Parameters
sidebar_position: 2
---

# Assignment 1: Node Implementation and Parameters

## Instructions

This assignment requires you to implement a complete ROS 2 system with nodes, parameters, and communication patterns. You will create a robot configuration node that manages parameters and interacts with other nodes.

## Learning Objectives

Upon completion of this assignment, you will be able to:
- Implement ROS 2 nodes with parameter management
- Use launch files to coordinate multiple nodes
- Apply appropriate Quality of Service (QoS) policies
- Create a cohesive system with multiple communication patterns

## Assignment Requirements

### Part 1: Robot Configuration Node (40 points)

Create a ROS 2 node called `robot_config` that:

1. Declares the following parameters with appropriate default values:
   - `robot_name` (string, default: "default_robot")
   - `max_velocity` (double, default: 1.0)
   - `safety_distance` (double, default: 0.5)
   - `operating_mode` (string, default: "autonomous")
   - `sensor_enabled` (boolean, default: True)

2. Creates a publisher that periodically publishes robot configuration information as a custom message to a topic called `robot_config_status`. The message should contain all the parameters.

3. Implements a service server that allows external nodes to update configuration parameters. The service should be called `update_config` and use a custom service definition that accepts a parameter name and value.

4. Monitors parameter changes and logs significant changes to the console.

### Part 2: Robot Controller Node (30 points)

Create a second node called `robot_controller` that:

1. Uses parameters to configure its behavior (e.g., movement speed, control mode)

2. Subscribes to the `robot_config_status` topic to receive configuration updates

3. Implements a service client to update configuration parameters when certain conditions are met

4. Uses appropriate QoS policies for reliable communication

### Part 3: Launch File (20 points)

Create a launch file that starts both nodes with specific parameter configurations.

### Part 4: Documentation (10 points)

Provide clear documentation for:
- How to build and run your nodes
- What parameters your nodes accept
- How to interact with your system

## Implementation Guidelines

1. Use Python for implementation
2. Organize your code following ROS 2 best practices
3. Include error handling for parameter declarations and services
4. Use appropriate logging throughout your nodes
5. Follow the standard ROS 2 package structure

## Example Structure

```
robot_assignment/
├── robot_assignment/
│   ├── __init__.py
│   ├── robot_config.py
│   ├── robot_controller.py
│   └── config_message.py
├── launch/
│   └── robot_system_launch.py
├── CMakeLists.txt
├── package.xml
└── setup.py
```

## Assessment Criteria

### Part 1: Robot Configuration Node
- [ ] Parameters are correctly declared and managed (10 points)
- [ ] Publisher correctly publishes configuration status (10 points)
- [ ] Service server implements parameter updates (10 points)
- [ ] Parameter changes are logged appropriately (10 points)

### Part 2: Robot Controller Node
- [ ] Node uses parameters correctly (10 points)
- [ ] Subscription to config status works (10 points)
- [ ] Service client for updates functions (10 points)

### Part 3: Launch File
- [ ] Launch file correctly starts both nodes (10 points)
- [ ] Parameters are properly configured (10 points)

### Part 4: Documentation
- [ ] Clear build and run instructions (5 points)
- [ ] Parameter documentation (5 points)

## Submission Requirements

1. Submit all source code files
2. Include a README.md file with build and run instructions
3. Provide a brief explanation of your QoS policy choices
4. Document any assumptions made during implementation

## Additional Resources

- ROS 2 parameters documentation
- ROS 2 services documentation
- ROS 2 launch files documentation
- Quality of Service policies in ROS 2

## Rubric

- Total points: 100
- Passing score: 70/100 (70%)
- Late submission penalty: 5% per day