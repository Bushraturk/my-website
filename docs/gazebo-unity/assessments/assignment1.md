---
title: Assignment 1 - Multi-Platform Simulation Validation
sidebar_position: 11
---

# Assignment 1: Multi-Platform Simulation Validation

## Instructions

This assignment requires you to implement and compare simulation scenarios in both Gazebo and Unity, then validate the results against each other. You will create equivalent environments in both platforms and analyze the differences in simulation output.

## Learning Objectives

Upon completion of this assignment, you will be able to:
- Create equivalent simulation scenarios in both Gazebo and Unity
- Design experiments to validate simulation results across platforms
- Analyze differences between physics and sensor simulation in different engines
- Evaluate the transferability of simulation results between platforms
- Document findings and limitations of each simulation platform

## Assignment Requirements

### Part 1: Gazebo Simulation Environment (30 points)

Create a Gazebo simulation environment with:
1. A simple mobile robot (differential drive)
2. At least 3 obstacles in the environment
3. A LiDAR sensor for obstacle detection
4. A camera sensor
5. A navigation task where the robot must reach a goal while avoiding obstacles
6. Proper URDF model for the robot with correct inertial properties

### Part 2: Unity Simulation Environment (30 points)

Create an equivalent Unity simulation with:
1. A 3D model representing the same robot
2. The same arrangement of obstacles
3. Equivalent sensors (LiDAR and camera)
4. The same navigation task
5. Photo-realistic rendering appropriate for perception tasks

### Part 3: Comparison and Analysis (30 points)

1. Run the same navigation task in both environments
2. Collect metrics such as:
   - Time to reach goal
   - Path efficiency (distance traveled vs. straight-line distance)
   - Number of collisions
   - Sensor data quality
3. Analyze the differences in simulation output
4. Discuss the advantages and limitations of each platform

### Part 4: Documentation and Report (10 points)

Provide clear documentation including:
- Setup instructions for both simulations
- Screenshots of both environments
- Analysis of results with tables/charts
- Recommendations for when to use each platform

## Implementation Guidelines

1. Use appropriate coordinate frames in both environments
2. Maintain equivalent physical properties where possible
3. Use realistic sensor parameters
4. Include proper error handling in your implementations
5. Consider computational requirements for both platforms

## Example Structure for Gazebo

```
gazebo_simulation/
├── models/
│   ├── robot/
│   │   ├── meshes/
│   │   ├── materials/
│   │   └── model.urdf
│   └── world/
│       └── environment.sdf
├── launch/
│   └── simulation.launch.py
└── scripts/
    └── navigation_node.py
```

## Example Structure for Unity

```
UnitySimulation/
├── Assets/
│   ├── Scenes/
│   │   └── SimulationScene.unity
│   ├── Scripts/
│   │   ├── RobotController.cs
│   │   └── SensorManager.cs
│   ├── Models/
│   │   └── Robot.fbx
│   └── Materials/
└── Packages/
    └── manifest.json
```

## Assessment Criteria

### Part 1: Gazebo Simulation Environment
- [ ] Robot model with proper URDF (7 points)
- [ ] Environment with obstacles (6 points)
- [ ] Sensor integration (LiDAR & camera) (7 points)
- [ ] Navigation task implementation (10 points)

### Part 2: Unity Simulation Environment
- [ ] Equivalent 3D robot model (7 points)
- [ ] Environment matching Gazebo setup (6 points)
- [ ] Sensor simulation (7 points)
- [ ] Task implementation (10 points)

### Part 3: Comparison and Analysis
- [ ] Metrics collection (10 points)
- [ ] Results analysis (10 points)
- [ ] Differences discussion (10 points)

### Part 4: Documentation and Report
- [ ] Setup instructions (5 points)
- [ ] Screenshots and results (3 points)
- [ ] Platform recommendations (2 points)

## Submission Requirements

1. Submit source code for both Gazebo and Unity implementations
2. Include a comprehensive report comparing results (PDF format)
3. Provide video demonstrations of the robot completing the tasks in both environments
4. Include analysis of computational requirements for each platform

## Additional Resources

- Gazebo tutorials and documentation
- Unity Perception package documentation
- ROS 2 integration guides
- Physics simulation best practices

## Rubric

- Total points: 100
- Passing score: 70/100 (70%)
- Late submission penalty: 5% per day