# Week 6: Unity Integration and Advanced Sensors

In this week, we'll explore Unity as a simulation platform for robotics and learn about advanced sensor simulation techniques. Unity provides high-fidelity rendering and a rich ecosystem of tools that complement the physics-accurate simulation offered by Gazebo.

## Learning Objectives

By the end of this week, you will be able to:

- Set up Unity for robotics simulation scenarios
- Implement advanced sensor models for cameras and LiDAR
- Compare Unity and Gazebo simulation outputs
- Design photo-realistic environments for perception tasks
- Validate robot perception systems using Unity simulation
- Transfer simulation results between platforms

## Introduction to Unity for Robotics

Unity is a powerful game engine that provides high-fidelity visuals and a rich development environment. While originally designed for game development, Unity has become a valuable tool for robotics simulation, particularly for perception tasks requiring photo-realistic rendering.

### Unity Robotics Tools

Unity provides several tools specifically for robotics:

- **Unity Robotics Hub**: Centralized package management for robotics tools
- **Unity Perception Package**: Tools for generating synthetic data for training AI models
- **ROS#**: C# bridge for ROS communication
- **ML-Agents**: Framework for training intelligent agents using deep reinforcement learning

## Setting Up Unity for Robotics

![Unity Perception Pipeline](/img/unity-perception-pipeline.png)

### Installing Unity Robotics Packages

1. Install Unity Hub and a recent version of Unity (2021.3 LTS or newer)
2. Install the Unity Robotics Hub from the Unity Asset Store
3. Add the ROS# package for ROS communication
4. Include the Perception package for synthetic data generation

### Basic Unity ROS Integration

Here's a basic Unity script to interface with ROS:

```csharp
using System.Collections;
using UnityEngine;
using RosSharp;
using RosSharp.Messages.Geometry;

public class RobotController : MonoBehaviour
{
    [SerializeField] private string topicName = "robot_velocity";
    private RosSocket rosSocket;
    private Subscriber<Velocity> velocitySubscriber;

    void Start()
    {
        // Initialize connection to ROS
        rosSocket = new RosSocket(new RosSharp.Communication.Uri("ws://127.0.0.1:9090"));
        velocitySubscriber = rosSocket.Subscribe<Velocity>(topicName, VelocityReceived);
    }

    private void VelocityReceived(Velocity velocity)
    {
        // Apply received velocity to the robot
        transform.Translate(new Vector3(velocity.linear.x, 0, velocity.linear.z) * Time.deltaTime);
    }
}
```

## Advanced Sensor Simulation in Unity

### Camera Sensor Simulation

Unity's rendering engine allows for advanced camera simulation with:

- **Physical Camera Properties**: Matching real-world camera specifications like focal length, aperture, etc.
- **Lens Distortion**: Simulation of lens distortion effects
- **Lighting Effects**: Realistic response to varying lighting conditions
- **Synthetic Data Generation**: Creation of labeled datasets for perception training

### LiDAR Simulation

Unity Perception provides LiDAR simulation capabilities:

- **Raycasting**: Physically accurate ray casting to simulate LiDAR measurements
- **Noise Simulation**: Addition of realistic noise patterns to sensor data
- **Dynamic Obstacles**: Real-time response to moving objects in the scene
- **Multiple Return Simulations**: Modeling of multi-return LiDAR sensors

```csharp
using UnityEngine;
using Unity.Perception.GroundTruth;

public class LidarSimulator : MonoBehaviour
{
    [Range(10, 360)]
    public int verticalResolution = 64;
    [Range(10, 2000)]
    public int horizontalResolution = 1000;
    [Range(10f, 300f)]
    public float range = 100f;

    void Start()
    {
        var lidarSensor = GetComponent<LidarSensor>();
        lidarSensor.SetParameter(LidarParameters.VerticalResolution, verticalResolution);
        lidarSensor.SetParameter(LidarParameters.HorizontalResolution, horizontalResolution);
        lidarSensor.SetParameter(LidarParameters.Range, range);
    }
}
```

## Unity vs Gazebo: When to Use Each Platform

### Choose Unity When:

- Perception tasks require photorealistic rendering
- Generating training data for computer vision models
- Developing augmented or mixed reality interfaces
- Need sophisticated visual environments
- Creating training simulations for human operators

### Choose Gazebo When:

- Accurate physics simulation is critical
- Testing robot dynamics and control algorithms
- Working with standard ROS/ROS 2 robot models
- Need realistic sensor simulation (LiDAR, IMU, cameras)
- Validating control algorithms prior to real-world deployment

## Photo-Realistic Environment Design

### Creating Realistic Lighting

Unity's lighting system allows for realistic environment rendering:

- **Directional Lights**: Simulating sunlight at different times of day
- **Reflection Probes**: Capturing environmental reflections for realistic surfaces
- **Light Probes**: Interpolating lighting information for moving objects
- **Real-time Global Illumination**: Simulating light bouncing between surfaces

### Material Design for Robotics Simulation

Creating realistic materials for robotics applications:

- **Physically-Based Materials**: Using Unity's Standard Shader for accurate material responses
- **Surface Detail**: Adding micro-details like scratches, dust, and wear patterns
- **Texture Mapping**: Using high-resolution textures for realistic surface appearance
- **Normal Maps**: Simulating surface details without increasing geometry complexity

### Synthetic Data Generation

Unity's Perception Package enables synthetic data generation:

- **Semantic Segmentation**: Automatic labeling of all objects in the scene
- **Instance Segmentation**: Individual labeling of each object instance
- **Depth Information**: Accurate depth measurement for every pixel
- **Bounding Boxes**: Automatic 2D and 3D bounding box annotation
- **Optical Flow**: Pixel-level motion tracking between frames

Sample configuration for synthetic data generation:

```json
{
  "cameras": [
    {
      "name": "MainCamera",
      "captureRgb": true,
      "captureDepth": true,
      "captureSegmentation": true,
      "captureOpticalFlow": true,
      "captureBoundingBox2D": true,
      "captureBoundingBox3D": true,
      "recording": {
        "outputDir": "/path/to/output",
        "framerate": 30,
        "frameskip": 0
      }
    }
  ]
}
```

## Perception Algorithm Validation

### Pre-training Validation

Before training perception algorithms on real data:

- Test algorithms in synthetic environments with ground truth
- Validate detection and tracking algorithms with known test cases
- Establish baseline performance metrics
- Identify potential failure modes in simulation

### Sim-to-Real Transfer

Transferring algorithms from simulation to real-world robotics:

- Domain randomization to increase algorithm robustness
- Synthetic-to-real adaptation techniques
- Validation of physics models against real robot behavior
- Sensor characteristic matching between simulation and reality

## Integration with ROS/ROS 2

Unity supports ROS/ROS 2 integration through several tools:

### ROS# (ROS Sharp)

A C# package that enables direct communication with ROS:

- Publish and subscribe to ROS topics
- Call ROS services
- Use ROS transforms and coordinate frames
- Send and receive standard ROS message types

### Unity Bridge for ROS 2

More recent integration solutions for ROS 2:

- Real-time bidirectional communication
- Support for newer ROS 2 message types
- Better performance for high-frequency messaging
- Integration with ROS 2 launch files

## Practical Exercise: Unity Perception Pipeline

### Objective

Create a Unity scene that simulates a perception task and generates synthetic training data.

### Steps

1. Create a new Unity 3D project
2. Import Unity Robotics Hub and Perception Package
3. Set up a camera with realistic parameters
4. Create a scene with various objects for detection
5. Configure the Perception Package to annotate the scene
6. Generate synthetic datasets for training a computer vision model

### Scene Creation

Create a scene with:

- A moving vehicle platform (simulated robot)
- Various static objects (trees, buildings, signs)
- Moving objects (pedestrians, other vehicles)
- Changing lighting conditions
- Different weather scenarios

### Data Annotation

Configure the Perception Package to capture:

- RGB images
- Depth maps
- Semantic segmentation masks
- Instance segmentation masks
- Bounding boxes for all objects
- 3D bounding boxes for objects

## Comparing Simulation Results

### Quantitative Metrics

When comparing Unity and Gazebo results:

- Sensor accuracy: Compare simulated sensor readings to real-world values
- Physics fidelity: Validate motion predictions against real-world behavior
- Computational performance: Benchmark simulation speed and resource usage
- Perceptual quality: Evaluate how well each platform supports perception tasks

### Qualitative Assessment

Subjective evaluation of simulation quality:

- Visual realism for perception tasks
- Physics accuracy for control validation
- Ease of use for scenario creation
- Integration capabilities with robotics frameworks

## Lab Exercise: Unity-Gazebo Comparison

### Setup

Compare a simple navigation task implemented in both Unity and Gazebo:

1. Create a similar environment in both simulators
2. Implement the same robot model with identical sensors
3. Run a basic navigation algorithm in both platforms
4. Record sensor readings, control commands, and robot trajectories

### Analysis

Compare the results across:

- Trajectory similarity
- Control signal differences
- Sensor reading variations
- Computational performance

Document findings about when Unity might be preferred over Gazebo or vice versa.

## Assessment

Complete the [Unity-Gazebo Comparison Quiz](./assessments/quiz1.md) to test your understanding of when to use each simulation platform.

## Navigation

[← Previous: Week 4-5: Simulation Environments (Gazebo)](./week4-5.md) | [Next: Module Conclusion](./conclusion.md) | [Module Home](./intro.md)

Continue with [Module Conclusion](./conclusion.md) to review the Gazebo/Unity simulation concepts.