---
title: Week 6 - Unity Integration and Advanced Sensors
sidebar_position: 7
---

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

    void OnApplicationQuit()
    {
        rosSocket.Close();
    }
}
```

## Advanced Sensor Simulation in Unity

### Camera Sensors

The Unity Perception package provides realistic camera simulation with several parameters:

```csharp
using Unity.Robotics.Perception;
using Unity.Robotics.Perception.GroundTruth;

public class CameraSensorSetup : MonoBehaviour
{
    [SerializeField] private Sensor sensor;

    void Start()
    {
        var cameraSensor = sensor.GetComponent<CameraSensor>();

        // Configure camera properties
        cameraSensor.Camera.fieldOfView = 60f;  // Field of view in degrees
        cameraSensor.Camera.nearClipPlane = 0.1f;
        cameraSensor.Camera.farClipPlane = 100f;

        // Enable depth rendering for depth camera
        cameraSensor.EnableSegmentation = true;
        cameraSensor.EnableOpticalFlow = true;
    }
}
```

### LiDAR Simulation

Unity provides plugins for simulating LiDAR sensors with realistic noise models:

```csharp
using Unity.Robotics.Perception.LiDAR;

public class LiDARSensorSetup : MonoBehaviour
{
    [SerializeField] private LiDARManager liDARManager;

    void Start()
    {
        // Configure LiDAR properties
        liDARManager.Range = 20.0f;  // Max detection range in meters
        liDARManager.AngleResolution = 0.25f;  // Angular resolution in degrees
        liDARManager.VerticalAngleResolution = 0.5f;
        liDARManager.VerticalAngleRange = new Vector2(-15.0f, 15.0f);

        // Add noise model
        liDARManager.NoiseModel = new GaussianNoiseModel(0.01f, 0.001f);
    }
}
```

### Advanced Unity Perception Features

Here's an example of how to generate synthetic training data for computer vision models:

```csharp
using Unity.Robotics.Perception;
using Unity.Robotics.Perception.GroundTruth;
using UnityEngine;

public class SyntheticDataGenerator : MonoBehaviour
{
    [Header("Capture Settings")]
    [Range(0.1f, 5f)] public float captureInterval = 1f;
    public string outputDirectory = "SyntheticData";
    public bool captureRGB = true;
    public bool captureDepth = true;
    public bool captureSegmentation = true;
    public bool captureBoundingBoxes = true;

    private float lastCaptureTime;

    void Start()
    {
        lastCaptureTime = Time.time;

        // Configure the Perception Manager
        PerceptionManager.Instance.outputDirectory = outputDirectory;
        PerceptionManager.Instance.annotationCaptureRgb = captureRGB;
        PerceptionManager.Instance.annotationCaptureDepth = captureDepth;
        PerceptionManager.Instance.annotationCaptureSegmentation = captureSegmentation;
        PerceptionManager.Instance.annotationCaptureBoundingBox2D = captureBoundingBoxes;
    }

    void Update()
    {
        if (Time.time - lastCaptureTime >= captureInterval)
        {
            // Add environmental variations
            AddEnvironmentalVariations();

            // Capture a sample with annotations
            PerceptionManager.Instance.CaptureSample();

            lastCaptureTime = Time.time;
        }
    }

    void AddEnvironmentalVariations()
    {
        // Randomize lighting
        var lights = FindObjectsOfType<Light>();
        foreach (var light in lights)
        {
            // Add random variations to light properties
            light.intensity = Mathf.Clamp(light.intensity + Random.Range(-0.1f, 0.1f), 0.5f, 1.5f);
        }

        // Apply random material changes
        var renderers = FindObjectsOfType<Renderer>();
        foreach (var renderer in renderers)
        {
            if (renderer.material.HasProperty("_Color"))
            {
                var originalColor = renderer.material.color;
                var variation = new Color(
                    Mathf.Clamp01(originalColor.r + Random.Range(-0.05f, 0.05f)),
                    Mathf.Clamp01(originalColor.g + Random.Range(-0.05f, 0.05f)),
                    Mathf.Clamp01(originalColor.b + Random.Range(-0.05f, 0.05f)),
                    originalColor.a
                );
                renderer.material.color = variation;
            }
        }
    }
}
```

## Unity vs Gazebo Comparison

| Feature | Gazebo | Unity |
|--------|---------|-------|
| Physics Accuracy | High | Medium-High |
| Visual Fidelity | Medium | High |
| Sensor Simulation | Excellent | Excellent |
| Development Speed | Fast | Medium |
| Rendering Quality | Basic | Photorealistic |
| Ecosystem | Robotics-focused | General-purpose |

## Creating Photo-Realistic Environments

Unity excels in creating visually realistic environments for perception tasks:

### Environment Design Principles

1. **Scene Complexity**: Balance visual quality with simulation performance
2. **Lighting**: Use realistic lighting to match target environments
3. **Materials**: Use physically-based materials for accurate rendering
4. **Assets**: Use high-quality models that match real-world objects
5. **Weather**: Include weather variations for comprehensive testing

### Sample Environment Setup

```csharp
using UnityEngine;

public class EnvironmentSetup : MonoBehaviour
{
    public Light sunLight;
    public GameObject[] environmentObjects;
    
    [System.Serializable]
    public class WeatherSettings
    {
        public float fogDensity = 0.0f;
        public Color skyColor = Color.blue;
        public float windSpeed = 0.0f;
    }
    
    public WeatherSettings currentWeather;
    
    void Start()
    {
        // Configure environmental settings
        RenderSettings.fog = true;
        RenderSettings.fogDensity = currentWeather.fogDensity;
        RenderSettings.ambientSkyColor = currentWeather.skyColor;
        
        // Add wind zones for outdoor simulation
        foreach (var obj in environmentObjects)
        {
            if (obj.CompareTag("WindZone"))
            {
                var windZone = obj.GetComponent<WindZone>();
                windZone.windMain = currentWeather.windSpeed;
            }
        }
    }
}
```

## Cross-Platform Validation

Validating simulation results across platforms ensures robustness:

1. **Consistent Metrics**: Use the same evaluation metrics in both Gazebo and Unity
2. **Shared Scenarios**: Create similar test scenarios in both environments
3. **Parameter Mapping**: Ensure physical parameters are consistent
4. **Data Comparison**: Analyze performance differences between platforms

## Practical Applications in Industry

Unity simulation is used in:

- Autonomous vehicle perception training
- Industrial robot path planning
- Augmented reality applications
- Training AI models with synthetic data
- Human-robot interaction studies

## Lab Exercise Preview

In the next section, you'll find the detailed instructions for the advanced Gazebo/Unity lab, where you'll implement a complete perception pipeline using both simulation platforms.

## Summary

In this week, you've learned:

- How to set up Unity for robotics simulation
- How to implement advanced sensor models in Unity
- How to design photo-realistic environments for perception tasks
- How to compare simulation results across platforms
- The applications of Unity in robotics

## Navigation

[← Previous: Week 4-5: Simulation Environments](./week4-5.md) | [Next: Gazebo/Unity Module Conclusion](./conclusion.md) | [Module Home](./intro.md)

Continue to [Gazebo/Unity Module Conclusion](./conclusion.md) to review what you've learned and how it connects to the next modules.