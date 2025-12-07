---
title: Lab Exercise 2 - Unity Perception Pipeline Integration
sidebar_position: 9
---

# Lab Exercise 2: Unity Perception Pipeline Integration

## Objective

In this lab exercise, you will implement a perception pipeline using Unity's high-fidelity rendering to generate synthetic data for training an object detection model. You'll learn to create realistic environments, configure sensor models, and validate perception systems before real-world deployment.

## Learning Objectives

After completing this lab, you will be able to:
- Configure Unity for synthetic data generation
- Set up realistic camera sensors with appropriate parameters
- Create labeled datasets for object detection training
- Compare perception results between Unity simulation and real-world data
- Validate perception algorithms in a photo-realistic environment

## Prerequisites

- Completion of Gazebo/Unity Week 6 content
- Unity installation with Robotics packages
- Basic knowledge of computer vision concepts
- Understanding of object detection algorithms

## Equipment Required

- Computer with Unity Hub and Unity installed (2021.3 LTS or newer)
- Unity Robotics Hub and Perception package installed
- Python environment with OpenCV and related libraries
- At least 8GB RAM recommended

## Lab Steps

### Step 1: Setting Up Unity Environment

1. Create a new Unity project for robotics simulation:
   - Open Unity Hub
   - Click "New Project"
   - Select "3D (Built-in Render Pipeline)" template
   - Name the project "UnityPerceptionTutorial"
   - Make sure the render pipeline is compatible with Perception package

2. Install required packages:
   - Open the Package Manager (Window > Package Manager)
   - Install "Unity Perception" package
   - Install "Robotics" package
   - Install "ROS# (Robot Operating System)" if you plan to interface with ROS

3. Set up the scene with realistic lighting:
   - Add a Directional Light for sun simulation
   - Configure Environment Lighting with realistic settings
   - Add Reflection Probes for accurate reflections

### Step 2: Creating a Realistic Environment

1. Design a warehouse-like environment:
   - Create floor using a large plane (10x10 units)
   - Add warehouse shelving units using basic primitives
   - Create objects to detect (cubes, spheres, cylinders)
   - Add realistic materials to all objects

2. Configure environmental parameters:
   ```csharp
   using UnityEngine;
   
   public class EnvironmentManager : MonoBehaviour
   {
       [Header("Lighting Settings")]
       public Light sunLight;
       public Color ambientColor = Color.gray;
       [Range(0, 2)] public float intensity = 1.0f;
       
       [Header("Weather Settings")]
       [Range(0, 0.1f)] public float fogDensity = 0.0f;
       
       void Start()
       {
           // Configure lighting
           RenderSettings.ambientLight = ambientColor;
           sunLight.intensity = intensity;
           
           // Configure fog based on weather
           RenderSettings.fog = true;
           RenderSettings.fogDensity = fogDensity;
           RenderSettings.fogColor = Color.Lerp(Color.white, ambientColor, 0.5f);
       }
   }
   ```

### Step 3: Setting Up Sensor Simulation

1. Create a robot with a camera sensor:
   - Create an empty GameObject named "Robot"
   - Add a simple cube as the robot body
   - Position a camera at the front of the robot (e.g., at position [0.3, 0.5, 0])

2. Configure the Perception Camera:
   - Attach the "Camera Sensor" component from the Perception package to the camera
   - Set the Camera ID to "perception_cam"
   - Configure the following parameters:
     - Width: 640
     - Height: 480
     - Field of View: 60°
     - Enable Semantic Segmentation
     - Enable Bounding Box generation

3. Implement the camera setup script:
   ```csharp
   using UnityEngine;
   using Unity.Robotics.Sensors;
   using Unity.Robotics.Perception;
   using Unity.Robotics.Perception.GroundTruth;
   
   public class PerceptionCameraSetup : MonoBehaviour
   {
       [SerializeField] private Sensor sensor;
       [SerializeField] private Camera cam;
       
       [Header("Camera Properties")]
       [Range(30f, 120f)] public float fieldOfView = 60f;
       [Range(0.1f, 1000f)] public float nearClip = 0.1f;
       [Range(1f, 1000f)] public float farClip = 100f;
       
       [Header("Perception Settings")]
       public bool enableSegmentation = true;
       public bool enableBbLabeler = true;
       public bool enableOpticalFlow = false;
       
       void Start()
       {
           ConfigureCamera();
           ConfigurePerceptionComponents();
       }
       
       void ConfigureCamera()
       {
           cam.fieldOfView = fieldOfView;
           cam.nearClipPlane = nearClip;
           cam.farClipPlane = farClip;
       }
       
       void ConfigurePerceptionComponents()
       {
           if (enableSegmentation)
           {
               var segLabeler = sensor.GetComponent<SegmentationLabeler>();
               if (segLabeler == null)
                   segLabeler = sensor.gameObject.AddComponent<SegmentationLabeler>();
           }
           
           if (enableBbLabeler)
           {
               var bbLabeler = sensor.GetComponent<BoundingBoxLabeler>();
               if (bbLabeler == null)
                   bbLabeler = sensor.gameObject.AddComponent<BoundingBoxLabeler>();
           }
           
           if (enableOpticalFlow)
           {
               var optFlowLabeler = sensor.GetComponent<OpticalFlowLabeler>();
               if (optFlowLabeler == null)
                   optFlowLabeler = sensor.gameObject.AddComponent<OpticalFlowLabeler>();
           }
       }
   }
   ```

### Step 4: Adding Objects for Detection

1. Create object prefabs with semantic labels:
   - Create a cube prefab named "Box"
   - Create a sphere prefab named "Ball"
   - Create a cylinder prefab named "Can"

2. Assign semantic labels to objects:
   ```csharp
   using Unity.Robotics.Perception;
   using UnityEngine;
   
   public class ObjectSemanticLabel : MonoBehaviour, ISegmentable
   {
       [SerializeField] private string semanticLabel = "object";
       [SerializeField] private int semanticId = -1;
       
       public string GetSemanticLabel()
       {
           return semanticLabel;
       }
       
       public int GetSemanticId()
       {
           if (semanticId == -1)
           {
               // Auto assign ID if not set
               semanticId = SegmentationTagManager.Instance.GetNextAvailableId();
           }
           return semanticId;
       }
   }
   ```

3. Place objects randomly in the environment:
   ```csharp
   using UnityEngine;
   using System.Collections.Generic;
   using System.Linq;
   
   public class RandomObjectSpawner : MonoBehaviour
   {
       [Header("Object Prefabs")]
       public GameObject[] objectPrefabs;
       
       [Header("Environment Settings")]
       public Vector2 spawnArea = new Vector2(8, 8);  // Range for x and z coordinates
       [Range(0, 5)] public float minHeight = 0.3f;
       [Range(0, 5)] public float maxHeight = 2f;
       
       [Header("Spawn Settings")]
       [Range(1, 50)] public int maxObjects = 10;
       
       private List<GameObject> spawnedObjects = new List<GameObject>();
       
       void Start()
       {
           SpawnObjects();
       }
       
       public void SpawnObjects()
       {
           // Clear previous objects
           foreach (var obj in spawnedObjects)
           {
               if (obj != null) DestroyImmediate(obj);
           }
           spawnedObjects.Clear();
           
           int numToSpawn = Random.Range(3, maxObjects + 1);
           
           for (int i = 0; i < numToSpawn; i++)
           {
               SpawnSingleObject();
           }
       }
       
       void SpawnSingleObject()
       {
           if (objectPrefabs.Length == 0) return;
           
           // Select random prefab
           GameObject prefab = objectPrefabs[Random.Range(0, objectPrefabs.Length)];
           
           // Generate random position
           Vector3 pos = new Vector3(
               Random.Range(-spawnArea.x/2, spawnArea.x/2),
               Random.Range(minHeight, maxHeight),
               Random.Range(-spawnArea.y/2, spawnArea.y/2)
           );
           
           // Create object and store reference
           GameObject instance = Instantiate(prefab, pos, Quaternion.identity);
           instance.transform.SetParent(transform);
           spawnedObjects.Add(instance);
       }
   }
   ```

### Step 5: Configuring Synthetic Data Generation

1. Set up the Perception Camera Controller:
   ```csharp
   using Unity.Robotics.Perception;
   using UnityEngine;
   using System.Collections.Generic;
   
   public class PerceptionController : MonoBehaviour
   {
       [Header("Capture Settings")]
       [Range(0.1f, 2f)] public float captureInterval = 1f;
       public string outputDirectory = "PerceptionOutput";
       public int numFramesToCapture = 100;
       
       [Header("Annotation Settings")]
       public bool captureRGB = true;
       public bool captureDepth = false;
       public bool captureSegmentation = true;
       public bool captureBoundingBoxes = true;
       
       private int frameCount = 0;
       private float lastCaptureTime;
       
       void Start()
       {
           lastCaptureTime = Time.time;
           
           // Configure perception manager
           PerceptionManager.Instance.outputDirectory = outputDirectory;
           PerceptionManager.Instance.maxNumSamples = numFramesToCapture;
       }
       
       void Update()
       {
           if (frameCount >= numFramesToCapture) return;
           
           if (Time.time - lastCaptureTime >= captureInterval)
           {
               CaptureFrame();
               lastCaptureTime = Time.time;
               frameCount++;
           }
       }
       
       void CaptureFrame()
       {
           // Generate random environmental variations for more diverse data
           GenerateEnvironmentalVariations();
           
           // Trigger perception capture
           PerceptionManager.Instance.CaptureSample();
           
           Debug.Log($"Captured frame {frameCount}/{numFramesToCapture}");
       }
       
       void GenerateEnvironmentalVariations()
       {
           // Change lighting conditions
           var lights = FindObjectsByType<Light>(FindObjectsSortMode.None);
           foreach (var light in lights)
           {
               // Add slight variations to lighting
               light.intensity = Random.Range(0.8f, 1.2f);
           }
           
           // Change material colors slightly
           var renderers = FindObjectsByType<Renderer>(FindObjectsSortMode.None);
           foreach (var renderer in renderers)
           {
               if (renderer.material.HasProperty("_Color"))
               {
                   var baseColor = renderer.material.color;
                   var variation = new Color(
                       Mathf.Clamp01(baseColor.r + Random.Range(-0.1f, 0.1f)),
                       Mathf.Clamp01(baseColor.g + Random.Range(-0.1f, 0.1f)),
                       Mathf.Clamp01(baseColor.b + Random.Range(-0.1f, 0.1f)),
                       baseColor.a
                   );
                   renderer.material.color = variation;
               }
           }
       }
   }
   ```

### Step 6: Running the Simulation and Collecting Data

1. Set up the Unity scene:
   - Add the PerceptionCameraSetup script to your camera
   - Add the RandomObjectSpawner to a GameObject in your scene
   - Add the PerceptionController to manage data collection
   - Create prefabs for the objects to detect with semantic labels

2. Configure the Perception Manager:
   - In the Unity menu, go to Window > Perception > Perception Manager
   - Set up the required settings for data capture
   - Configure annotation types you want to capture

3. Run the simulation:
   - Press Play in Unity
   - The simulation will automatically capture frames with annotations
   - Data will be saved to the specified output directory

### Step 7: Analyzing Captured Data

1. The captured data will include:
   - RGB images
   - Semantic segmentation masks
   - Bounding box annotations
   - Metadata files with camera parameters

2. You can load and visualize the data using Python:
   ```python
   import cv2
   import numpy as np
   import json
   import os
   
   def visualize_annotations(rgb_path, seg_path, bboxes_path):
       # Load RGB image
       rgb_img = cv2.imread(rgb_path)
       
       # Load segmentation mask
       seg_img = cv2.imread(seg_path, cv2.IMREAD_UNCHANGED)
       
       # Load bounding box annotations
       with open(bboxes_path, 'r') as f:
           bboxes_data = json.load(f)
       
       # Draw bounding boxes on the RGB image
       for bbox in bboxes_data['captures'][0]['annotations']:
           if bbox['id'] == '2d_bounding_box':
               for box in bbox['data']:
                   # Extract bounding box coordinates
                   x_min = int(box['x'])
                   y_min = int(box['y'])
                   x_max = x_min + int(box['width'])
                   y_max = y_min + int(box['height'])
                   
                   # Draw bounding box
                   cv2.rectangle(rgb_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                   cv2.putText(rgb_img, box['label'], (x_min, y_min - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
       
       # Show the annotated image
       cv2.imshow('Annotated Image', rgb_img)
       cv2.waitKey(0)
       cv2.destroyAllWindows()
   
   # Example usage
   visualize_annotations(
       'PerceptionOutput/frame_000000.color.png',
       'PerceptionOutput/frame_000000.segmentation.png',
       'PerceptionOutput/frame_000000.json'
   )
   ```

## Expected Results

- The Unity simulation should generate diverse, photo-realistic images
- Each image should have corresponding annotations (segmentation, bounding boxes)
- The robot should navigate through various environmental conditions
- The captured data should be suitable for training computer vision models

## Troubleshooting

- If semantic segmentation isn't working, ensure all objects have the ISegmentable interface
- If bounding boxes aren't generated, verify that objects have colliders and are properly labeled
- If capture rate is too high, increase the captureInterval parameter
- If Unity crashes during capture, reduce the capture rate or scene complexity

## Extension Activities

1. Implement a dynamic scene where objects move during simulation
2. Add weather effects (rain, fog, etc.) to generate more diverse data
3. Train an object detection model using the generated synthetic data
4. Compare model performance on synthetic vs. real-world data

## Assessment Questions

1. What are the advantages of synthetic data generation over real-world data collection for perception tasks?
2. How do variations in lighting and environment affect the performance of perception systems?
3. What limitations of Unity simulation for perception tasks did you identify?

## Summary

This lab introduced you to Unity's perception capabilities for generating realistic training data for computer vision applications. You've created a complete pipeline for synthetic data generation with accurate annotations that can be used to train perception models for robotics applications.