---
title: Lab Exercise 2 - Sensor Integration in Simulation
sidebar_position: 16
---

# Lab Exercise 2: Sensor Integration in Simulation

## Objective

In this lab exercise, you will integrate various sensors into your simulated robot and process the simulated sensor data. You'll learn how to configure sensors in Gazebo, subscribe to sensor data streams, and implement basic perception algorithms using simulated sensor inputs.

## Learning Objectives

After completing this lab, you will be able to:
- Configure multiple sensor types in Gazebo simulation environment
- Subscribe to and process simulated sensor data streams
- Implement basic perception algorithms using simulated data
- Compare simulated sensor data to real-world sensor characteristics
- Integrate sensor data with robot navigation and control systems

## Prerequisites

- Completion of Lab Exercise 1: Simulation Environment Setup
- Understanding of ROS 2 message types for sensor data
- Basic knowledge of sensor types (camera, LiDAR, IMU, etc.)

## Equipment Required

- Computer with Ubuntu 22.04 and ROS 2 Humble
- Gazebo simulation environment
- Completed simple robot model from Lab Exercise 1

## Lab Steps

### Step 1: Enhance Robot Model with Sensors

1. Update your robot URDF to include sensor configurations. Modify the robot URDF from Lab 1 (`urdf/simple_robot.urdf`) to add a camera and LiDAR:

   ```xml
   <?xml version="1.0"?>
   <robot name="sensor_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
     <!-- Materials -->
     <material name="blue">
       <color rgba="0.0 0.0 0.8 1.0"/>
     </material>
     <material name="green">
       <color rgba="0.0 0.8 0.0 1.0"/>
     </material>
     <material name="red">
       <color rgba="0.8 0.0 0.0 1.0"/>
     </material>

     <!-- Base Link -->
     <link name="base_link">
       <visual>
         <geometry>
           <box size="0.5 0.3 0.15"/>
         </geometry>
         <material name="green"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.5 0.3 0.15"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="5.0"/>
         <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Left Wheel -->
     <link name="left_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
       </inertial>
     </link>

     <!-- Right Wheel -->
     <link name="right_wheel">
       <visual>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.1" length="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
       </inertial>
     </link>

     <!-- Camera Link -->
     <link name="camera_link">
       <visual>
         <geometry>
           <box size="0.05 0.1 0.03"/>
         </geometry>
         <material name="blue"/>
       </visual>
       <collision>
         <geometry>
           <box size="0.05 0.1 0.03"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.1"/>
         <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
       </inertial>
     </link>

     <!-- LiDAR Link -->
     <link name="lidar_link">
       <visual>
         <geometry>
           <cylinder radius="0.05" length="0.04"/>
         </geometry>
         <material name="red"/>
       </visual>
       <collision>
         <geometry>
           <cylinder radius="0.05" length="0.04"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.15"/>
         <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
       </inertial>
     </link>

     <!-- Joints -->
     <joint name="base_to_left_wheel" type="continuous">
       <parent link="base_link"/>
       <child link="left_wheel"/>
       <origin xyz="0.0 0.2 0.0" rpy="1.57079 0.0 0.0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <joint name="base_to_right_wheel" type="continuous">
       <parent link="base_link"/>
       <child link="right_wheel"/>
       <origin xyz="0.0 -0.2 0.0" rpy="1.57079 0.0 0.0"/>
       <axis xyz="0 0 1"/>
     </joint>

     <joint name="base_to_camera" type="fixed">
       <parent link="base_link"/>
       <child link="camera_link"/>
       <origin xyz="0.2 0.0 0.1" rpy="0 0 0"/>
     </joint>

     <joint name="base_to_lidar" type="fixed">
       <parent link="base_link"/>
       <child link="lidar_link"/>
       <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
     </joint>

     <!-- Gazebo Plugins and Sensors -->
     <gazebo reference="base_link">
       <material>Gazebo/Green</material>
       <mu1>0.2</mu1>
       <mu2>0.2</mu2>
     </gazebo>

     <gazebo reference="left_wheel">
       <material>Gazebo/Red</material>
       <mu1>1.0</mu1>
       <mu2>1.0</mu2>
     </gazebo>

     <gazebo reference="right_wheel">
       <material>Gazebo/Red</material>
       <mu1>1.0</mu1>
       <mu2>1.0</mu2>
     </gazebo>

     <!-- Camera Sensor Configuration -->
     <gazebo reference="camera_link">
       <sensor name="camera_sensor" type="camera">
         <update_rate>30</update_rate>
         <camera name="head">
           <horizontal_fov>1.3962634</horizontal_fov>
           <image>
             <width>640</width>
             <height>480</height>
             <format>R8G8B8</format>
           </image>
           <clip>
             <near>0.1</near>
             <far>100</far>
           </clip>
         </camera>
         <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
           <frame_name>camera_link</frame_name>
           <topic_name>/camera/image_raw</topic_name>
         </plugin>
       </sensor>
     </gazebo>

     <!-- LiDAR Sensor Configuration -->
     <gazebo reference="lidar_link">
       <sensor name="lidar_sensor" type="ray">
         <update_rate>10</update_rate>
         <ray>
           <scan>
             <horizontal>
               <samples>360</samples>
               <resolution>1.0</resolution>
               <min_angle>-3.14159</min_angle>
               <max_angle>3.14159</max_angle>
             </horizontal>
           </scan>
           <range>
             <min>0.1</min>
             <max>10.0</max>
             <resolution>0.01</resolution>
           </range>
         </ray>
         <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
           <ros>
             <namespace>lidar</namespace>
             <remapping>~/out:=scan</remapping>
           </ros>
           <output_type>sensor_msgs/LaserScan</output_type>
         </plugin>
       </sensor>
     </gazebo>
   </robot>
   ```

### Step 2: Create New Launch File for Sensor Robot

1. Create a new launch file (`launch/sensor_robot.launch.py`):

   ```python
   import os
   from launch import LaunchDescription
   from launch.actions import ExecuteProcess
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory


   def generate_launch_description():
       package_dir = get_package_share_directory('simple_robot_description')
       
       # Launch Gazebo with custom world
       gazebo = ExecuteProcess(
           cmd=['gz', 'sim', '-r', os.path.join(package_dir, 'launch', 'simple_world.sdf')],
           output='screen'
       )

       # Robot State Publisher node
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           parameters=[{
               'robot_description': open(os.path.join(package_dir, 'urdf', 'sensor_robot.urdf')).read()
           }]
       )

       # Spawn robot in Gazebo
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'sensor_robot',
               '-x', '0.0',
               '-y', '0.0',
               '-z', '0.15'
           ],
           output='screen'
       )

       # Camera and LiDAR processing nodes would be added in Step 6

       return LaunchDescription([
           gazebo,
           robot_state_publisher,
           spawn_entity
       ])
   ```

### Step 3: Verify Sensor Data Streams

1. Build your updated package:
   ```bash
   cd ~/gazebo_lab_ws
   colcon build --packages-select simple_robot_description
   source install/setup.bash
   ```

2. Launch the updated robot:
   ```bash
   ros2 launch simple_robot_description sensor_robot.launch.py
   ```

3. In a new terminal, verify that sensor topics are being published:
   ```bash
   # Check all topics
   ros2 topic list
   
   # Verify camera data
   ros2 topic echo /camera/image_raw --field data
   
   # Verify LiDAR data
   ros2 topic echo /scan --field ranges
   ```

### Step 4: Create Basic Perception Node

1. Create a perception processing node (`scripts/perception_node.py`):

   ```python
   #!/usr/bin/env python3
   
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, LaserScan
   from cv_bridge import CvBridge
   import numpy as np
   import cv2
   from std_msgs.msg import Float32
   from geometry_msgs.msg import Twist


   class PerceptionNode(Node):
       def __init__(self):
           super().__init__('perception_node')
           
           # Initialize CvBridge for image processing
           self.bridge = CvBridge()
           
           # Create subscriptions
           self.camera_subscriber = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.camera_callback,
               10
           )
           
           self.lidar_subscriber = self.create_subscription(
               LaserScan,
               '/scan',
               self.lidar_callback,
               10
           )
           
           # Create publishers for processed data
           self.object_distance_publisher = self.create_publisher(
               Float32,
               '/object_distance',
               10
           )
           
           self.cmd_vel_publisher = self.create_publisher(
               Twist,
               '/cmd_vel',
               10
           )
           
           # State variables
           self.latest_image = None
           self.latest_scan = None
           self.object_detected = False
           self.object_distance = Float32()
           self.object_distance.data = float('inf')
           
           self.get_logger().info("Perception Node Initialized")

       def camera_callback(self, msg):
           """Process incoming camera images"""
           try:
               # Convert ROS Image message to OpenCV image
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
               self.latest_image = cv_image
               
               # Simple color-based object detection (detecting red objects)
               hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
               
               # Define range for red color
               lower_red = np.array([0, 50, 50])
               upper_red = np.array([10, 255, 255])
               mask1 = cv2.inRange(hsv, lower_red, upper_red)
               
               lower_red = np.array([170, 50, 50])
               upper_red = np.array([180, 255, 255])
               mask2 = cv2.inRange(hsv, lower_red, upper_red)
               
               mask = mask1 + mask2
               
               # Apply morphological operations to reduce noise
               kernel = np.ones((5,5), np.uint8)
               mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
               mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
               
               # Find contours
               contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
               
               if contours:
                   # Find the largest contour
                   largest_contour = max(contours, key=cv2.contourArea)
                   if cv2.contourArea(largest_contour) > 100:  # Minimum area threshold
                       self.object_detected = True
                       self.get_logger().info("Red object detected!")
                   else:
                       self.object_detected = False
               else:
                   self.object_detected = False
                   
           except Exception as e:
               self.get_logger().error(f"Error processing camera image: {e}")

       def lidar_callback(self, msg):
           """Process incoming LiDAR scan data"""
           try:
               # Find closest obstacle in front of robot (within 30 degrees)
               front_scan = msg.ranges[:15] + msg.ranges[-15:]  # Approximate front 30 degrees
               
               # Filter out invalid measurements (inf, nan)
               valid_distances = [d for d in front_scan if d != float('inf') and not np.isnan(d)]
               
               if valid_distances:
                   min_distance = min(valid_distances)
                   self.object_distance.data = min_distance
                   
                   # Publish the closest distance
                   self.object_distance_publisher.publish(self.object_distance)
                   
                   self.get_logger().info(f"Closest obstacle: {min_distance:.2f}m")
               else:
                   self.object_distance.data = float('inf')
                   
           except Exception as e:
               self.get_logger().error(f"Error processing LiDAR data: {e}")

       def run_navigation_logic(self):
           """Simple navigation based on sensor inputs"""
           cmd_msg = Twist()
           
           # If we detect an object both visually and with LiDAR, stop
           if self.object_detected and self.object_distance.data < 1.0:
               cmd_msg.linear.x = 0.0
               cmd_msg.angular.z = 0.0
               self.get_logger().info("Stopping: obstacle detected!")
           elif self.object_distance.data < 0.5:
               # Too close to obstacle, stop and turn
               cmd_msg.linear.x = 0.0
               cmd_msg.angular.z = 0.5  # Turn right
           else:
               # Go forward if no obstacles
               cmd_msg.linear.x = 0.3
               cmd_msg.angular.z = 0.0
               
           # Publish command
           self.cmd_vel_publisher.publish(cmd_msg)


   def main(args=None):
       rclpy.init(args=args)
       perception_node = PerceptionNode()
       
       # Create a timer to run navigation logic at 10Hz
       timer = perception_node.create_timer(0.1, perception_node.run_navigation_logic)
       
       try:
           rclpy.spin(perception_node)
       except KeyboardInterrupt:
           perception_node.get_logger().info("Shutting down perception node...")
       finally:
           # Stop the robot when shutting down
           cmd_stop = Twist()
           cmd_stop.linear.x = 0.0
           cmd_stop.angular.z = 0.0
           perception_node.cmd_vel_publisher.publish(cmd_stop)
           perception_node.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 5: Update the Launch File to Include Perception Node

1. Modify the launch file to include the perception node:

   ```python
   # Updated version of launch/sensor_robot.launch.py
   import os
   from launch import LaunchDescription
   from launch.actions import ExecuteProcess, RegisterEventHandler
   from launch.event_handlers import OnProcessStart
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory


   def generate_launch_description():
       package_dir = get_package_share_directory('simple_robot_description')
       
       # Launch Gazebo with custom world
       gazebo = ExecuteProcess(
           cmd=['gz', 'sim', '-r', os.path.join(package_dir, 'launch', 'simple_world.sdf')],
           output='screen'
       )

       # Robot State Publisher node
       robot_state_publisher = Node(
           package='robot_state_publisher',
           executable='robot_state_publisher',
           name='robot_state_publisher',
           parameters=[{
               'robot_description': open(os.path.join(package_dir, 'urdf', 'sensor_robot.urdf')).read()
           }]
       )

       # Spawn robot in Gazebo
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-topic', 'robot_description',
               '-entity', 'sensor_robot',
               '-x', '0.0',
               '-y', '0.0',
               '-z', '0.15'
           ],
           output='screen'
       )

       # Perception node
       perception_node = Node(
           package='simple_robot_description',
           executable='perception_node',
           name='perception_node',
           output='screen'
       )

       # Create launch description
       ld = LaunchDescription()
       
       # Add the primary nodes
       ld.add_action(gazebo)
       ld.add_action(robot_state_publisher)
       ld.add_action(spawn_entity)
       
       # Add perception node after spawn is complete
       ld.add_action(
           RegisterEventHandler(
               OnProcessStart(
                   target_action=spawn_entity,
                   on_start=[perception_node],
               )
           )
       )
       
       return ld
   ```

### Step 6: Create Package Configuration

1. Create a proper `package.xml` file for your robot description package:

   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>simple_robot_description</name>
     <version>0.0.1</version>
     <description>Simple robot model with sensors for simulation lab</description>
     <maintainer email="student@university.edu">Student</maintainer>
     <license>Apache-2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>gazebo_ros_pkgs</depend>
     <depend>robot_state_publisher</depend>
     <depend>geometry_msgs</depend>
     <depend>sensor_msgs</depend>
     <depend>cv_bridge</depend>
     <depend>rclpy</depend>

     <exec_depend>ros_gz</exec_depend>
     <exec_depend>gazebo_ros</exec_depend>

     <export>
       <build_type>ament_python</build_type>
     </export>
   </package>
   ```

2. Create a `setup.py` file:

   ```python
   from setuptools import setup
   import os
   from glob import glob

   package_name = 'simple_robot_description'

   setup(
       name=package_name,
       version='0.0.1',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
           (os.path.join('share', package_name, 'launch'), glob('launch/*')),
           (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Student',
       maintainer_email='student@university.edu',
       description='Simple robot model with sensors for simulation lab',
       license='Apache-2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'perception_node = simple_robot_description.perception_node:main',
               'move_robot = simple_robot_description.move_robot:main',
           ],
       },
   )
   ```

### Step 7: Test Integrated Perception System

1. Make sure your scripts are executable:
   ```bash
   chmod +x ~/gazebo_lab_ws/src/simple_robot_description/scripts/perception_node.py
   ```

2. Build and run the complete system:
   ```bash
   cd ~/gazebo_lab_ws
   colcon build --packages-select simple_robot_description
   source install/setup.bash
   ros2 launch simple_robot_description sensor_robot.launch.py
   ```

3. Monitor the perception node's output in another terminal:
   ```bash
   source ~/gazebo_lab_ws/install/setup.bash
   ros2 run simple_robot_description perception_node
   ```

4. Visualize the sensor data in RViz:
   ```bash
   source ~/gazebo_lab_ws/install/setup.bash
   ros2 run rviz2 rviz2
   # Add displays for camera feed and LiDAR scan
   ```

### Step 8: Advanced Perception Task

1. Implement a simple object tracking algorithm by enhancing the perception node to track the detected object:

   ```python
   # Additional method to add to perception_node.py
   
   def detect_and_track_object(self, cv_image):
       """Detect and track colored objects in the image"""
       hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
       
       # Define range for blue color (you can try detecting different colors)
       lower_blue = np.array([100, 50, 50])
       upper_blue = np.array([130, 255, 255])
       mask = cv2.inRange(hsv, lower_blue, upper_blue)
       
       # Apply morphological operations
       kernel = np.ones((5,5), np.uint8)
       mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
       mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
       
       # Find contours
       contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
       
       if contours:
           # Find the largest contour
           largest_contour = max(contours, key=cv2.contourArea)
           if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
               # Get the center of the contour
               M = cv2.moments(largest_contour)
               if M["m00"] != 0:
                   cx = int(M["m10"] / M["m00"])
                   cy = int(M["m01"] / M["m00"])
                   
                   # Calculate the horizontal position relative to image center
                   img_center_x = cv_image.shape[1] / 2
                   horizontal_error = cx - img_center_x
                   
                   # Normalize to -1 (left) to 1 (right)
                   normalized_position = horizontal_error / (cv_image.shape[1] / 2)
                   
                   return True, normalized_position
       
       return False, 0.0
   ```

## Lab Report

Submit a lab report including:

1. **Sensor Integration**: Documentation of how you added sensors to your robot model
2. **Perception Implementation**: Explanation of your perception algorithms and how they process sensor data
3. **System Integration**: Description of how sensor data influences robot behavior
4. **Results**: Analysis of your robot's performance in detecting and responding to obstacles
5. **Comparison**: Differences between simulated and real-world sensor behavior
6. **Challenges**: Technical obstacles encountered and solutions implemented

## Troubleshooting

### Common Issues and Solutions

1. **Sensor Topics Not Publishing**
   - Problem: Camera or LiDAR topics don't appear in `ros2 topic list`
   - Solution: Check URDF sensor plugin configuration; verify Gazebo simulation is running

2. **Image Processing Errors**
   - Problem: CvBridge conversion errors or incorrect image formats
   - Solution: Verify image message format and adjust CvBridge conversion parameters

3. **LiDAR Data Inconsistencies**
   - Problem: Unexpected ranges or missing data points
   - Solution: Check LiDAR plugin configuration and update rates

4. **Robot Navigation Issues**
   - Problem: Robot not responding to sensor-based navigation
   - Solution: Verify topic remapping and message types match expectations

## Performance Evaluation

### Key Metrics
- **Detection Accuracy**: Percentage of obstacles correctly detected
- **Response Time**: Time from detection to robot response
- **Navigation Success**: Success rate of obstacle avoidance behavior
- **Computational Efficiency**: CPU utilization of perception algorithms

## Extension Activities

For advanced learners, consider:

1. **SLAM Implementation**: Use sensor data for mapping and localization
2. **Object Classification**: Implement deep learning models for object recognition
3. **Multi-Sensor Fusion**: Combine data from multiple sensors for better perception
4. **Real Robot Comparison**: Compare simulation results with real robot sensor data

## Next Steps

In the next module, you'll learn about NVIDIA Isaac perception systems that build on these fundamental sensor integration concepts with AI-powered perception algorithms.