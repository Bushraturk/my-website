---
title: Lab Exercise 2 - SLAM Implementation with NVIDIA Isaac
sidebar_position: 20
---

# Lab Exercise 2: SLAM Implementation with NVIDIA Isaac

## Objective

In this lab exercise, you will implement Simultaneous Localization and Mapping (SLAM) using NVIDIA Isaac ROS packages. You'll learn to create maps of unknown environments while simultaneously tracking the robot's position within those maps, using GPU-accelerated algorithms optimized for NVIDIA hardware.

## Learning Objectives

After completing this lab, you will be able to:
- Configure and deploy Isaac ROS SLAM packages
- Implement GPU-accelerated mapping and localization algorithms
- Integrate SLAM systems with perception and navigation pipelines
- Evaluate SLAM performance metrics such as accuracy and computational efficiency
- Create and utilize maps for robot navigation in Isaac environments

## Prerequisites

- Completion of ROS 2 modules (Weeks 1-3)
- Experience with Gazebo/Unity simulation (Weeks 4-6) 
- Understanding of perception algorithms (NVIDIA Isaac Week 7-8)
- Knowledge of robot kinematics and coordinate frames

## Equipment Required

- Computer with NVIDIA GPU (RTX 3070 or equivalent)
- Ubuntu 22.04 with ROS 2 Humble
- Isaac ROS packages installed
- Previous Isaac perception lab completed
- Gazebo simulation environment

## Lab Steps

### Step 1: System Verification and SLAM Package Setup

1. Verify Isaac SLAM packages availability:
   ```bash
   # Check for Isaac SLAM packages
   ros2 pkg list | grep -i slam
   
   # Specifically look for Isaac packages
   ros2 pkg list | grep isaac_ros
   # Should include packages like:
   # - isaac_ros_visual_slam
   # - isaac_ros_occupancy_grid_localizer
   # - etc.
   ```

2. Set up your SLAM workspace:
   ```bash
   mkdir -p ~/isaac_slam_ws/src
   cd ~/isaac_slam_ws/src
   
   # Create a package for SLAM examples
   ros2 pkg create --build-type ament_python isaac_slam_examples --dependencies rclpy sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2_msgs tf2_ros
   ```

### Step 2: Install Isaac SLAM Dependencies

1. Install Isaac SLAM packages (if not already installed):
   ```bash
   sudo apt update
   sudo apt install ros-humble-isaac-ros-visual-slam
   # Additional packages as needed
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

2. Verify installation:
   ```bash
   # Check that SLAM packages are accessible
   ros2 run | grep visual_slam
   ```

### Step 3: Create Isaac Visual SLAM Node

1. Create a SLAM implementation node (`isaac_slam_examples/isaac_slam_examples/slam_node.py`):

   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, CameraInfo, Imu
   from geometry_msgs.msg import Twist, TransformStamped
   from nav_msgs.msg import Odometry, OccupancyGrid
   from tf2_ros import TransformBroadcaster
   from cv_bridge import CvBridge
   import numpy as np
   import cv2
   from std_msgs.msg import Header
   import tf2_ros
   import tf2_geometry_msgs
   from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
   import yaml
   import os


   class IsaacSLAMNode(Node):
       """
       Isaac ROS SLAM implementation node demonstrating 
       GPU-accelerated mapping and localization
       """
       
       def __init__(self):
           super().__init__('isaac_slam_node')
           
           # Initialize CvBridge
           self.bridge = CvBridge()
           
           # Initialize TF broadcaster
           self.tf_broadcaster = TransformBroadcaster(self)
           
           # Initialize SLAM state
           self.odom_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
           self.map_resolution = 0.05  # meters per pixel
           self.map_width = 200  # pixels
           self.map_height = 200  # pixels
           self.map_origin_x = -5.0  # meters
           self.map_origin_y = -5.0  # meters
           self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
           
           # Create subscriptions
           self.image_sub = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )
           
           self.imu_sub = self.create_subscription(
               Imu,
               '/imu/data',
               self.imu_callback,
               10
           )
           
           self.odom_sub = self.create_subscription(
               Odometry,
               '/odom',
               self.odom_callback,
               10
           )
           
           # Create publishers
           self.map_pub = self.create_publisher(
               OccupancyGrid,
               '/map',
               10
           )
           
           self.position_pub = self.create_publisher(
               PoseStamped,
               '/slam_pose',
               10
           )
           
           # Timer for map publishing
           self.map_timer = self.create_timer(1.0, self.publish_map)
           
           self.get_logger().info("Isaac SLAM Node Initialized")

       def image_callback(self, msg):
           """Process images for visual SLAM features"""
           try:
               # Convert ROS image to OpenCV
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
               
               # In a real Isaac implementation, this would use:
               # - Hardware-accelerated feature extraction
               # - GPU-accelerated matching algorithms
               # - Optimized pipeline for consistent frame-to-frame tracking
               
               # For this example, we'll simulate visual feature extraction
               gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
               
               # Detect features using ORB (in Isaac, this would be accelerated)
               orb = cv2.ORB_create(nfeatures=500)
               keypoints, descriptors = orb.detectAndCompute(gray, None)
               
               # Draw keypoints for visualization
               vis_image = cv2.drawKeypoints(cv_image, keypoints, None, color=(0, 255, 0))
               
               # In a real Isaac setup, we would use these features for:
               # 1. Visual odometry computation
               # 2. Loop closure detection
               # 3. Map point creation and refinement
               
               self.get_logger().debug(f"Detected {len(keypoints)} features")
               
           except Exception as e:
               self.get_logger().error(f"Error processing image: {e}")

       def imu_callback(self, msg):
           """Process IMU data for SLAM refinement"""
           # In Isaac SLAM, IMU data is fused with visual data
           # for more robust pose estimation
           self.get_logger().debug("Received IMU data")

       def odom_callback(self, msg):
           """Process odometry data for SLAM"""
           # Update pose estimate based on odometry
           self.odom_pose[0] = msg.pose.pose.position.x
           self.odom_pose[1] = msg.pose.pose.position.y
           
           # Convert quaternion to yaw angle
           q = msg.pose.pose.orientation
           siny_cosp = 2 * (q.w * q.z + q.x * q.y)
           cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
           self.odom_pose[2] = np.arctan2(siny_cosp, cosy_cosp)
           
           # In Isaac SLAM, this odometry would be fused with
           # visual and IMU data for optimized pose estimates
           
           self.publish_transform()
           
       def publish_transform(self):
           """Publish transform for the robot's pose"""
           t = TransformStamped()
           
           # Fill in header
           t.header.stamp = self.get_clock().now().to_msg()
           t.header.frame_id = 'map'
           t.child_frame_id = 'base_link'
           
           # Fill in transform
           t.transform.translation.x = float(self.odom_pose[0])
           t.transform.translation.y = float(self.odom_pose[1])
           t.transform.translation.z = 0.0
           
           # Convert yaw to quaternion
           from math import sin, cos
           yaw = self.odom_pose[2]
           quaternion = Quaternion()
           quaternion.x = 0.0
           quaternion.y = 0.0
           quaternion.z = sin(yaw / 2.0)
           quaternion.w = cos(yaw / 2.0)
           
           t.transform.rotation = quaternion
           
           # Send transform
           self.tf_broadcaster.sendTransform(t)

       def update_map(self, position, measurement):
           """
           Update occupancy grid map with sensor measurement
           In Isaac, this would use optimized GPU algorithms
           """
           # Convert world coordinates to map coordinates
           map_x = int((position[0] - self.map_origin_x) / self.map_resolution)
           map_y = int((position[1] - self.map_origin_y) / self.map_resolution)
           
           # Check bounds
           if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
               # Update occupancy value (simplified model)
               # 0 = free, 100 = occupied, -1 = unknown
               self.occupancy_map[map_y, map_x] = 100  # Mark as occupied

       def publish_map(self):
           """Publish the occupancy grid map"""
           try:
               # Create OccupancyGrid message
               msg = OccupancyGrid()
               msg.header.stamp = self.get_clock().now().to_msg()
               msg.header.frame_id = 'map'
               
               # Set map metadata
               msg.info.resolution = self.map_resolution
               msg.info.width = self.map_width
               msg.info.height = self.map_height
               msg.info.origin.position.x = self.map_origin_x
               msg.info.origin.position.y = self.map_origin_y
               msg.info.origin.position.z = 0.0
               msg.info.origin.orientation.w = 1.0
               
               # Flatten the occupancy map for the message
               # The map is stored in row-major order (row by row)
               flat_map = self.occupancy_map.flatten()
               msg.data = flat_map.tolist()
               
               # Publish the map
               self.map_pub.publish(msg)
               
               self.get_logger().info(f"Published map with {len(msg.data)} cells")
               
           except Exception as e:
               self.get_logger().error(f"Error publishing map: {e}")

       def save_map(self, filename_prefix="slam_map"):
           """Save the current map to file"""
           try:
               # Save map as an image file
               map_image = ((1.0 - self.occupancy_map / 100.0) * 255).astype(np.uint8)
               cv2.imwrite(f"{filename_prefix}.png", map_image)
               
               # Also save map metadata
               map_metadata = {
                   'resolution': float(self.map_resolution),
                   'origin_x': float(self.map_origin_x),
                   'origin_y': float(self.map_origin_y),
                   'width': int(self.map_width),
                   'height': int(self.map_height)
               }
               
               with open(f"{filename_prefix}.yaml", 'w') as f:
                   yaml.dump(map_metadata, f)
                   
               self.get_logger().info(f"Map saved as {filename_prefix}.png and {filename_prefix}.yaml")
               
           except Exception as e:
               self.get_logger().error(f"Error saving map: {e}")


   def main(args=None):
       rclpy.init(args=args)
       slam_node = IsaacSLAMNode()
       
       try:
           rclpy.spin(slam_node)
       except KeyboardInterrupt:
           slam_node.get_logger().info("Shutting down Isaac SLAM Node...")
           # Save the map before shutting down
           slam_node.save_map()
       finally:
           slam_node.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 4: Create Isaac VSLAM Launch File

1. Create a launch directory and file (`isaac_slam_examples/launch/isaac_slam_pipeline.launch.py`):

   ```python
   import os
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, RegisterEventHandler
   from launch.event_handlers import OnProcessStart
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node


   def generate_launch_description():
       # Launch configuration variables
       use_sim_time = LaunchConfiguration('use_sim_time', default='true')
       
       # Declare launch arguments
       declare_use_sim_time = DeclareLaunchArgument(
           'use_sim_time',
           default_value='true',
           description='Use simulation (Gazebo) clock if true'
       )
       
       # Isaac Visual SLAM node
       visual_slam_node = Node(
           package='isaac_ros_visual_slam',
           executable='visual_slam_node',  # Actual Isaac executable
           name='visual_slam',
           parameters=[
               {'use_sim_time': use_sim_time},
               {'enable_occupancy_grid': True},
               {'occupancy_grid_resolution': 0.05},
               {'map_frame': 'map'},
               {'base_frame': 'base_link'},
           ],
           remappings=[
               ('/visual_slam/camera/imu', '/imu/data'),
               ('/visual_slam/camera/image', '/camera/image_raw'),
               ('/visual_slam/camera/camera_info', '/camera/camera_info'),
           ],
           output='screen'
       )
       
       # Odometry to TF broadcaster (if needed)
       tf_broadcaster = Node(
           package='tf2_ros',
           executable='static_transform_publisher',
           name='map_odom_broadcaster',
           arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
       )
       
       # Create launch description
       ld = LaunchDescription()
       
       # Add launch arguments
       ld.add_action(declare_use_sim_time)
       
       # Add nodes
       ld.add_action(visual_slam_node)
       ld.add_action(tf_broadcaster)
       
       return ld
   ```

### Step 5: Create Comprehensive SLAM Integration

1. Create a more complete SLAM integration example (`isaac_slam_examples/scripts/comprehensive_slam.py`):

   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image, CameraInfo, Imu, LaserScan
   from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
   from nav_msgs.msg import Odometry, OccupancyGrid, Path
   from tf2_ros import TransformBroadcaster, Buffer, TransformListener
   from cv_bridge import CvBridge
   import numpy as np
   import cv2
   from std_msgs.msg import Header
   from visualization_msgs.msg import Marker, MarkerArray
   import tf2_geometry_msgs
   import math
   from threading import Lock
   import os
   import yaml


   class IsaacComprehensiveSLAM(Node):
       """
       Comprehensive Isaac SLAM implementation integrating perception,
       mapping, localization, and navigation components
       """
       
       def __init__(self):
           super().__init__('isaac_comprehensive_slam')
           
           # Initialize components
           self.bridge = CvBridge()
           self.tf_buffer = Buffer()
           self.tf_listener = TransformListener(self.tf_buffer, self)
           self.tf_broadcaster = TransformBroadcaster(self)
           self.lock = Lock()
           
           # SLAM parameters
           self.map_resolution = 0.05  # meters per cell
           self.map_width = 400  # cells
           self.map_height = 400  # cells
           self.map_origin_x = -10.0  # meters
           self.map_origin_y = -10.0  # meters
           
           # Robot pose tracking
           self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
           self.odom_pose = np.array([0.0, 0.0, 0.0])
           
           # Initialize occupancy map
           self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
           self.map_updated = False
           
           # Feature tracking
           self.previous_features = None
           self.current_features = None
           self.feature_tracked = False
           
           # Create subscriptions
           self.image_sub = self.create_subscription(
               Image,
               '/camera/image_raw',
               self.image_callback,
               10
           )
           
           self.laser_sub = self.create_subscription(
               LaserScan,
               '/scan',
               self.laser_callback,
               10
           )
           
           self.odom_sub = self.create_subscription(
               Odometry,
               '/odom',
               self.odom_callback,
               10
           )
           
           # Create publishers
           self.map_pub = self.create_publisher(
               OccupancyGrid,
               '/map',
               10
           )
           
           self.path_pub = self.create_publisher(
               Path,
               '/slam_path',
               10
           )
           
           self.pose_pub = self.create_publisher(
               PoseStamped,
               '/slam_pose',
               10
           )
           
           self.marker_pub = self.create_publisher(
               MarkerArray,
               '/slam_features',
               10
           )
           
           # Timer for publishing results
           self.slam_timer = self.create_timer(0.1, self.slam_update)
           
           # Path tracking
           self.path = Path()
           self.path.header.frame_id = 'map'
           
           self.get_logger().info("Isaac Comprehensive SLAM Initialized")

       def image_callback(self, msg):
           """Process camera images for visual SLAM"""
           try:
               cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
               
               # Convert to grayscale for feature detection
               gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
               
               # Detect features using FAST algorithm
               fast = cv2.FastFeatureDetector_create(threshold=25)
               keypoints = fast.detect(gray, None)
               
               # In a real Isaac implementation, this would be:
               # - GPU-accelerated feature extraction
               # - Hardware-optimized descriptor computation
               # - Optimized for consistent tracking across frames
               
               # Store features for tracking
               if keypoints:
                   self.current_features = np.float32([kp.pt for kp in keypoints]).reshape(-1, 1, 2)
               
               self.get_logger().debug(f"Detected {len(keypoints) if keypoints else 0} features")
               
           except Exception as e:
               self.get_logger().error(f"Error processing image: {e}")

       def laser_callback(self, msg):
           """Process laser scan data for mapping"""
           # In Isaac SLAM, laser data is used for precise mapping
           # especially when visual features are insufficient
           
           try:
               # Get robot's current position
               robot_x = self.robot_pose[0]
               robot_y = self.robot_pose[1]
               robot_theta = self.robot_pose[2]
               
               # Process laser ranges
               for i, range_val in enumerate(msg.ranges):
                   if not (math.isnan(range_val) or math.isinf(range_val)):
                       # Calculate angle of this laser point
                       angle = msg.angle_min + i * msg.angle_increment + robot_theta
                       
                       # Calculate world coordinates of obstacle
                       world_x = robot_x + range_val * math.cos(angle)
                       world_y = robot_y + range_val * math.sin(angle)
                       
                       # Update occupancy grid
                       self.update_occupancy_grid(world_x, world_y, 100)  # Mark as occupied
               
           except Exception as e:
               self.get_logger().error(f"Error processing laser data: {e}")

       def odom_callback(self, msg):
           """Process odometry data"""
           # Update odometry-based pose estimate
           self.odom_pose[0] = msg.pose.pose.position.x
           self.odom_pose[1] = msg.pose.pose.position.y
           
           # Convert quaternion to yaw
           q = msg.pose.pose.orientation
           siny_cosp = 2 * (q.w * q.z + q.x * q.y)
           cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
           self.odom_pose[2] = math.atan2(siny_cosp, cosy_cosp)
           
           # In Isaac SLAM, this would be fused with visual/IMU data
           # for improved pose estimation

       def update_occupancy_grid(self, world_x, world_y, value):
           """Update occupancy grid with new information"""
           # Convert world coordinates to map coordinates
           map_x = int((world_x - self.map_origin_x) / self.map_resolution)
           map_y = int((world_y - self.map_origin_y) / self.map_resolution)
           
           # Check bounds
           if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
               # In a real SLAM system, this would use probabilistic updates
               self.occupancy_map[map_y, map_x] = value
               self.map_updated = True

       def visual_odometry_update(self):
           """Estimate motion using visual features"""
           if self.previous_features is not None and self.current_features is not None:
               # Compute motion between previous and current features
               # This is a simplified approach - Isaac would use GPU-optimized methods
               
               # Calculate feature displacement
               if len(self.previous_features) > 10 and len(self.current_features) > 10:
                   # Compute average displacement
                   avg_disp_x = np.mean(self.current_features[:, 0, 0] - self.previous_features[:, 0, 0])
                   avg_disp_y = np.mean(self.current_features[:, 0, 1] - self.previous_features[:, 0, 1])
                   
                   # Convert to world motion (simplified)
                   # In reality, this would involve camera calibration and pose estimation
                   self.get_logger().debug(f"Visual displacement: ({avg_disp_x}, {avg_disp_y})")
                   
                   # Update robot pose based on visual motion
                   # This is where Isaac's GPU acceleration provides significant benefits
                   self.robot_pose[0] += avg_disp_x * self.map_resolution
                   self.robot_pose[1] += avg_disp_y * self.map_resolution
                   
                   self.feature_tracked = True
           
           # Update previous features
           self.previous_features = self.current_features.copy()

       def slam_update(self):
           """Main SLAM update loop"""
           with self.lock:
               # Perform visual odometry update if possible
               if self.feature_tracked and self.current_features is not None:
                   self.visual_odometry_update()
                   self.feature_tracked = False
               
               # Fused pose estimate: use odometry when vision is unreliable
               # In Isaac, this would be a sophisticated sensor fusion algorithm
               self.robot_pose = 0.7 * self.robot_pose + 0.3 * self.odom_pose
               
               # Publish current pose
               self.publish_pose()
               
               # Publish path
               self.publish_path()
               
               # Publish features visualization
               self.publish_features()
               
               # Publish map if updated
               if self.map_updated:
                   self.publish_map()
                   self.map_updated = False
               
               # Publish transform
               self.publish_transform()

       def publish_pose(self):
           """Publish the robot's current pose estimate"""
           pose_msg = PoseStamped()
           pose_msg.header.stamp = self.get_clock().now().to_msg()
           pose_msg.header.frame_id = 'map'
           pose_msg.pose.position.x = float(self.robot_pose[0])
           pose_msg.pose.position.y = float(self.robot_pose[1])
           pose_msg.pose.position.z = 0.0
           
           # Convert yaw to quaternion
           qw = math.cos(self.robot_pose[2] / 2.0)
           qz = math.sin(self.robot_pose[2] / 2.0)
           pose_msg.pose.orientation.w = qw
           pose_msg.pose.orientation.z = qz
           
           self.pose_pub.publish(pose_msg)

       def publish_path(self):
           """Publish the robot's path for visualization"""
           # Add current pose to path
           pose_stamped = PoseStamped()
           pose_stamped.header.stamp = self.get_clock().now().to_msg()
           pose_stamped.header.frame_id = 'map'
           pose_stamped.pose.position.x = float(self.robot_pose[0])
           pose_stamped.pose.position.y = float(self.robot_pose[1])
           pose_stamped.pose.position.z = 0.0
           
           qw = math.cos(self.robot_pose[2] / 2.0)
           qz = math.sin(self.robot_pose[2] / 2.0)
           pose_stamped.pose.orientation.w = qw
           pose_stamped.pose.orientation.z = qz
           
           self.path.poses.append(pose_stamped)
           self.path.header.stamp = self.get_clock().now().to_msg()
           
           self.path_pub.publish(self.path)

       def publish_features(self):
           """Publish feature points for visualization"""
           marker_array = MarkerArray()
           
           # Create markers for visualized features (if any)
           # In a real implementation, this would show tracked features
           
           self.marker_pub.publish(marker_array)

       def publish_map(self):
           """Publish the occupancy grid map"""
           map_msg = OccupancyGrid()
           map_msg.header.stamp = self.get_clock().now().to_msg()
           map_msg.header.frame_id = 'map'
           
           map_msg.info.resolution = self.map_resolution
           map_msg.info.width = self.map_width
           map_msg.info.height = self.map_height
           map_msg.info.origin.position.x = self.map_origin_x
           map_msg.info.origin.position.y = self.map_origin_y
           map_msg.info.origin.position.z = 0.0
           map_msg.info.origin.orientation.w = 1.0
           
           # Flatten map data
           flat_map = self.occupancy_map.flatten()
           map_msg.data = flat_map.tolist()
           
           self.map_pub.publish(map_msg)

       def publish_transform(self):
           """Publish the transform between map and robot frames"""
           t = TransformStamped()
           t.header.stamp = self.get_clock().now().to_msg()
           t.header.frame_id = 'map'
           t.child_frame_id = 'base_link'
           
           t.transform.translation.x = float(self.robot_pose[0])
           t.transform.translation.y = float(self.robot_pose[1])
           t.transform.translation.z = 0.0
           
           qw = math.cos(self.robot_pose[2] / 2.0)
           qz = math.sin(self.robot_pose[2] / 2.0)
           t.transform.rotation.w = qw
           t.transform.rotation.z = qz
           
           self.tf_broadcaster.sendTransform(t)

       def save_map(self, filename_prefix="comprehensive_slam_map"):
           """Save the map and pose history"""
           try:
               # Save occupancy map
               map_image = ((1.0 - self.occupancy_map / 100.0) * 255).astype(np.uint8)
               cv2.imwrite(f"{filename_prefix}_map.png", map_image)
               
               # Save map metadata
               map_metadata = {
                   'resolution': float(self.map_resolution),
                   'origin_x': float(self.map_origin_x),
                   'origin_y': float(self.map_origin_y),
                   'width': int(self.map_width),
                   'height': int(self.map_height)
               }
               
               with open(f"{filename_prefix}_map.yaml", 'w') as f:
                   yaml.dump(map_metadata, f)
               
               # Save path
               path_data = []
               for pose_stamped in self.path.poses:
                   path_data.append([
                       pose_stamped.pose.position.x,
                       pose_stamped.pose.position.y
                   ])
               
               with open(f"{filename_prefix}_path.yaml", 'w') as f:
                   yaml.dump(path_data, f)
               
               self.get_logger().info(f"SLAM data saved: {filename_prefix}_map.png/.yaml and path")
               
           except Exception as e:
               self.get_logger().error(f"Error saving SLAM data: {e}")


   def main(args=None):
       rclpy.init(args=args)
       slam_node = IsaacComprehensiveSLAM()
       
       try:
           rclpy.spin(slam_node)
       except KeyboardInterrupt:
           slam_node.get_logger().info("Shutting down Isaac Comprehensive SLAM...")
           slam_node.save_map()
       finally:
           slam_node.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 6: Create SLAM Evaluation Tools

1. Create an evaluation script (`isaac_slam_examples/scripts/evaluate_slam.py`):

   ```python
   #!/usr/bin/env python3

   import rclpy
   from rclpy.node import Node
   from nav_msgs.msg import OccupancyGrid, Path, Odometry
   from geometry_msgs.msg import PoseStamped
   import numpy as np
   import math
   import time


   class SLAMEvaluator(Node):
       """
       Evaluate SLAM system performance metrics
       """
       def __init__(self):
           super().__init__('slam_evaluator')
           
           # Performance tracking variables
           self.map_coverage = 0.0
           self.path_length = 0.0
           self.start_time = time.time()
           self.last_pose = None
           self.traveled_distance = 0.0
           self.map_update_count = 0
           self.localization_accuracy = 0.0
           
           # Subscriptions
           self.map_sub = self.create_subscription(
               OccupancyGrid,
               '/map',
               self.map_callback,
               10
           )
           
           self.path_sub = self.create_subscription(
               Path,
               '/slam_path',
               self.path_callback,
               10
           )
           
           self.pose_sub = self.create_subscription(
               PoseStamped,
               '/slam_pose',
               self.pose_callback,
               10
           )
           
           # Timer for periodic evaluation
           self.eval_timer = self.create_timer(5.0, self.evaluate_performance)

       def map_callback(self, msg):
           """Process map updates for coverage calculation"""
           # Calculate map coverage percentage
           total_cells = msg.info.width * msg.info.height
           occupied_cells = sum(1 for cell in msg.data if cell == 100)
           free_cells = sum(1 for cell in msg.data if cell == 0)
           
           coverage_percentage = (occupied_cells + free_cells) / total_cells * 100 if total_cells > 0 else 0
           self.map_coverage = coverage_percentage
           self.map_update_count += 1
           
           self.get_logger().debug(f"Map coverage: {coverage_percentage:.2f}%")

       def path_callback(self, msg):
           """Process path for length calculation"""
           if len(msg.poses) > 1:
               # Calculate total path length
               total_length = 0.0
               for i in range(1, len(msg.poses)):
                   p1 = msg.poses[i-1].pose.position
                   p2 = msg.poses[i].pose.position
                   dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
                   total_length += dist
               
               self.path_length = total_length

       def pose_callback(self, msg):
           """Process pose for localization tracking"""
           current_pose = np.array([msg.pose.position.x, msg.pose.position.y])
           
           if self.last_pose is not None:
               # Calculate distance traveled
               dist = np.linalg.norm(current_pose - self.last_pose)
               self.traveled_distance += dist
           
           self.last_pose = current_pose

       def evaluate_performance(self):
           """Evaluate and report SLAM performance metrics"""
           elapsed_time = time.time() - self.start_time
           avg_map_update_rate = self.map_update_count / elapsed_time if elapsed_time > 0 else 0
           
           # Calculate metrics
           mapping_efficiency = self.path_length / elapsed_time if elapsed_time > 0 else 0
           map_coverage_per_time = self.map_coverage / elapsed_time if elapsed_time > 0 else 0
           
           # Log metrics
           self.get_logger().info(
               f"\n--- SLAM Performance Evaluation ---\n"
               f"Elapsed Time: {elapsed_time:.2f}s\n"
               f"Map Coverage: {self.map_coverage:.2f}%\n"
               f"Path Length: {self.path_length:.2f}m\n"
               f"Traveled Distance: {self.traveled_distance:.2f}m\n"
               f"Map Update Rate: {avg_map_update_rate:.2f} Hz\n"
               f"Mapping Efficiency: {mapping_efficiency:.2f} m/s\n"
               f"Coverage Rate: {map_coverage_per_time:.4f} %/s\n"
               f"Map Updates: {self.map_update_count}\n"
               f"------------------------------------"
           )

           # Additional metrics could include:
           # - Loop closure detection performance
           # - Localization accuracy vs ground truth (if available)
           # - Computational resource usage
           # - Map consistency metrics


   def main(args=None):
       rclpy.init(args=args)
       evaluator = SLAMEvaluator()
       
       try:
           rclpy.spin(evaluator)
       except KeyboardInterrupt:
           evaluator.get_logger().info("Shutting down SLAM evaluator...")
       finally:
           evaluator.destroy_node()
           rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 7: Test SLAM System with Simulation

1. Build the SLAM package:
   ```bash
   cd ~/isaac_slam_ws
   colcon build --packages-select isaac_slam_examples
   source install/setup.bash
   ```

2. Run the comprehensive SLAM system with the Gazebo simulation:
   ```bash
   # First, in one terminal, launch the Gazebo simulation from Lab 1
   # ros2 launch simple_robot_description sensor_robot.launch.py
   
   # Then launch the SLAM system
   ros2 run isaac_slam_examples comprehensive_slam
   ```

3. Run the evaluation tool in parallel:
   ```bash
   ros2 run isaac_slam_examples evaluate_slam
   ```

4. Visualize results in RViz:
   ```bash
   # In another terminal
   ros2 run rviz2 rviz2
   # Add displays for map, path, and robot pose
   ```

### Step 8: Analyze and Optimize SLAM Performance

1. Review the performance metrics from the evaluation tool
2. Adjust SLAM parameters in your implementation for better performance
3. Consider computational efficiency optimizations
4. Document the trade-offs between accuracy and performance

## Lab Report

Submit a lab report including:

1. **SLAM Implementation**: Documentation of your SLAM pipeline components
2. **Integration Challenges**: Technical obstacles in integrating perception and mapping
3. **Performance Analysis**: Results from your SLAM evaluation with metrics
4. **Isaac Advantages**: Discussion of how Isaac packages enhanced your implementation
5. **Optimization Strategies**: Techniques used to improve SLAM performance
6. **Comparison**: Differences between Isaac-based and traditional SLAM approaches

## Troubleshooting

### Common Issues and Solutions

1. **Feature Tracking Instability**
   - Problem: SLAM drift due to poor feature matching
   - Solution: Adjust feature detection parameters; ensure adequate lighting

2. **Map Quality Issues**
   - Problem: Inconsistent or low-quality maps
   - Solution: Calibrate sensors; adjust mapping parameters; verify coordinate frames

3. **Computational Performance**
   - Problem: SLAM running too slowly on GPU
   - Solution: Optimize algorithms; reduce feature count; use efficient data structures

4. **Coordinate Frame Misalignment**
   - Problem: Mapping errors due to incorrect frame transforms
   - Solution: Verify all TF frames are properly defined and published

## Performance Optimization Tips

### Isaac SLAM Specific Optimizations
- Use Isaac's optimized image pipelines for faster preprocessing
- Implement multi-resolution mapping for efficiency
- Utilize GPU acceleration for all computationally intensive operations
- Optimize feature detection parameters for your specific environment
- Consider using Isaac's DNN packages for semantic SLAM

## Extension Activities

For advanced learners, consider implementing:

1. **Semantic SLAM**: Integrate object recognition with mapping
2. **Multi-session Mapping**: Create consistent maps across multiple runs
3. **Online Loop Closure**: Implement real-time loop closure detection
4. **SLAM for Navigation**: Connect SLAM output to navigation stack

## Next Steps

With SLAM capabilities implemented, you now have a complete perception and mapping foundation that integrates with the NVIDIA Isaac ecosystem. Continue to apply these concepts in real-world scenarios and explore the integration of SLAM with navigation and planning systems.