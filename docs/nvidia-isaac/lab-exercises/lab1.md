---
title: Lab Exercise 1 - NVIDIA Isaac Perception and VSLAM Implementation
sidebar_position: 12
---

# Lab Exercise 1: NVIDIA Isaac Perception and VSLAM Implementation

## Objective

In this lab exercise, you will implement a complete perception and visual SLAM pipeline using NVIDIA Isaac SDK. You'll create a system that processes camera data to detect objects, estimate depth, and build a map of the environment while the robot navigates.

## Learning Objectives

After completing this lab, you will be able to:
- Set up NVIDIA Isaac perception pipeline with optimized deep learning models
- Implement Visual SLAM to map the environment and track robot position
- Integrate perception with navigation and control systems
- Deploy AI models on edge computing platforms (e.g., NVIDIA Jetson)
- Evaluate and tune perception algorithms for your specific robot platform

## Prerequisites

- Completion of NVIDIA Isaac Modules 7-9 content
- Access to NVIDIA GPU or Jetson development kit
- Understanding of ROS 2 concepts from Module 1
- Basic Python and C++ programming skills
- Familiarity with computer vision concepts

## Equipment Required

- NVIDIA Jetson platform (Xavier NX, Orin, or equivalent) or RTX workstation
- RGB camera with calibration parameters
- Robot platform capable of differential drive control
- ROS 2-enabled robot controller

## Lab Steps

![NVIDIA Isaac Perception and VSLAM Architecture](/img/nvidia-isaac-perception-pipeline.png)

### Step 1: Environment Setup

1. Verify your NVIDIA Isaac and ROS 2 integration:

```bash
# Verify Isaac ROS packages are installed
dpkg -l | grep isaac

# Source ROS 2 and Isaac environments
source /opt/ros/humble/setup.bash
source /opt/nvidia/isaac/ros_humble/latest/setup.bash

# Check for Isaac ROS nodes
ros2 pkg list | grep isaac
```

2. Create a new workspace for the perception pipeline:

```bash
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_detection_postprocessor.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_unified_rectifier.git

cd ~/isaac_ws
colcon build --symlink-install --packages-select \
  isaac_ros_visual_slam \
  isaac_ros_detection_postprocessor \
  isaac_ros_unified_rectifier
```

### Step 2: Create the Perception Pipeline Node

Create a new ROS 2 package for the perception system:

```bash
cd ~/isaac_ws/src
ros2 pkg create --build-type ament_python perception_pipeline
cd perception_pipeline
mkdir -p perception_pipeline/{launch,config}
```

1. Create the main perception pipeline node `perception_pipeline/perception_pipeline/main_perception.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from builtin_interfaces.msg import Time
import cv2
from cv_bridge import CvBridge
import numpy as np
import torch
import torchvision.transforms as T
from collections import deque
import tf_transformations


class IsaacPerceptionPipeline(Node):
    def __init__(self):
        super().__init__('isaac_perception_pipeline')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize queues for temporal processing
        self.image_queue = deque(maxlen=5)
        
        # Initialize pose tracking for VSLAM
        self.current_pose = np.zeros(3)  # x, y, theta
        self.current_orientation = np.eye(3)
        
        # Subscribe to camera topics
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )
        
        # Publisher for processed perception data
        self.object_detection_publisher = self.create_publisher(
            Detection2DArray,
            '/perception/detections',
            10
        )
        
        self.vslam_publisher = self.create_publisher(
            Odometry,
            '/vslam/odometry',
            10
        )
        
        # Initialize camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None
        
        # Initialize deep learning models for perception
        self.setup_deep_learning_models()
        
        self.get_logger().info("Isaac Perception Pipeline initialized")

    def setup_deep_learning_models(self):
        """
        Initialize deep learning models for object detection and depth estimation
        """
        try:
            # Load pre-trained detection model (e.g., using TorchVision)
            self.detection_model = torch.hub.load(
                'ultralytics/yolov5', 
                'yolov5s', 
                pretrained=True
            )
            self.detection_model.eval()
            
            # For a more Isaac-specific approach, you'd load TensorRT optimized models
            # self.detection_model = self.load_tensorrt_model("/path/to/optimized_model.plan")
            
            self.get_logger().info("Deep learning models loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load deep learning models: {e}")

    def info_callback(self, msg):
        """Handle camera calibration information"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process incoming camera image for perception and VSLAM"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Add to queue for temporal processing
            self.image_queue.append({
                'image': cv_image,
                'timestamp': msg.header.stamp
            })
            
            # Run perception pipeline
            detections = self.run_object_detection(cv_image)
            vslam_result = self.run_visual_slam(cv_image, msg.header.stamp)
            
            # Publish results
            if detections is not None:
                self.publish_detections(detections, msg.header)
            
            if vslam_result is not None:
                self.publish_vslam_result(vslam_result, msg.header)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def run_object_detection(self, image):
        """Run object detection on the input image"""
        if self.detection_model is None:
            return None
            
        try:
            # Convert image for model input
            img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img_tensor = T.ToTensor()(img_rgb).unsqueeze(0)  # Add batch dimension
            
            # Run inference
            with torch.no_grad():
                results = self.detection_model(img_tensor)
            
            # Process results
            # Note: This is a simplified approach. In Isaac, you'd use optimized 
            # CUDA-accelerated models through Isaac ROS packages
            detections = Detection2DArray()
            
            # Convert YOLO results to vision_msgs format
            # This is a placeholder; actual implementation would depend on your model
            for detection in results.xyxy[0]:  # xyxy format: [x1, y1, x2, y2, conf, class]
                if detection[4] > 0.5:  # Confidence threshold
                    # Process detection and add to result
                    pass
            
            return results
        except Exception as e:
            self.get_logger().error(f"Error in object detection: {e}")
            return None

    def run_visual_slam(self, image, timestamp):
        """Estimate camera motion using visual slam"""
        if len(self.image_queue) < 2:
            return None
            
        try:
            # Get the previous image for comparison
            prev_data = self.image_queue[-2]
            curr_data = self.image_queue[-1]
            
            # Convert images to grayscale
            prev_gray = cv2.cvtColor(prev_data['image'], cv2.COLOR_BGR2GRAY)
            curr_gray = cv2.cvtColor(curr_data['image'], cv2.COLOR_BGR2GRAY)
            
            # Feature detection and matching (simplified approach)
            # In Isaac, you'd use optimized CUDA algorithms
            orb = cv2.ORB_create(nfeatures=1000)
            kp1, des1 = orb.detectAndCompute(prev_gray, None)
            kp2, des2 = orb.detectAndCompute(curr_gray, None)
            
            if des1 is not None and des2 is not None:
                bf = cv2.BFMatcher()
                matches = bf.knnMatch(des1, des2, k=2)
                
                # Apply Lowe's ratio test
                good_matches = []
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.75 * n.distance:
                            good_matches.append(m)
                
                if len(good_matches) >= 10:  # Minimum matches required
                    # Extract matched keypoints
                    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    
                    # Estimate essential matrix if camera matrix is available
                    if self.camera_matrix is not None:
                        E, mask = cv2.findEssentialMat(
                            src_pts, dst_pts, 
                            self.camera_matrix, 
                            method=cv2.RANSAC, 
                            prob=0.999, 
                            threshold=1.0
                        )
                        
                        if E is not None:
                            # Recover pose
                            _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, self.camera_matrix)
                            
                            # Update pose estimate (simplified)
                            dt = 0.1  # Time delta between frames
                            self.current_pose[:2] += dt * t.flatten()[:2]  # Update x, y
                            self.current_pose[2] += np.arctan2(R[1, 0], R[0, 0])  # Update theta
                        
                            return {
                                'position': self.current_pose.copy(),
                                'orientation': R.copy(),
                                'timestamp': timestamp
                            }
            
            return None
        except Exception as e:
            self.get_logger().error(f"Error in VSLAM: {e}")
            return None

    def publish_detections(self, detections, header):
        """Publish object detection results"""
        # This method would convert the detection results to the appropriate ROS message format
        # For now, it's a placeholder
        pass

    def publish_vslam_result(self, vslam_result, header):
        """Publish VSLAM results as odometry"""
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Set position
        odom_msg.pose.pose.position.x = float(vslam_result['position'][0])
        odom_msg.pose.pose.position.y = float(vslam_result['position'][1])
        odom_msg.pose.pose.position.z = 0.0  # Assuming planar navigation
        
        # Convert orientation matrix to quaternion
        R = vslam_result['orientation']
        qw, qx, qy, qz = tf_transformations.quaternion_from_matrix(
            np.block([[R, [[0], [0], [0]]], [[0, 0, 0, 1]]])
        )
        
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        
        # Publishing velocity estimates would require more sophisticated tracking
        self.vslam_publisher.publish(odom_msg)

    def load_tensorrt_model(self, model_path):
        """
        Load a TensorRT optimized model (placeholder implementation)
        """
        # This would use TensorRT Python bindings to load an optimized model
        # In practice, you'd use Isaac ROS packages that handle this efficiently
        pass


def main(args=None):
    rclpy.init(args=args)
    
    perception_pipeline = IsaacPerceptionPipeline()
    
    try:
        rclpy.spin(perception_pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        perception_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. Create a launch file `perception_pipeline/launch/perception_pipeline_launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the perception pipeline'
    )
    
    # Create the perception pipeline node
    perception_node = Node(
        package='perception_pipeline',
        executable='main_perception.py',
        name='isaac_perception_pipeline',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            # Add parameters for tuning the perception algorithms
            {'detection_threshold': 0.5},
            {'tracking_lifetime': 2.0},
            {'map_resolution': 0.05},  # meters per cell
            {'max_map_size_x': 20.0},  # meters
            {'max_map_size_y': 20.0},  # meters
        ],
        remappings=[
            ('/camera/image_raw', '/front_camera/image_raw'),
            ('/camera/camera_info', '/front_camera/camera_info'),
            ('/perception/detections', '/isaac/detections'),
            ('/vslam/odometry', '/isaac/vslam_odom'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        namespace_arg,
        perception_node
    ])
```

### Step 3: Implement a Reinforcement Learning Component

For the AI-robot brain integration, let's implement a simple navigation policy using reinforcement learning:

1. Create `perception_pipeline/perception_pipeline/navigation_rl.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
from collections import deque
import math


class NavigationPolicyNetwork(nn.Module):
    """Simple neural network for navigation policy"""
    def __init__(self, state_size=24, action_size=5):
        super(NavigationPolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, action_size)
        self.relu = nn.ReLU()
        
    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        return self.fc3(x)


class IsaacNavigationRL(Node):
    """Reinforcement learning node for navigation"""
    
    def __init__(self):
        super().__init__('isaac_navigation_rl')
        
        # Initialize neural networks
        state_size = 24  # 20 LIDAR bins + 3 for goal direction + 1 for current speed
        action_size = 5  # Discrete actions for navigation
        
        self.q_network = NavigationPolicyNetwork(state_size, action_size)
        self.target_network = NavigationPolicyNetwork(state_size, action_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=0.001)
        
        # Initialize DQN parameters
        self.memory = deque(maxlen=10000)
        self.epsilon = 1.0  # Exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.gamma = 0.95  # Discount factor
        self.tau = 0.005  # Target network update rate
        self.batch_size = 32
        
        # Initialize state variables
        self.laser_data = None
        self.odom_data = None
        self.goal_pose = None
        self.current_action = None
        
        # Subscribers
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
        
        self.vslam_sub = self.create_subscription(
            Odometry,
            '/vslam/odometry',
            self.vslam_callback,
            10
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.episode_reward_pub = self.create_publisher(Float32, '/rl/episode_reward', 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_step)
        
        self.get_logger().info("Isaac Navigation RL node initialized")

    def laser_callback(self, msg):
        """Process LIDAR data"""
        # Simplify LIDAR data to fixed-size representation
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        
        # Bin the data into fixed number of directions
        bin_count = 20
        step = len(ranges) // bin_count
        binned_ranges = [np.min(ranges[i:i+step]) for i in range(0, len(ranges), step)]
        binned_ranges = binned_ranges[:bin_count]
        
        self.laser_data = np.array(binned_ranges)

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg

    def vslam_callback(self, msg):
        """Process VSLAM data"""
        # Use VSLAM pose estimation if available
        self.vslam_data = msg

    def get_state(self):
        """Construct state representation from sensor data"""
        if self.laser_data is None or self.odom_data is None:
            return None
            
        # Combine LIDAR data, goal direction, and current speed
        state = self.laser_data.copy()
        
        # Add goal direction if available
        if self.goal_pose is not None and self.odom_data is not None:
            dx = self.goal_pose.pose.position.x - self.odom_data.pose.pose.position.x
            dy = self.goal_pose.pose.position.y - self.odom_data.pose.pose.position.y
            dist_to_goal = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            
            state = np.append(state, [dx, dy, dist_to_goal])
        else:
            # Default values if no goal
            state = np.append(state, [0.0, 0.0, 10.0])
        
        # Add current linear/angular velocities
        lin_vel = self.odom_data.twist.twist.linear.x
        ang_vel = self.odom_data.twist.twist.angular.z
        state = np.append(state, [lin_vel])
        
        return state

    def select_action(self, state):
        """Select action using epsilon-greedy policy"""
        if random.random() < self.epsilon:
            # Random action for exploration
            return random.randrange(5)
        
        # Use neural network for exploitation
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        q_values = self.q_network(state_tensor)
        return q_values.argmax().item()

    def step_simulation(self):
        """Simulate a step in the environment (placeholder)"""
        # In a real implementation, this would interface with the simulator or real robot
        # For now, we'll just return a mock reward
        
        # Calculate reward based on:
        # 1. Distance to goal (positive reward for getting closer)
        # 2. Collision avoidance (negative reward for obstacles too close)
        # 3. Efficiency (small negative reward for taking too long)
        
        reward = -0.01  # Small negative reward for time
        
        if self.laser_data is not None:
            # Check for potential collisions
            min_distance = np.min(self.laser_data)
            if min_distance < 0.5:  # Threshold for collision danger
                reward -= 1.0  # Significant negative reward for collision risk
            
            # Positive reward for forward progress
            if self.odom_data is not None:
                lin_vel = self.odom_data.twist.twist.linear.x
                if lin_vel > 0.1:  # Moving forward
                    reward += 0.05
        
        return reward, False  # No terminal condition for now

    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay memory"""
        self.memory.append((state, action, reward, next_state, done))

    def replay(self):
        """Train the model on a batch of experiences"""
        if len(self.memory) < self.batch_size:
            return
            
        batch = random.sample(self.memory, self.batch_size)
        states = torch.FloatTensor([e[0] for e in batch])
        actions = torch.LongTensor([e[1] for e in batch])
        rewards = torch.FloatTensor([e[2] for e in batch])
        next_states = torch.FloatTensor([e[3] for e in batch])
        dones = torch.BoolTensor([e[4] for e in batch])
        
        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        target_q_values = rewards + (self.gamma * next_q_values * ~dones)
        
        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

    def update_target_network(self):
        """Soft update the target network"""
        for target_param, local_param in zip(
            self.target_network.parameters(), 
            self.q_network.parameters()
        ):
            target_param.data.copy_(
                self.tau * local_param.data + (1.0 - self.tau) * target_param.data
            )

    def control_step(self):
        """Main control loop for RL navigation"""
        state = self.get_state()
        if state is None:
            return
            
        # Select action using current policy
        action = self.select_action(state)
        
        # Convert discrete action to continuous velocity commands
        cmd_vel = Twist()
        if action == 0:  # Move forward
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.0
        elif action == 1:  # Turn left
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = 0.3
        elif action == 2:  # Turn right
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = -0.3
        elif action == 3:  # Move backward
            cmd_vel.linear.x = -0.1
            cmd_vel.angular.z = 0.0
        else:  # Stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        # Publish command
        self.cmd_pub.publish(cmd_vel)
        
        # Get reward and update model
        reward, done = self.step_simulation()
        
        next_state = self.get_state()
        if next_state is not None:
            self.remember(state, action, reward, next_state, done)
        
        # Train the model
        if len(self.memory) > self.batch_size:
            self.replay()
            self.update_target_network()
        
        # Decay exploration rate
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        
        # Publish episode reward if needed
        reward_msg = Float32()
        reward_msg.data = reward
        self.episode_reward_pub.publish(reward_msg)


def main(args=None):
    rclpy.init(args=args)
    
    rl_node = IsaacNavigationRL()
    
    try:
        rclpy.spin(rl_node)
    except KeyboardInterrupt:
        pass
    finally:
        rl_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Running the Complete System

1. Build your new package:

```bash
cd ~/isaac_ws
source install/setup.bash
colcon build --packages-select perception_pipeline
```

2. Launch the complete perception and navigation system:

```bash
source install/setup.bash
ros2 launch perception_pipeline perception_pipeline_launch.py
```

3. In another terminal, launch the RL navigation:

```bash
source install/setup.bash
ros2 run perception_pipeline navigation_rl.py
```

### Step 5: Testing and Evaluation

1. Monitor the perception output:
```bash
# View detection results
ros2 topic echo /perception/detections

# View VSLAM odometry
ros2 topic echo /vslam/odometry

# View robot commands
ros2 topic echo /cmd_vel
```

2. Evaluate the performance of your perception system by:
   - Checking detection accuracy in various scenarios
   - Measuring pose estimation drift over time
   - Assessing navigation performance in different environments
   - Monitoring resource usage (CPU, GPU, memory)

## Expected Results

- The perception system should detect objects in the camera feed
- The VSLAM system should estimate robot motion relative to the environment
- The RL navigation system should move the robot toward goals while avoiding obstacles
- The system should run efficiently on your NVIDIA hardware platform

## Troubleshooting

- If GPU acceleration isn't working, verify that CUDA and TensorRT are properly installed
- If detections are inaccurate, try calibrating your camera and adjusting thresholds
- If VSLAM is unstable, check that the robot has sufficient distinctive visual features in its environment
- If navigation is erratic, tune the RL hyperparameters (epsilon decay, learning rate, etc.)

## Extension Activities

1. Train a custom object detection model for your specific application
2. Implement a more sophisticated navigation policy using advanced RL algorithms (PPO, SAC)
3. Add semantic mapping capability to create labeled environment maps
4. Integrate IMU data for improved motion estimation
5. Optimize the pipeline for your specific robot's computational constraints

## Summary

In this lab, you've implemented a complete perception system using NVIDIA Isaac tools, including visual SLAM for localization and a reinforcement learning algorithm for navigation. This demonstrates the tight integration between AI perception and robot control that is possible with NVIDIA's platform.