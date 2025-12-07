---
title: Lab Exercise 1 - Gazebo Simulation Environment Setup
sidebar_position: 8
---

# Lab Exercise 1: Gazebo Simulation Environment Setup

## Objective

In this lab exercise, you will create a complete Gazebo simulation environment with a mobile robot navigating through a simple obstacle course. You'll learn to configure the environment, spawn a robot, and implement basic obstacle avoidance behavior.

## Learning Objectives

After completing this lab, you will be able to:
- Create a custom Gazebo world with obstacles
- Spawn a robot model with appropriate sensors
- Implement a ROS 2 node to control the robot in simulation
- Use sensor data to navigate around obstacles
- Evaluate the simulation results

## Prerequisites

- Completion of Gazebo/Unity Weeks 4-5 content
- Working ROS 2 installation with Gazebo packages
- Basic knowledge of ROS 2 nodes, topics, and messages
- Understanding of coordinate systems and transformations

## Equipment Required

- Computer with ROS 2 and Gazebo installed
- Terminal/shell access
- Text editor or IDE

## Lab Steps

### Step 1: Setting up the Simulation Environment

1. Create a new ROS 2 package for the simulation:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python gazebo_simulation_tutorial
   ```

2. Create the directory structure for the robot model:
   ```bash
   cd gazebo_simulation_tutorial
   mkdir -p models/simple_robot/meshes
   mkdir -p models/simple_robot/materials/textures
   ```

3. Create a URDF file for the simple robot in `models/simple_robot/urdf/simple_robot.urdf`:
   ```xml
   <?xml version="1.0"?>
   <robot name="simple_robot">
     <!-- Base link -->
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.5" radius="0.2"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 0.8"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.5" radius="0.2"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <!-- Hokuyo laser range finder -->
     <link name="laser_link">
       <visual>
         <geometry>
           <box size="0.1 0.05 0.05"/>
         </geometry>
       </visual>
       <collision>
         <geometry>
           <box size="0.1 0.05 0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.1"/>
         <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
       </inertial>
     </link>

     <!-- Joint connecting laser to base -->
     <joint name="laser_joint" type="fixed">
       <parent link="base_link"/>
       <child link="laser_link"/>
       <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
     </joint>

     <!-- Gazebo plugins for ROS 2 integration -->
     <gazebo reference="base_link">
       <material>Gazebo/Blue</material>
     </gazebo>

     <gazebo reference="laser_link">
       <sensor name="laser" type="ray">
         <pose>0 0 0 0 0 0</pose>
         <visualize>false</visualize>
         <update_rate>10</update_rate>
         <ray>
           <scan>
             <horizontal>
               <samples>360</samples>
               <resolution>1</resolution>
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
         <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
           <ros>
             <argument>~/out:=scan</argument>
           </ros>
           <output_type>sensor_msgs/LaserScan</output_type>
           <frame_name>laser_link</frame_name>
         </plugin>
       </sensor>
     </gazebo>
   </robot>
   ```

### Step 2: Creating a Custom World

1. Create a world file in `models/worlds/obstacle_course.world`:
   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="obstacle_course">
       <!-- Include the outdoor environment -->
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Add ground plane -->
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Define obstacles -->
       <model name="wall1">
         <pose>0 -4 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>10 0.5 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>10 0.5 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.5 0.5 0.5 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>1.0</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>1.0</iyy>
               <iyz>0.0</iyz>
               <izz>1.0</izz>
             </inertia>
           </inertial>
         </link>
       </model>

       <model name="wall2">
         <pose>0 4 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>10 0.5 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>10 0.5 1</size>
               </box>
             </geometry>
             <material>
               <ambient>0.5 0.5 0.5 1</ambient>
               <diffuse>0.5 0.5 0.5 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>1.0</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>1.0</iyy>
               <iyz>0.0</iyz>
               <izz>1.0</izz>
             </inertial>
           </inertial>
         </link>
       </model>

       <model name="obstacle1">
         <pose>-2 0 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
             <material>
               <ambient>1 0 0 1</ambient>
               <diffuse>1 0 0 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>1.0</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>1.0</iyy>
               <iyz>0.0</iyz>
               <izz>1.0</izz>
             </inertial>
           </inertial>
         </link>
       </model>

       <model name="obstacle2">
         <pose>3 1 0.5 0 0 0</pose>
         <link name="link">
           <collision name="collision">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
           </collision>
           <visual name="visual">
             <geometry>
               <box>
                 <size>1 1 1</size>
               </box>
             </geometry>
             <material>
               <ambient>1 0 0 1</ambient>
               <diffuse>1 0 0 1</diffuse>
             </material>
           </visual>
           <inertial>
             <mass>1.0</mass>
             <inertia>
               <ixx>1.0</ixx>
               <ixy>0.0</ixy>
               <ixz>0.0</ixz>
               <iyy>1.0</iyy>
               <iyz>0.0</iyz>
               <izz>1.0</izz>
             </inertial>
           </inertial>
         </link>
       </model>
     </world>
   </sdf>
   ```

### Step 3: Implementing the Obstacle Avoidance Node

1. Create the obstacle avoidance node in `gazebo_simulation_tutorial/gazebo_simulation_tutorial/obstacle_avoidance.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   from geometry_msgs.msg import Twist
   import math
   
   
   class ObstacleAvoidance(Node):
       def __init__(self):
           super().__init__('obstacle_avoidance')
           
           # Create publisher for robot velocity commands
           self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
           
           # Create subscriber for laser scan data
           self.scan_sub = self.create_subscription(
               LaserScan,
               '/scan',
               self.scan_callback,
               10
           )
           
           # Timer for control loop
           self.timer = self.create_timer(0.1, self.control_loop)
           
           # Initialize robot state
           self.linear_vel = 0.5
           self.angular_vel = 0.0
           self.lidar_data = None
           
           self.get_logger().info('Obstacle avoidance node initialized')
       
       def scan_callback(self, msg):
           self.lidar_data = msg
       
       def control_loop(self):
           if self.lidar_data is None:
               return
           
           # Analyze the lidar data to detect obstacles
           min_distance = min(self.lidar_data.ranges)
           min_index = self.lidar_data.ranges.index(min_distance)
           min_angle = self.lidar_data.angle_min + min_index * self.lidar_data.angle_increment
           
           # Simple obstacle avoidance logic
           if min_distance < 1.0:  # Obstacle detected within 1 meter
               # Stop and turn away from obstacle
               self.linear_vel = 0.0
               self.angular_vel = 0.5 if min_angle > 0 else -0.5
           else:
               # Move forward
               self.linear_vel = 0.5
               self.angular_vel = 0.0
           
           # Publish velocity commands
           twist = Twist()
           twist.linear.x = self.linear_vel
           twist.angular.z = self.angular_vel
           self.cmd_vel_pub.publish(twist)
   
   
   def main(args=None):
       rclpy.init(args=args)
       obstacle_avoidance = ObstacleAvoidance()
       
       try:
           rclpy.spin(obstacle_avoidance)
       except KeyboardInterrupt:
           pass
       
       obstacle_avoidance.destroy_node()
       rclpy.shutdown()
   
   
   if __name__ == '__main__':
       main()
   ```

### Step 4: Running the Simulation

1. Build your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select gazebo_simulation_tutorial
   source install/setup.bash
   ```

2. Launch Gazebo with your custom world:
   ```bash
   # In terminal 1
   ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/src/gazebo_simulation_tutorial/models/worlds/obstacle_course.world
   ```

3. In another terminal, spawn your robot:
   ```bash
   # In terminal 2
   ros2 run gazebo_ros spawn_entity.py -entity simple_robot -file $(pwd)/src/gazebo_simulation_tutorial/models/simple_robot/urdf/simple_robot.urdf -x 0 -y 0 -z 0.5
   ```

4. Run the obstacle avoidance node:
   ```bash
   # In terminal 3
   ros2 run gazebo_simulation_tutorial obstacle_avoidance
   ```

### Step 5: Observing and Analyzing Results

1. Use RViz2 to visualize the robot and sensor data:
   ```bash
   # In terminal 4
   rviz2
   ```
   
   Add displays for:
   - Robot model using RobotModel display
   - Laser scan data using LaserScan display
   - Robot path using Path display

2. Record the robot's behavior and measure:
   - Time to navigate through the course
   - Number of collisions (should be zero)
   - Success rate of obstacle avoidance

## Expected Results

- The robot should navigate through the obstacle course without collisions
- The laser scanner should detect obstacles and trigger avoidance behavior
- The robot should successfully find paths around obstacles
- Performance metrics should show high success rate

## Troubleshooting

- If the robot doesn't move, check that the `/cmd_vel` topic is properly connected
- If obstacles aren't detected, verify the laser scan topic is publishing data
- If Gazebo crashes, ensure sufficient system resources are available

## Extension Activities

1. Implement a more sophisticated path planning algorithm (e.g., A* or Dijkstra)
2. Add dynamic obstacles that move around the environment
3. Integrate camera data for more complex navigation strategies
4. Compare simulation results with real robot behavior

## Assessment Questions

1. How does the quality of Gazebo simulation affect the performance of your obstacle avoidance algorithm?
2. What are the differences between sim-to-real transfer using Gazebo versus real robot execution?
3. What limitations of simulation did you encounter during this lab?

## Summary

This lab introduced you to creating a complete Gazebo simulation environment with robot modeling, sensor integration, and control logic. You've successfully implemented obstacle avoidance behavior and evaluated its performance in simulation.