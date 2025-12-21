# Week 4-5: Simulation Environments and Gazebo Integration

In these two weeks, we'll dive deep into Gazebo simulation, which is specifically designed for robotics applications. You'll learn to create realistic environments, spawn robots, and interface them with ROS 2 nodes for comprehensive testing.

## Learning Objectives

By the end of this week, you will be able to:

- Install and configure Gazebo for robotics simulation
- Create and customize 3D environments for robot testing
- Spawn robots and models in Gazebo with realistic physics
- Interface Gazebo with ROS 2 using Gazebo plugins
- Implement sensor models that reflect real-world sensors
- Design simulation experiments to validate robot behaviors

## Introduction to Gazebo

Gazebo is a physics-based simulation engine that provides realistic rendering, sensor simulation, and dynamics. It's widely used in robotics research and development as it provides high-fidelity simulation of robots in complex environments.

### Key Features of Gazebo

- **Physics Simulation**: Accurate simulation of forces, collisions, and dynamics
- **Sensor Simulation**: Realistic models for cameras, LIDAR, IMU, GPS, and other sensors
- **3D Visualization**: Interactive 3D visualization of simulation scenarios
- **Robot Models**: Support for common robot formats like URDF and SDF
- **Plugin Architecture**: Extensible architecture to customize simulation behavior

## Setting Up Gazebo with ROS 2

First, let's install the necessary packages:

```bash
# Install Gazebo and ROS 2 integration packages
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Creating a Gazebo World

![Gazebo World Architecture](/img/gazebo-architecture.png)

Let's create a simple world file for our simulation:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include the outdoor environment -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- <DOMTranslate>Add ground plane</DOMTranslate> -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- <DOMTranslate>Define a simple box obstacle</DOMTranslate> -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
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
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Connecting Gazebo to ROS 2

Gazebo integrates with ROS 2 through plugins. Let's create a robot model with ROS 2 integration:

### Robot Model with ROS 2 Plugins

```xml
<?xml version="1.0" ?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
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

  <!-- Sensor link -->
  <link name="sensor_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting sensor to base -->
  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins for ROS 2 integration -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
      <robotNamespace>/simple_robot</robotNamespace>
      <robotSimType>gazebo_ros2_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Gazebo sensor plugin -->
  <gazebo reference="sensor_link">
    <sensor name="camera_sensor" type="camera">
      <update_rate>30</update_rate>
      <camera name="camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <cameraName>simple_robot/camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>sensor_link</frameName>
        <hackBaseline>0.07</hackBaseline>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Advanced Sensor Models

Let's also examine a more complex LiDAR sensor model for more realistic simulation:

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
          <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/simple_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Working with Gazebo and ROS 2

### Launching Gazebo with a ROS 2 Robot

Create a launch file to bring up Gazebo with your robot:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('your_robot_description'),
                'worlds',
                'simple_world.world'
            ])
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
```

## Creating Simulation Experiments

Simulation experiments help validate robot behaviors before real-world deployment. Here's how to design an effective simulation experiment:

1. **Define the objective**: What behavior are you testing?
2. **Set up the environment**: Create a scenario that reflects real-world conditions
3. **Establish metrics**: How will you measure success?
4. **Run multiple trials**: Test with different parameters to ensure robustness
5. **Analyze results**: Compare simulation results with expected outcomes

## Practical Application

Gazebo simulation is used in many robotics projects, including:

- Autonomous vehicle testing
- Warehouse robot navigation
- Drone flight simulation
- Humanoid robot gait development
- Agricultural robot path planning

## Lab Exercise Preview

In the next section, you'll find the detailed instructions for the Gazebo/Unity lab exercise, where you'll implement a complete simulation environment with obstacle avoidance.

## Summary

In these weeks, you've learned:

- How to set up Gazebo for robotics simulation
- The structure of Gazebo world files and robot models
- How to connect Gazebo to ROS 2 using plugins
- How to design simulation experiments to validate robot behaviors

## Navigation

[← Previous: Introduction to Gazebo/Unity](./intro.md) | [Next: Week 6: Sensor Integration and Advanced Simulation](./week6.md) | [Module Home](./intro.md)

Continue to [Week 6: Sensor Integration and Advanced Simulation](./week6.md) to explore Unity integration and advanced sensor simulation techniques.