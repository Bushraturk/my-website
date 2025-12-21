---
title: "ہفتہ 4-5: Gazebo کی بنیاد اور ماحول کی تیاری"
sidebar_position: 6
---

# ہفتہ 4-5: Gazebo کی بنیاد اور ماحول کی تیاری

Gazebo سیمولیٹر کی دنیا میں خوش آمدید! یہ دو ہفتے آپ کو Gazebo میں روبوٹکس سیمولیشن کی دنیا میں لے جائے گا، جہاں آپ فزکل اور سینسر ماڈلنگ کے اہم تصورات سیکھیں گے۔

## سیکھنے کے اہداف

اس ہفتے کے اختتام پر، آپ درج ذیل کر سکیں گے:

- Gazebo ماحول کو تیار کرنا اور ترتیب دینا
- سیمولیٹڈ روبوٹس کو اسپون کرنا اور ROS 2 کے ساتھ جوڑنا
- سینسر ماڈلز کو تیار کرنا اور جانچنا
- فزکس پیرامیٹرز کو ترتیب دینا اور کارکردگی کو بہتر بنانا
- سیمولیشن اور حقیقی دنیا کے درمیان منتقلی کے لیے کارآمد تکنیکیں سیکھنا

## Gazebo کا تعارف

Gazebo ایک حقیقت پسندانہ فزکس سیمولیٹر ہے جو روبوٹکس کی ترقی کے لیے ڈیزائن کیا گیا ہے۔ یہ OpenGL استعمال کرتا ہے تاکہ 3D ویژنلائزیشن کا عمل ہو اور ODE، Bullet، یا DART استعمال کرتا ہے تاکہ فزکل کے ایکٹس کی تشریح ہو۔

### Gazebo کی اہم خصوصیات

- **High-fidelity physics**: Realistic simulation of rigid-body dynamics
- **Realistic rendering**: High-quality graphics and lighting effects
- **Sensor models**: Accurate simulation of cameras, LiDAR, IMUs, etc.
- **Plugins architecture**: Extensible system for custom simulation logic
- **Open source**: Free and actively maintained by the community

## تنصیب اور ترتیب

### Gazebo Classic کو انسٹال کرنا

Ubuntu 20.04 کے لیے:

```bash
sudo apt update
sudo apt install gazebo11 libgazebo11-dev
```

### ROS 2 کے ساتھ انضمام

Gazebo کو ROS 2 کے ساتھ استعمال کرنے کے لیے، یقین کریں کہ ROS 2 packages انسٹال ہیں:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

## بنیادی ماحول تیار کرنا

### ایک سادہ ورلڈ بنانا

Gazebo ورلڈز SDF (Simulation Description Format) میں تحریر کیے جاتے ہیں:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="small_room">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- ایک سادہ کیوب شامل کریں -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <iyy>0.083</iyy>
            <izz>0.083</izz>
          </inertia>
        </inertial>
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
      </link>
    </model>
  </world>
</sdf>
```

اس فائل کو `small_room.world` کے نام سے محفوظ کریں اور اسے اس طرح لوڈ کریں:

```bash
gazebo small_room.world
```

## سیمولیٹڈ روبوٹس استعمال کرنا

### ایک سادہ روبوٹ مدل کرنا

TurtleBot3 کی مثل کے طور پر:

```xml
<?xml version="1.0" ?>
<robot name="turtlebot3_burger" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Footprint -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link" />
    <origin xyz="0.0 0.0 0.010" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>

    <inertial>
      <!-- Inertial parameters -->
      <mass value="1e-5" />
      <origin xyz="0 0 0" />
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6" />
    </inertial>
  </link>

  <!-- چکر لگانا -->
  <link name="wheel_left_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder length="0.04" radius="0.033"/>
      </geometry>
    </visual>
  </link>

  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <origin xyz="0.0 0.1 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="wheel_right_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.570796 0 0"/>
      <geometry>
        <cylinder length="0.04" radius="0.033"/>
      </geometry>
    </visual>
  </link>

  <joint name="wheel_right_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <origin xyz="0.0 -0.1 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Gazebo کے ساتھ سیمولیٹڈ روبوٹ کو چلانا

Gazebo کے ساتھ روبوٹ کو چلانے کے لیے، آپ کو Gazebo plugins کی ضرورت ہوگی:

```xml
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_footprint</robotBaseFrame>
    <publishOdomTF>true</publishOdomTF>
    <odometrySource>world</odometrySource>
    <publishWheelTF>false</publishWheelTF>
    <publishWheelJointState>true</publishWheelJointState>
    <legacyMode>false</legacyMode>
    <minDepth>0.0</minDepth>
    <maxDepth>0.0</maxDepth>
    <wheelSeparation>0.160</wheelSeparation>
    <wheelDiameter>0.066</wheelDiameter>
    <broadcastTF>1</broadcastTF>
    <updateRate>30.0</updateRate>
    <leftJoint>wheel_left_joint</leftJoint>
    <rightJoint>wheel_right_joint</rightJoint>
    <wheelTorque>1.0</wheelTorque>
    <wheelAcceleration>1.8</wheelAcceleration>
  </plugin>
</gazebo>
```

## سینسر ماڈلنگ

### کیمرہ سینسر

Gazebo میں کیمرہ سینسر کو اس طرح تعریف کیا جاتا ہے:

```xml
<sensor name="camera" type="camera">
  <update_rate>30.0</update_rate>
  <camera name="rgb_camera">
    <horizontal_fov>1.047</horizontal_fov>
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
    <frame_name>camera_frame</frame_name>
  </plugin>
</sensor>
```

### LiDAR سینسر

2D LiDAR سینسر کی تعریف:

```xml
<sensor name="laser" type="gpu_lidar">
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
      <min>0.10</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser" filename="libgazebo_ros_gpu_lidar.so">
    <topicName>scan</topicName>
    <frameName>laser_frame</frameName>
  </plugin>
</sensor>
```

## ROS 2 کے ساتھ انضمام

### launch فائل کا استعمال

робوٹ کو Gazebo میں شروع کرنے کے لیے، launch فائل استعمال کریں:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Gazebo server launch
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        )
    )
    
    # Gazebo client launch
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
    ])
```

## کارکردگی بہتر بنانا

### فزکس کی ترتیبات

Gazebo ماحول کی کارکردگی کے لیے، فزکس کی ترتیبات کو اس طرح ایڈجسٹ کریں:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</rtf>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### GPU ایکسلریشن کا استعمال

GPU ایکسلریشن کو یقینی بنانے کے لیے، یقین کریں کہ آپ کے گرافکس ڈرائیور درست طور پر انسٹال ہیں، اور Gazebo کو اس طرح چلائیں:

```bash
# GPU استعمال کے لیے
export OGRE_RTT_MODE=Copy
gazebo --verbose your_world.world
```

## خود سے کریں کا ورکشاپ

### مشق 1: ایک سادہ ماحول

1. ایک نیا Gazebo world تیار کریں جس میں ایک چھوٹا کمرہ ہو
2. اس میں چند obstacles رکھیں
3. TurtleBot3 کو ماحول میں اسپون کریں
4. ٹیلی اوپریٹ کریں تاکہ روبوٹ کمرہ میں سفر کر سکے

### مشق 2: سینسر کی جانچ

1. کیمرہ اور LiDAR سینسرز کو روبوٹ میں شامل کریں
2. سینسر ڈیٹا کو ROS topic کے ذریعے دیکھیں
3. rviz2 استعمال کر کے سینسر ڈیٹا کی ویژنلائزیشن کریں

## خلاصہ

اس ہفتے میں، آپ نے Gazebo ماحول کی بنیاد رکھی ہے اور سیکھا کہ کیسے حقیقی ماحول کی تیاری کی جائے، روبوٹس کو سیمولیٹ کیا جائے اور سینسر ماڈلز بنائے جائیں۔ یہ تمام عناصر Gazebo/Unity MA 2 module میں آپ کے مستقبل کے کام کے لیے بنیاد ہیں۔

[← پچھلا: Gazebo/Unity کا تعارف](./intro.md) | [اگلا: ہفتہ 6: Unity انضمام اور اعلیٰ سینسر](./week6.md) | [MA 2 صفحہ](./intro.md)

اگلے [ہفتہ 6: Unity انضمام اور اعلیٰ سینسر](./week6.md) پر جائیں جہاں آپ Unity کی دنیا میں درآمد کریں گے اور اعلی معیار کے سینسرز کے ماڈلز کو سمجھیں گے۔