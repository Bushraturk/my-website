---
title: Lab Exercise 1 - Basic Node Communication
sidebar_position: 1
---

# Lab Exercise 1: Basic Node Communication in ROS 2

## Objective

In this lab exercise, you will create and run a simple publisher-subscriber system in ROS 2 to understand the fundamental concepts of node communication.

## Learning Objectives

After completing this lab, you will be able to:
- Create a ROS 2 package for your project
- Implement a publisher node that sends custom messages
- Implement a subscriber node that receives and processes messages
- Launch and test your publisher-subscriber system
- Use ROS 2 command-line tools to inspect your nodes

## Prerequisites

- ROS 2 installation (Humble Hawksbill or later)
- Basic Python programming knowledge
- Understanding of ROS 2 concepts (covered in Week 1-2 content)

## Equipment Required

- Computer with ROS 2 installed
- Terminal/shell access
- Text editor or IDE

## Lab Steps

### Step 1: Create a ROS 2 Package

1. Create a new workspace directory:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Create a new ROS 2 package called `robot_basics_lab`:
   ```bash
   ros2 pkg create --build-type ament_python robot_basics_lab
   ```

3. Navigate to the package directory:
   ```bash
   cd robot_basics_lab
   ```

### Step 2: Create the Publisher Node

1. Create a directory for your Python scripts:
   ```bash
   mkdir robot_basics_lab
   cd robot_basics_lab
   ```

2. Create a file called `talker.py` with the following content:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class TalkerNode(Node):

       def __init__(self):
           super().__init__('talker')
           self.publisher_ = self.create_publisher(String, 'chatter', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Hello World: {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1


   def main(args=None):
       rclpy.init(args=args)
       talker = TalkerNode()
       rclpy.spin(talker)
       talker.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 3: Create the Subscriber Node

1. Create a file called `listener.py` with the following content:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class ListenerNode(Node):

       def __init__(self):
           super().__init__('listener')
           self.subscription = self.create_subscription(
               String,
               'chatter',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'I heard: "{msg.data}"')


   def main(args=None):
       rclpy.init(args=args)
       listener = ListenerNode()
       rclpy.spin(listener)
       listener.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 4: Make Scripts Executable and Configure Setup

1. Go back to the package root:
   ```bash
   cd ~/ros2_ws/src/robot_basics_lab
   ```

2. Make the Python files executable:
   ```bash
   chmod +x robot_basics_lab/talker.py
   chmod +x robot_basics_lab/listener.py
   ```

3. Edit the `setup.py` file to add entry points for your scripts:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   
   package_name = 'robot_basics_lab'
   
   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
           (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='Your Name',
       maintainer_email='your.email@example.com',
       description='Robot Basics Lab for ROS 2',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'talker = robot_basics_lab.talker:main',
               'listener = robot_basics_lab.listener:main',
           ],
       },
   )
   ```

### Step 5: Build and Test Your Package

1. Go to your workspace root directory:
   ```bash
   cd ~/ros2_ws
   ```

2. Build your package:
   ```bash
   colcon build --packages-select robot_basics_lab
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Step 6: Run Your Publisher-Subscriber System

1. In one terminal, run the publisher:
   ```bash
   ros2 run robot_basics_lab talker
   ```

2. In another terminal (remember to source setup again), run the subscriber:
   ```bash
   ros2 run robot_basics_lab listener
   ```

3. Observe the messages being published and received.

### Step 7: Explore with ROS 2 Tools

1. In a third terminal, list active nodes:
   ```bash
   ros2 node list
   ```

2. Check the topics being used:
   ```bash
   ros2 topic list
   ```

3. Echo the chatter topic to see the messages:
   ```bash
   ros2 topic echo /chatter
   ```

## Expected Results

- The talker node should publish "Hello World" messages to the `/chatter` topic every 0.5 seconds
- The listener node should receive and print these messages to the terminal
- You should be able to use ROS 2 tools to inspect the nodes and topics in your system

## Troubleshooting

- If you get "command not found" errors, ensure ROS 2 is sourced in your terminal
- If nodes can't communicate, check that they're on the same ROS_DOMAIN_ID
- If the build fails, check that all dependencies are properly declared in your package.xml

## Extension Activities

1. Modify the publisher to send different types of messages (integers, floats)
2. Add a second publisher or subscriber to your system
3. Experiment with different Quality of Service (QoS) settings

## Assessment Questions

1. What is the purpose of the `create_publisher` method?
2. How does the timer callback work in the publisher node?
3. Why do we need to call `rclpy.spin()` in the main function?
4. What would happen if you changed the timer period to 2 seconds?

## Summary

This lab introduced you to creating basic ROS 2 nodes for communication. You've successfully implemented a publisher-subscriber pattern, which is one of the fundamental communication methods in ROS 2.