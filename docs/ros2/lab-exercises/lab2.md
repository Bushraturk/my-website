---
title: Lab Exercise 2 - Services and Actions in ROS 2
sidebar_position: 2
---

# Lab Exercise 2: Services and Actions in ROS 2

## Objective

In this lab exercise, you will implement service-server and action-based communication in ROS 2 to understand advanced communication patterns used in robotics applications.

## Learning Objectives

After completing this lab, you will be able to:
- Create and implement a ROS 2 service server and client
- Implement a ROS 2 action server and client
- Compare the use cases for services versus actions
- Understand when to use each communication pattern

## Prerequisites

- ROS 2 installation (Humble Hawksbills or later)
- Basic Python programming knowledge
- Understanding of ROS 2 advanced concepts (covered in Week 3 content)
- Completion of Lab Exercise 1 (Basic Node Communication)

## Equipment Required

- Computer with ROS 2 installed
- Terminal/shell access
- Text editor or IDE

## Lab Steps

### Step 1: Create a ROS 2 Package

1. Navigate to your workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new ROS 2 package called `robot_advanced_lab`:
   ```bash
   ros2 pkg create --build-type ament_python robot_advanced_lab
   ```

3. Navigate to the package directory:
   ```bash
   cd robot_advanced_lab
   ```

### Step 2: Create the Service Files

1. Create directories for your code:
   ```bash
   mkdir robot_advanced_lab
   cd robot_advanced_lab
   ```

2. Create a service server called `math_service.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import AddTwoInts


   class MathService(Node):

       def __init__(self):
           super().__init__('math_service')
           self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

       def add_callback(self, request, response):
           response.sum = request.a + request.b
           self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')
           return response


   def main(args=None):
       rclpy.init(args=args)
       math_service = MathService()
       rclpy.spin(math_service)
       math_service.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

3. Create a service client called `math_client.py`:
   ```python
   import sys
   import rclpy
   from rclpy.node import Node
   from example_interfaces.srv import AddTwoInts


   class MathClient(Node):

       def __init__(self):
           super().__init__('math_client')
           self.client = self.create_client(AddTwoInts, 'add_two_ints')
           while not self.client.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Service not available, waiting again...')
           self.request = AddTwoInts.Request()

       def send_request(self, a, b):
           self.request.a = a
           self.request.b = b
           future = self.client.call_async(self.request)
           return future


   def main(args=None):
       rclpy.init(args=args)
       
       if len(sys.argv) != 3:
           print('Usage: ros2 run robot_advanced_lab math_client <int1> <int2>')
           return
       
       math_client = MathClient()
       future = math_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
       
       while rclpy.ok():
           rclpy.spin_once(math_client)
           if future.done():
               try:
                   response = future.result()
                   math_client.get_logger().info(f'Result: {response.sum}')
               except Exception as e:
                   math_client.get_logger().info(f'Service call failed: {e}')
               break

       math_client.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### Step 3: Create the Action Files

1. Create an action server called `fibonacci_action_server.py`:
   ```python
   import time
   import rclpy
   from rclpy.action import ActionServer, CancelResponse, GoalResponse
   from rclpy.node import Node
   from example_interfaces.action import Fibonacci


   class FibonacciActionServer(Node):

       def __init__(self):
           super().__init__('fibonacci_action_server')
           self._action_server = ActionServer(
               self,
               Fibonacci,
               'fibonacci',
               execute_callback=self.execute_callback,
               goal_callback=self.goal_callback,
               cancel_callback=self.cancel_callback)

       def destroy(self):
           self._action_server.destroy()
           super().destroy_node()

       def goal_callback(self, goal_request):
           self.get_logger().info('Received goal request')
           return GoalResponse.ACCEPT

       def cancel_callback(self, goal_handle):
           self.get_logger().info('Received cancel request')
           return CancelResponse.ACCEPT

       def execute_callback(self, goal_handle):
           self.get_logger().info('Executing goal...')
           
           feedback_msg = Fibonacci.Feedback()
           feedback_msg.sequence = [0, 1]
           
           for i in range(1, goal_handle.request.order):
               if goal_handle.is_cancel_requested:
                   goal_handle.canceled()
                   self.get_logger().info('Goal canceled')
                   return Fibonacci.Result()

               feedback_msg.sequence.append(
                   feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
               
               self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
               goal_handle.publish_feedback(feedback_msg)
               
               time.sleep(1)

           goal_handle.succeed()
           result = Fibonacci.Result()
           result.sequence = feedback_msg.sequence
           return result


   def main(args=None):
       rclpy.init(args=args)
       action_server = FibonacciActionServer()
       rclpy.spin(action_server)
       action_server.destroy()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

2. Create an action client called `fibonacci_action_client.py`:
   ```python
   import time
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from example_interfaces.action import Fibonacci


   class FibonacciActionClient(Node):

       def __init__(self):
           super().__init__('fibonacci_action_client')
           self._action_client = ActionClient(
               self,
               Fibonacci,
               'fibonacci')

       def send_goal(self, order):
           goal_msg = Fibonacci.Goal()
           goal_msg.order = order

           self._action_client.wait_for_server()

           self._send_goal_future = self._action_client.send_goal_async(
               goal_msg,
               feedback_callback=self.feedback_callback)

           self._send_goal_future.add_done_callback(self.goal_response_callback)

       def goal_response_callback(self, future):
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Goal rejected :(')
               return

           self.get_logger().info('Goal accepted :)')

           self._get_result_future = goal_handle.get_result_async()
           self._get_result_future.add_done_callback(self.get_result_callback)

       def feedback_callback(self, feedback_msg):
           feedback = feedback_msg.feedback
           self.get_logger().info(f'Received feedback: {feedback.sequence}')

       def get_result_callback(self, future):
           result = future.result().result
           self.get_logger().info(f'Result: {result.sequence}')
           rclpy.shutdown()


   def main(args=None):
       rclpy.init(args=args)
       action_client = FibonacciActionClient()
       action_client.send_goal(10)
       rclpy.spin(action_client)


   if __name__ == '__main__':
       main()
   ```

### Step 4: Make Scripts Executable and Configure Setup

1. Go back to the package root:
   ```bash
   cd ~/ros2_ws/src/robot_advanced_lab
   ```

2. Make the Python files executable:
   ```bash
   chmod +x robot_advanced_lab/math_service.py
   chmod +x robot_advanced_lab/math_client.py
   chmod +x robot_advanced_lab/fibonacci_action_server.py
   chmod +x robot_advanced_lab/fibonacci_action_client.py
   ```

3. Edit the `setup.py` file to add entry points for your scripts:
   ```python
   import os
   from glob import glob
   from setuptools import setup
   
   package_name = 'robot_advanced_lab'
   
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
       description='Robot Advanced Lab for ROS 2 - Services and Actions',
       license='TODO: License declaration',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'math_service = robot_advanced_lab.math_service:main',
               'math_client = robot_advanced_lab.math_client:main',
               'fibonacci_action_server = robot_advanced_lab.fibonacci_action_server:main',
               'fibonacci_action_client = robot_advanced_lab.fibonacci_action_client:main',
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
   colcon build --packages-select robot_advanced_lab
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Step 6: Test Your Service Implementation

1. In one terminal, run the service server:
   ```bash
   ros2 run robot_advanced_lab math_service
   ```

2. In another terminal (remember to source setup again), run the client:
   ```bash
   ros2 run robot_advanced_lab math_client 2 3
   ```

3. You should see the server process the request and the client receive the result.

### Step 7: Test Your Action Implementation

1. In one terminal, run the action server:
   ```bash
   ros2 run robot_advanced_lab fibonacci_action_server
   ```

2. In another terminal (remember to source setup again), run the client:
   ```bash
   ros2 run robot_advanced_lab fibonacci_action_client
   ```

3. Observe the feedback messages during execution and the final result.

## Expected Results

- The math service should take two integers as input and return their sum
- The Fibonacci action should generate a Fibonacci sequence of the specified order
- You should receive feedback during the action execution
- Both implementations should demonstrate different communication patterns in ROS 2

## Troubleshooting

- If services or actions don't connect, ensure all terminals have sourced the workspace setup
- If you get import errors, verify that the example_interfaces package is installed
- If the action client terminates before completion, ensure you're not accidentally ending the program too soon

## Extension Activities

1. Modify the service to perform multiple mathematical operations (add, subtract, multiply)
2. Create an action that simulates a robot moving to a goal position with feedback on progress
3. Implement a parameter server that can dynamically adjust service behavior

## Assessment Questions

1. What is the main difference between services and actions in ROS 2?
2. When would you use a service instead of a topic for communication?
3. What are the three components of an action message?
4. Explain the purpose of feedback in ROS 2 actions.

## Summary

This lab introduced you to advanced communication patterns in ROS 2. You've implemented both services for synchronous request/response communication and actions for goal-oriented tasks with feedback. These patterns are essential for building complex robotic systems that require coordination between multiple components.