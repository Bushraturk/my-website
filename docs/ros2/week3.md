---
title: Week 3 - Advanced ROS 2 Concepts
sidebar_position: 3
---

# Week 3: Advanced ROS 2 Concepts

## Navigation

[Week 1-2](./week1-2.md) | [Module Conclusion](./conclusion.md) | [Module Home](./intro.md)

In this final week of the ROS 2 module, we'll explore advanced communication patterns, services, actions, and parameter management that are essential for building complex robotic systems.

## Learning Objectives

By the end of this week, you will be able to:

- Implement service-server communication patterns in ROS 2
- Create and use actions for goal-oriented tasks with feedback
- Define and implement custom service and action interfaces
- Manage parameters in ROS 2 systems with runtime configuration
- Create and configure launch files to coordinate multiple nodes
- Debug complex ROS 2 systems using command-line tools
- Choose appropriate communication patterns (topics, services, actions) for different use cases
- Integrate multiple ROS 2 concepts in a complete robotic system

## Services in ROS 2

Services provide synchronous request/response communication between nodes. Unlike topics which are asynchronous, services block until a response is received.

![Service Communication Pattern](/img/service-communication.png)

The service communication pattern involves a client making a request to a server, which processes the request and returns a response. This is different from the publisher-subscriber pattern which is asynchronous.

### Creating a Service

First, define the service interface in a `.srv` file:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

This service definition creates a service that accepts two integers (a and b) and returns their sum.

### Service Server Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    """
    A minimal service server node that provides an AddTwoInts service.
    This demonstrates how to create a service server in ROS 2.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_service'
        super().__init__('minimal_service')

        # Create a service that will handle requests to the 'add_two_ints' service
        # The callback method add_two_ints_callback will be called for each request
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        """
        Callback method that is called when a service request is received.
        :param request: The request message containing the two integers to add
        :param response: The response message that will be returned to the client
        :return: The response message with the sum
        """
        # Calculate the sum of the two integers in the request
        response.sum = request.a + request.b

        # Log the incoming request for debugging purposes
        self.get_logger().info(f'Incoming request\na={request.a}, b={request.b}')

        # Return the response
        return response


def main(args=None):
    """Main function to initialize and run the service server node."""
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the MinimalService class
    minimal_service = MinimalService()

    # Keep the node running until it's shut down
    rclpy.spin(minimal_service)

    # Clean up when the node is shut down
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client Implementation

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):
    """
    A minimal service client node that calls the AddTwoInts service.
    This demonstrates how to create a service client in ROS 2.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_client'
        super().__init__('minimal_client')

        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object to hold the parameters for the service call
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service with two integers.
        :param a: First integer
        :param b: Second integer
        :return: The result of the service call
        """
        # Set the values in the request
        self.req.a = a
        self.req.b = b

        # Call the service asynchronously and return a future
        future = self.cli.call_async(self.req)

        # Wait for the result
        rclpy.spin_until_future_complete(self, future)

        # Return the result
        return future.result()


def main(args=None):
    """Main function to initialize and run the service client node."""
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the MinimalClient class
    minimal_client = MinimalClient()

    # Send a request with the first two command line arguments
    if len(sys.argv) < 3:
        print('Usage: ros2 run my_robot_tutorials client_member_function <int1> <int2>')
        return

    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    # Log the result
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    # Clean up when done
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions in ROS 2

Actions are used for goal-oriented communication that may take a long time to complete. They provide feedback during execution and can be canceled.

![Action Communication Pattern](/img/action-communication.png)

### Action Structure

An action has three parts:
- **Goal**: The objective to be reached
- **Feedback**: Status updates during execution
- **Result**: The final outcome

The diagram above illustrates how actions work. The client sends a goal to the action server, which provides feedback during execution. The client can also cancel the goal if needed, and eventually receives the result when the action completes.

### Creating an Action

Define the action interface in a `.action` file:

```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

## Parameters in ROS 2

Parameters allow nodes to be configured at runtime. They can be set at launch time or changed dynamically.

```python
import rclpy
from rclpy.node import Node


class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        self.get_logger().info(f'Robot: {self.robot_name}, Max velocity: {self.max_velocity}')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch Files

Launch files allow you to start multiple nodes with a single command and specify their configuration.

### Creating a Launch File

Create `robot_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='publisher_member_function',
            name='talker',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('/original_topic_name', '/new_topic_name')
            ]
        ),
        Node(
            package='my_robot_tutorials',
            executable='subscriber_member_function',
            name='listener'
        )
    ])
```

## Debugging ROS 2 Systems

### ROS 2 Command Line Tools

- `ros2 node list`: List all active nodes
- `ros2 node info <node_name>`: Get detailed information about a node
- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 service list`: List all available services
- `ros2 action list`: List all available actions

### Visualization Tools

- **RViz2**: 3D visualization for robot sensors and state
- **rqt**: Graphical user interface for ROS tools
- **ros2 bag**: Recording and playback of topic data

## Practical Application

In robotics, advanced ROS 2 concepts are used for:

- Navigation and path planning (using actions)
- Sensor fusion (using services and parameters)
- Multi-robot coordination (complex topic architectures)
- Dynamic reconfiguration (parameters)

## Lab Exercise Preview

In the next section, you'll find the detailed instructions for the advanced ROS 2 lab, where you'll implement a complete robotic system using services, actions, and parameters.

## Summary

In this week, you've learned:

- How to implement service-server communication
- How to use actions for goal-oriented tasks
- How to manage parameters in ROS 2 systems
- How to coordinate nodes with launch files
- Techniques for debugging complex systems

## Navigation

[← Previous: Week 1-2: ROS 2 Architecture](./week1-2.md) | [Next: ROS 2 Module Conclusion](./conclusion.md) | [Module Home](./intro.md)

You've completed the ROS 2 module! Continue to the [Course Introduction](../intro.md) to explore other modules as they become available.