---
title: Week 1-2 - ROS 2 Architecture and Fundamentals
sidebar_position: 2
---

# Week 1-2: ROS 2 Architecture and Fundamentals

## Navigation

[Introduction](./intro.md) | [Week 3](./week3.md) | [Module Home](./intro.md)

In this two-week segment, we'll cover the foundational concepts of ROS 2, including its architecture, nodes, topics, and basic communication patterns.

## Learning Objectives

By the end of this week, you will be able to:

- Explain the ROS 2 client library architecture and its components
- Create and configure a ROS 2 workspace and package
- Implement and execute a simple ROS 2 publisher node
- Develop and run a ROS 2 subscriber node
- Understand the DDS (Data Distribution Service) layer and its role in communication
- Implement publisher-subscriber communication patterns with proper message handling
- Use ROS 2 command-line tools to manage nodes and topics
- Configure Quality of Service (QoS) settings for different communication requirements

## ROS 2 Architecture Overview

ROS 2 is built on a client library architecture that provides multiple language bindings (C++, Python, etc.) to a common underlying middleware. The middleware is typically DDS (Data Distribution Service), which provides the communication infrastructure.

![ROS 2 Architecture Diagram](/img/ros2-architecture.png)

### Key Components

- **Nodes**: The basic unit of execution in ROS 2
- **Communication Primitives**: Topics, Services, Actions
- **DDS Implementation**: The middleware that handles message passing
- **RCL**: ROS Client Library, the abstraction layer
- **RCLCPP/RCLPY**: Language-specific client libraries

The architecture diagram above shows how these components interact. The client libraries (RCLCPP/RCLPY) provide language-specific interfaces to the DDS middleware, which handles the actual message passing between nodes.

![Publisher-Subscriber Communication Pattern](/img/publisher-subscriber-pattern.png)

The publisher-subscriber pattern is fundamental to ROS 2. Publishers send messages to topics, and subscribers receive messages from topics they're subscribed to. The DDS middleware handles the delivery of messages between publishers and subscribers.

## Creating Your First ROS 2 Node

Let's create a simple ROS 2 node in Python that publishes a message to a topic.

### Step 1: Setting Up the Package

First, create a new ROS 2 package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
```

This command creates a new ROS 2 package named `my_robot_tutorials` with the Python build type.

### Step 2: Creating the Publisher Node

Create a file `publisher_member_function.py` in the `my_robot_tutorials/my_robot_tutorials` directory:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal publisher node that publishes messages to a topic.
    This demonstrates the basic structure of a ROS 2 node.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages to the 'topic' topic
        # The second parameter (10) is the queue size
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Set up a timer to call the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of the number of messages published
        self.i = 0

    def timer_callback(self):
        """Callback method that is called every time the timer expires."""
        # Create a new String message
        msg = String()

        # Set the data field of the message
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message to the console
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter for the next message
        self.i += 1


def main(args=None):
    """Main function to initialize and run the publisher node."""
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPublisher class
    minimal_publisher = MinimalPublisher()

    # Keep the node running until it's shut down
    rclpy.spin(minimal_publisher)

    # Clean up when the node is shut down
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

### Step 3: Creating the Subscriber Node

Create a file `subscriber_member_function.py` in the same directory:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    A minimal subscriber node that listens to messages from a topic.
    This demonstrates the basic structure of a ROS 2 subscriber node.
    """

    def __init__(self):
        # Initialize the node with the name 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Create a subscription to the 'topic' topic with String messages
        # The callback method listener_callback will be called when a message is received
        # The second parameter (10) is the queue size
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        """
        Callback method that is called when a message is received on the subscribed topic.
        :param msg: The received message
        """
        # Log the received message to the console
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to initialize and run the subscriber node."""
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the MinimalSubscriber class
    minimal_subscriber = MinimalSubscriber()

    # Keep the node running until it's shut down
    rclpy.spin(minimal_subscriber)

    # Clean up when the node is shut down
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## DDS and Quality of Service (QoS)

DDS (Data Distribution Service) is the middleware that ROS 2 uses for communication. It provides Quality of Service (QoS) policies that allow you to control the behavior of communication between nodes.

![DDS QoS Configuration](/img/dds-qos.png)

### Common QoS Policies

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep all vs. keep last N messages
- **Deadline**: Maximum time between messages
- **Liveliness**: How to determine if a participant is alive

## Hands-On Practice

For this week's lab exercise, you'll implement a publisher-subscriber system that simulates sensor readings from a robot. The publisher will generate simulated sensor data (e.g., temperature readings), and the subscriber will process and log this data.

### Lab Exercise Preview

In the next section, you'll find the detailed instructions for the ROS 2 lab exercise, where you'll:

1. Create a publisher node that simulates sensor data
2. Create a subscriber node that processes the data
3. Experiment with different QoS policies
4. Use ROS 2 tools to visualize and debug your system

## Summary

In this week, you've learned:

- The fundamental architecture of ROS 2
- How to create publisher and subscriber nodes
- The role of DDS in ROS 2 communication
- How to work with basic ROS 2 concepts

## Navigation

[← Previous: Introduction to ROS 2](./intro.md) | [Next: Week 3: Advanced ROS 2 Concepts](./week3.md) | [Module Home](./intro.md)