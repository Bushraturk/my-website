# Week 1-2: ROS 2 Architecture and Fundamentals

## Navigation

[Introduction](./intro.md) | [Week 3](./week3.md) | [Module Home](./intro.md)

In this two-week segment, we'll cover the foundational concepts of ROS 2, including its architecture, nodes, topics, and basic communication patterns.

## Learning Objectives

By the end of this week, you will be able to:

- Explain the ROS 2 client library architecture and its components
- Create and run basic nodes in Python
- Implement publishers and subscribers for asynchronous communication
- Develop services for synchronous request/response communication
- Use actions for long-running tasks with feedback
- Configure parameters at runtime
- Debug and visualize ROS 2 applications

## ROS 2 Architecture Overview

ROS 2 is built on a client library architecture that provides language-specific APIs for robot development. The key components are:

- **DDS Implementation**: Data Distribution Service provides the underlying communication layer
- **ROS Middleware (RMW)**: Abstracts DDS details behind a consistent interface
- **Client Libraries**: Language-specific APIs (rclcpp, rclpy) that simplify programming
- **ROS APIs**: Higher-level abstractions built on client libraries

### DDS (Data Distribution Service)

DDS is a communications protocol and API standard for distributed computing. In ROS 2, DDS implementations provide:

- **Discovery**: Automatic detection of nodes and topics
- **Transport**: Reliable message delivery between components
- **Quality of Service**: Configurable behavior for latency, reliability, etc.
- **Security**: Encryption and authentication for secure communication

### Client Libraries

ROS 2 provides two primary client libraries:

- **rclcpp**: C++ client library with performance-focused design
- **rclpy**: Python client library for easier prototyping and development

## Nodes

Nodes are the fundamental units of computation in ROS 2. Each node represents a single process that performs computation and communicates with other nodes.

### Node Lifecycle

A ROS 2 node typically follows this lifecycle:

1. **Unconfigured**: Node created but not yet configured
2. **Inactive**: Node configured but not yet activated
3. **Active**: Node running and processing callbacks
4. **Finalized**: Node shutting down and destroying resources

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics and Message Passing

Topics provide asynchronous communication between nodes using a publish-subscribe pattern.

### Publishers

A publisher node sends messages to a topic:

```python
import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(String, 'topic_name', 10)
    
    msg = String()
    msg.data = 'Hello from publisher'
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()
```

### Subscribers

A subscriber node receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services

Services provide synchronous request-response communication:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

Actions are used for long-running tasks that provide feedback:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            
            self.get_logger().info('Publishing feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        
        return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

if __name__ == '__main__':
    main()
```

## Parameters

Parameters provide a way to configure nodes at runtime:

```python
import rclpy
from rclpy.node import Node

class ParamNode(Node):

    def __init__(self):
        super().__init__('param_node')
        
        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('other_param', 10)
        
        # Get parameter values
        param_val = self.get_parameter('param_name').get_parameter_value().string_value
        other_val = self.get_parameter('other_param').get_parameter_value().integer_value
        
        self.get_logger().info(f'param_name: {param_val}, other_param: {other_val}')

def main(args=None):
    rclpy.init(args=args)

    param_node = ParamNode()

    rclpy.spin(param_node)

    # Get updated parameter values during runtime
    param_val = param_node.get_parameter('param_name').get_parameter_value().string_value
    param_node.get_logger().info(f'Current param_name: {param_val}')

    param_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS)

QoS profiles control how messages are delivered in terms of reliability and durability:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Create a QoS profile for reliable communication
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# Publisher with specific QoS
publisher = node.create_publisher(String, 'topic_name', qos_profile)

# Subscriber with specific QoS  
subscriber = node.create_subscription(
    String,
    'topic_name',
    callback,
    qos_profile
)
```

## Launch Files

Launch files allow multiple nodes to be started together with specific configurations:

```python
# launch/example_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='minimal_listener',
        ),
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='minimal_talker',
        )
    ])
```

To run: `ros2 launch demo_nodes_py example_launch.py`

## Debugging and Visualization

ROS 2 includes powerful tools for debugging and visualization:

- **rqt**: Graphical user interface for inspecting topics, services, and nodes
- **RViz2**: 3D visualization tool for displaying robot sensor data and state
- **ros2 topic**: Command-line tools for inspecting topics
- **ros2 service**: Command-line tools for using services
- **ros2 bag**: Tools for recording and replaying ROS data

### Common Debugging Commands

```bash
# List all active topics
ros2 topic list

# Echo messages on a topic
ros2 topic echo /topic_name std_msgs/msg/String

# List all active services
ros2 service list

# Call a service
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# List all active nodes
ros2 node list
```

## Practical Exercise: Publisher-Subscriber Pair

Create a simple publisher-subscriber pair to practice node creation and message passing:

1. Create a publisher node that publishes counter values
2. Create a subscriber node that receives and prints the counter values
3. Use rqt_graph to visualize the node connection
4. Use ros2 topic echo to view the published messages

## Homework Assignment

1. Implement a ROS 2 node that subscribes to a sensor topic and publishes processed data to another topic
2. Create a launch file that starts both the sensor simulator and your processing node
3. Use parameters to configure processing behavior at runtime
4. Document the QoS settings used and why they were chosen

## Navigation

[← Previous: ROS 2 Introduction](./intro.md) | [Next: Week 3: Advanced ROS 2 Concepts](./week3.md) | [Module Home](./intro.md)

Continue to [Week 3: Advanced ROS 2 Concepts](./week3.md) to build on these fundamentals with more advanced topics.