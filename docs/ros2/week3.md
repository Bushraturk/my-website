# Week 3: Advanced ROS 2 Concepts

## Navigation

[Week 1-2](./week1-2.md) | [Module Conclusion](./conclusion.md) | [Module Home](./intro.md)

In this final week of the ROS 2 module, we'll explore advanced communication patterns, services, actions, and parameter management that are essential for building complex robotic systems.

## Learning Objectives

By the end of this week, you will be able to:

- Implement service-server communication patterns in ROS 2
- Create and use actions for long-running tasks with feedback
- Manage complex parameters and configurations at runtime
- Implement advanced debugging and profiling techniques
- Design robust architectures for multi-node systems
- Integrate ROS 2 with external systems and hardware

## Advanced Communication Patterns

### Services Deep Dive

Services provide request-reply communication for synchronous operations. Let's explore more complex service usage:

```python
# Advanced service implementation with error handling
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class AdvancedServiceNode(Node):

    def __init__(self):
        super().__init__('advanced_service_node')
        self.srv = self.create_service(
            Trigger, 
            'protected_operation', 
            self.protected_operation_callback
        )
        # Track service usage statistics
        self.call_count = 0

    def protected_operation_callback(self, request, response):
        self.call_count += 1
        self.get_logger().info(f'Service called {self.call_count} times')
        
        try:
            # Simulate a complex operation
            result = self.perform_complex_operation()
            
            if result.success:
                response.success = True
                response.message = f'Operation completed after {result.duration}s'
            else:
                response.success = False
                response.message = f'Operation failed: {result.error_message}'
                
        except Exception as e:
            response.success = False
            response.message = f'Internal error: {str(e)}'
            self.get_logger().error(f'Exception in service: {str(e)}')
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Service interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Actions Deep Dive

Actions are perfect for long-running tasks with feedback and the ability to cancel:

```python
# Advanced action implementation with complex execution
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class AdvancedActionServer(Node):

    def __init__(self):
        super().__init__('advanced_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'advanced_fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Track ongoing goals
        self.active_goals = {}

    def goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        # Accept all goals for this example
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')
        
        # Notify that this goal is active
        self.active_goals[goal_handle.goal_id.uuid] = True
        
        # Create messages for feedback and result
        feedback_msg = Fibonacci.Feedback()
        result_msg = Fibonacci.Result()
        
        # Start with the first two numbers in the sequence
        feedback_msg.sequence = [0, 1]
        
        # Generate the Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if this goal has been cancelled
            if not self.active_goals.get(goal_handle.goal_id.uuid, False):
                # Goal was cancelled
                goal_handle.canceled()
                result_msg.sequence = feedback_msg.sequence
                self.get_logger().info('Goal canceled')
                del self.active_goals[goal_handle.goal_id.uuid]
                return result_msg
            
            # Generate the next number in the sequence
            next_number = feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            feedback_msg.sequence.append(next_number)
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate processing time
            time.sleep(0.1)
        
        # Mark goal as succeeded
        goal_handle.succeed()
        result_msg.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded with sequence: {result_msg.sequence}')
        
        # Remove from active goals
        del self.active_goals[goal_handle.goal_id.uuid]
        
        return result_msg

def main(args=None):
    rclpy.init(args=args)
    action_server = AdvancedActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info('Action server interrupted')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import time
    main()
```

## Advanced Parameter Management

### Parameter Events and Callbacks

Handle parameter changes dynamically:

```python
from rclpy.parameter import Parameter
from rclpy.node import Node

class ParameterManagementNode(Node):

    def __init__(self):
        super().__init__('parameter_management_node')
        
        # Declare parameters with descriptions
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_speed', 1.0, 
                              ParameterDescriptor(description='Maximum robot speed'))
        self.declare_parameter('safety_radius', 0.5)
        
        # Set callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Track parameter history
        self.param_history = {}
    
    def parameter_callback(self, params):
        """Callback for parameter changes."""
        result = SetParametersResult(successful=True)
        
        for param in params:
            if param.name == 'max_speed':
                # Validate parameter
                if not (0.0 < param.value <= 5.0):
                    result.successful = False
                    result.reason = f'Max_speed must be between 0 and 5, got {param.value}'
                    return result
            
            # Store in history
            if param.name not in self.param_history:
                self.param_history[param.name] = []
            self.param_history[param.name].append(param.value)
            
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        
        return result

# Usage example
def main(args=None):
    rclpy.init(args=args)
    node = ParameterManagementNode()
    
    # Change parameters programmatically
    node.set_parameters([Parameter('max_speed', Parameter.Type.DOUBLE, 2.5)])
    
    # Spin for a bit
    rclpy.spin_once(node, timeout_sec=1)
    
    # Print parameter history
    print(f"Parameter history: {node.param_history}")
    
    node.destroy_node()
    rclpy.shutdown()
```

## Hardware Integration

Integrate with real hardware components:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import serial  # For hardware communication

class HardwareIntegrationNode(Node):

    def __init__(self):
        super().__init__('hardware_integration_node')
        
        # Publisher for sending commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Timer for hardware communication
        self.hardware_timer = self.create_timer(0.1, self.hardware_update)
        
        # Connect to hardware
        try:
            self.hardware_interface = serial.Serial('/dev/ttyUSB0', baudrate=115200)
            self.get_logger().info('Connected to hardware interface')
        except serial.SerialException:
            self.get_logger().error('Could not connect to hardware interface')
            self.hardware_interface = None

    def scan_callback(self, msg):
        """Process laser scan data."""
        # Find nearest obstacle
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Nearest obstacle: {min_distance:.2f}m')

    def hardware_update(self):
        """Communicate with hardware."""
        if self.hardware_interface and self.hardware_interface.is_open:
            try:
                # Read sensor data
                data = self.hardware_interface.readline().decode().strip()
                
                # Send processed data to ROS topics
                twist_msg = Twist()
                # Parse data and set speeds
                # ... processing logic here ...
                
                self.cmd_vel_pub.publish(twist_msg)
                
                self.get_logger().info(f'Sent command based on: {data}')
            except Exception as e:
                self.get_logger().error(f'Hardware communication error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HardwareIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down hardware interface')
        if node.hardware_interface:
            node.hardware_interface.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Performance and Profiling

### Memory and CPU Monitoring

Monitor performance metrics in real-time:

```python
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PerformanceMonitorNode(Node):

    def __init__(self):
        super().__init__('performance_monitor')
        
        # Publishers for different metrics
        self.cpu_pub = self.create_publisher(Float32, 'cpu_usage', 10)
        self.memory_pub = self.create_publisher(Float32, 'memory_usage', 10)
        self.ros_rate_pub = self.create_publisher(Float32, 'ros_rate', 10)
        
        # Timer for periodic monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_performance)
        
        # Track ROS loop rate
        self.loop_start_time = self.get_clock().now()
        self.iteration_count = 0

    def monitor_performance(self):
        """Monitor system performance."""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_pub.publish(cpu_msg)
        
        # Memory usage
        memory_info = psutil.virtual_memory()
        memory_msg = Float32()
        memory_msg.data = float(memory_info.percent)
        self.memory_pub.publish(memory_msg)
        
        # Calculate actual ROS loop rate
        current_time = self.get_clock().now()
        elapsed = (current_time - self.loop_start_time).nanoseconds / 1e9
        actual_rate = self.iteration_count / elapsed if elapsed > 0 else 0
        
        rate_msg = Float32()
        rate_msg.data = actual_rate
        self.ros_rate_pub.publish(rate_msg)
        
        # Log warnings for high resource usage
        if cpu_percent > 80:
            self.get_logger().warning(f'High CPU usage: {cpu_percent}%')
        
        if memory_info.percent > 85:
            self.get_logger().warning(f'High memory usage: {memory_info.percent}%')
        
        self.iteration_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Performance monitoring stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Debugging Techniques

### Lifecycle Nodes

Use lifecycle nodes for better control over node states:

```python
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

class LifecycleManagedNode(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_managed_node')
        self.declare_parameter('initial_state', 'inactive')

    def on_configure(self, state):
        """Called when transitioning to configuring state."""
        self.get_logger().info('Configuring node...')
        
        # Initialize resources
        self.publisher = self.create_publisher(String, 'lifecycle_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Disable timer until activated
        self.timer.cancel()
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Called when transitioning to activating state."""
        self.get_logger().info('Activating node...')
        
        # Activate timer
        self.timer.reset()
        
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Called when transitioning to deactivating state."""
        self.get_logger().info('Deactivating node...')
        
        # Deactivate timer
        self.timer.cancel()
        
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Called when transitioning to cleaningup state."""
        self.get_logger().info('Cleaning up node...')
        
        # Destroy resources
        self.destroy_publisher(self.publisher)
        self.destroy_timer(self.timer)
        
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Timer callback function."""
        msg = String()
        msg.data = f'Lifecycle node active at {self.get_clock().now()}'
        self.publisher.publish(msg)
```

## Testing and Quality Assurance

### Unit Testing for ROS 2 Nodes

Write tests for your ROS 2 nodes:

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from simple_node import SimplePublisher  # Your node

class TestSimplePublisher(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = SimplePublisher()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_publisher_has_data(self):
        """Test that publisher is publishing messages."""
        received_messages = []
        
        def callback(msg):
            received_messages.append(msg.data)
        
        # Create subscription to test publisher
        subscription = self.node.create_subscription(
            String,
            'topic',
            callback,
            10
        )
        
        # Spin to allow messages to be published and received
        self.executor.spin_once(timeout_sec=2)
        
        # Check that messages were received
        self.assertGreater(len(received_messages), 0)
        self.assertIn('Hello World', received_messages[0])

if __name__ == '__main__':
    unittest.main()
```

## Practical Exercise: Complete Robot Control Node

Create a robot control node that integrates all concepts:

1. Implement a node that controls robot movement based on sensor input
2. Use services for manual control commands
3. Implement actions for navigation tasks
4. Use parameters for configuration of robot behavior
5. Add performance monitoring and logging

## Homework Assignment

1. Implement a complete robot control system that integrates sensors, actuators, and decision-making
2. Create a state machine using lifecycle nodes to manage robot behavior
3. Write unit tests for your robot control system
4. Profile your system's performance and identify bottlenecks
5. Document your system architecture and design decisions

## Navigation

[← Previous: Week 1-2: ROS 2 Architecture and Fundamentals](./week1-2.md) | [Next: Module Conclusion](./conclusion.md) | [Module Home](./intro.md)

Continue to the [Module Conclusion](./conclusion.md) to review all ROS 2 concepts and prepare for the next module.