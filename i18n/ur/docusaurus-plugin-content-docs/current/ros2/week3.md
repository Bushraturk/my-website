---
title: "ہفتہ 3: ROS 2 کے اعلیٰ تصورات"
sidebar_position: 4
---

# ہفتہ 3: ROS 2 کے اعلیٰ تصورات

## نیوی گیشن

[ہفتہ 1-2](./week1-2.md) | [ماڈیول کا اختتام](./conclusion.md) | [ماڈیول ہوم](./intro.md)

اس ROS 2 ماڈیول کے آخری ہفتے میں، ہم اعلیٰ کمیونیکیشن پیٹرنز، سروسز، ایکشنز، اور پیرامیٹر مینجمنٹ کو استعمال کریں گے جو کمپلیکس روبوٹکس سسٹم تیار کرنے کے لیے ضروری ہیں۔

## سیکھنے کے اہداف

اس ہفتے کے اختتام تک، آپ درج ذیل کر سکیں گے:

- ROS 2 میں سروس-سرور کمیونیکیشن پیٹرنز نافذ کرنا
- فیڈبیک کے ساتھ طویل المدت کاموں کے لیے ایکشنز بنانا اور استعمال کرنا
- رن ٹائم پر پیچیدہ پیرامیٹرز اور کنفیگریشنز کا انتظام کرنا
- کمپلیکس مواصلاتی ایرکی ٹیکچر میں شامل ہونا
- ROS 2 ٹولز کا استعمال کر کے ڈیبگنگ اور مانیٹرنگ کرنا

## ایڈوانسڈ کمیونیکیشن پیٹرنز

### سروسز اور سروسز کلینٹز

Services ایک synchronous request/response کمیونیکیشن پیٹرن ہے:

```python
# server.py
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
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# client.py
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ایکشنز

Actions طویل المدت کاموں کے لیے استعمال ہوتے ہیں جن میں feedback اور cancelation کی ضرورت ہوتی ہے:

```python
# action_server.py
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
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# action_client.py
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
        self.get_logger().info('Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## پیرامیٹرز مینجمنٹ

### پیرامیٹر کی اقسام

ROS 2 parameters کई data types کی معاونت کرتے ہیں:

- integers
- floats
- strings
- booleans
- lists (arrays)

### پیرامیٹر ڈیکلریشن اور استعمال

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')
        
        # Parameters declare کریں
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('sensors_list', ['camera', 'lidar', 'imu'])
        
        # Parameters کو access کریں
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.safety_enabled = self.get_parameter('safety_enabled').get_parameter_value().bool_value
        self.sensors_list = self.get_parameter('sensors_list').get_parameter_value().string_array_value

    def update_parameter_callback(self, parameter_list):
        """Parameters کے تبدیلیوں کو handle کریں"""
        for param in parameter_list:
            if param.name == 'max_speed' and param.type_ == Parameter.Type.PARAMETER_DOUBLE:
                if 0.0 < param.value <= 5.0:  # Validation
                    self.get_logger().info(f'Updating max_speed to {param.value}')
                    self.max_speed = param.value
                else:
                    self.get_logger().warn(f'Invalid max_speed value: {param.value}')
        
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    
    # Parameters کے تبدیلیوں کے لیے callback add کریں
    node.add_on_set_parameters_callback(node.update_parameter_callback)
    
    # Parameters کو CLI کے ذریعے set کریں یا launch files میں define کریں
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### لانچ فائلز میں Parameters

```python
# launch/parameter_example.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='parameter_node',
            name='parameter_node',
            parameters=[
                {'robot_name': 'husky'},
                {'max_speed': 2.0},
                {'safety_enabled': True},
                {'sensors_list': ['camera', 'lidar']},
            ],
            output='screen',
        ),
    ])
```

## کمپلیکس مواصلاتی ایرکی ٹیکچر

### نوڈ گروپس

متعلقہ نوڈز کو ایک process میں چلانا:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String

class ComplexNode(Node):
    def __init__(self):
        super().__init__('complex_node')
        
        # Different callback groups for different threads
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        
        # Publishers and subscribers
        self.pub = self.create_publisher(String, 'result', 10)
        self.sub1 = self.create_subscription(
            String, 'input1', self.subscription1_callback, 10, 
            callback_group=self.group1)
        self.sub2 = self.create_subscription(
            String, 'input2', self.subscription2_callback, 10, 
            callback_group=self.group2)
        
        # Timer for periodic tasks
        self.timer = self.create_timer(0.5, self.periodic_task, callback_group=self.group1)

    def subscription1_callback(self, msg):
        self.get_logger().info(f'Received on input1: {msg.data}')
        # Process data from input1

    def subscription2_callback(self, msg):
        self.get_logger().info(f'Received on input2: {msg.data}')
        # Process data from input2

    def periodic_task(self):
        self.get_logger().info('Periodic task executed')
        # Perform periodic tasks

def main(args=None):
    rclpy.init(args=args)
    
    node = ComplexNode()
    
    # Multi-threaded executor to handle multiple callback groups
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ڈیبگنگ اور ٹولز

### ROS 2 CLI ٹولز

کچھ اہم ROS 2 ڈیبگنگ ٹولز:

```bash
# تمام نوڈز کو دیکھیں
ros2 node list

# نوڈ کی معلومات حاصل کریں
ros2 node info /node_name

# تمام ٹوپکس کو دیکھیں
ros2 topic list

# کسی ٹوپک پر messages دیکھیں
ros2 topic echo /topic_name

# کسی ٹوپک پر message publish کریں
ros2 topic pub /topic_name std_msgs/msg/String "data: 'hello'"

# خدمات کو دیکھیں
ros2 service list

# کسی سروس کو کال کریں
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# ایکشنز کو دیکھیں
ros2 action list

# parameters دیکھیں
ros2 param list

# parameter کو پڑھیں
ros2 param get /node_name param_name
```

### rqt ٹولز

ROS 2 کے لیے مفید GUI ٹولز:

- **rqt_graph**: نوڈ اور ٹوپک کنیکشن کو دیکھیں
- **rqt_console**: لاگ میسجز کو دیکھیں
- **rqt_plot**: ٹوپک کے ڈیٹا کو پلاٹ کریں
- **rqt_bag**: سابیق session کے ڈیٹا کو دیکھیں

## عملی ورکشاپ

### مشق 1: متعدد نوڈ کمیونیکیشن

ایک سسٹم بنائیں جس میں:

1. ایک sensor نوڈ جو sensor data generate کرے
2. ایک processor نوڈ جو sensor data process کرے
3. ایک controller نوڈ جو commands generate کرے
4. ایک monitor نوڈ جو system status دکھائے

### مشق 2: ایکشن کے ساتھ طویل المدت کام

ایک navigation action node بنائیں جو:

- گوئل کو قبول کرے
- feedback کو periodic طور پر publish کرے
- result کو complete کرنے پر provide کرے
- cancelation کو handle کرے

## خود سے کریں کا سرکار

### مسئلہ: معیاری روبوٹ کنٹرول سسٹم

ایک سسٹم ڈیزائن کریں جو:

- Sensor fusion perform کرے (camera, lidar, imu)
- Path planning perform کرے
- Navigation commands generate کرے
- Parameters at runtime تبدیل کر سکے

### حل کے اجزا:

1. ایک sensor_fusion_node
2. ایک path_planning_node 
3. ایک navigation_node
4. Launch file تیار کریں جو تمام نوڈز کو start کرے
5. Parameters کنفیگ فائل تیار کریں

```yaml
# config/robot_control.yaml
sensor_fusion_node:
  ros__parameters:
    camera_topic: "/camera/image_raw"
    lidar_topic: "/scan"
    fusion_frequency: 10.0
    enable_filtering: true

path_planning_node:
  ros__parameters:
    planner_type: "dijkstra"
    map_resolution: 0.05
    inflation_radius: 0.3

navigation_node:
  ros__parameters:
    controller_frequency: 20.0
    max_linear_velocity: 0.5
    max_angular_velocity: 1.0
    enable_obstacle_avoidance: true
```

## خلاصہ

ROS 2 کے اعلیٰ تصورات کے ساتھ، آپ کمپلیکس روبوٹکس سسٹمز ڈیزائن اور نافذ کر سکتے ہیں:

- **Services**: Request/response communication کے لیے
- **Actions**: Feedback اور cancelation کے ساتھ طویل کاموں کے لیے
- **Parameters**: Runtime configuration کے لیے
- **Complex architectures**: Multi-threaded processing کے لیے

یہ تمام اوزار آپ کو اسکیل ایبل اور robust robotic systems تیار کرنے میں مدد دیتے ہیں۔

[← پچھلا: ہفتہ 1-2: ROS 2 آرکیٹیکچر اور بنیادیات](./week1-2.md) | [اگلا: ماڈیول کا اختتام](./conclusion.md) | [ماڈیول ہوم](./intro.md)

ماڈیول کے [اختتام](./conclusion.md) پر جائیں جہاں آپ ROS 2 ماڈیول کا جائزہ لے سکیں گے اور اس کے اہم نکات کا جائزہ لے سکیں گے۔