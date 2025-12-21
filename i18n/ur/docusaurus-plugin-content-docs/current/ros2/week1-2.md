---
title: ہفتہ 1-2 - ROS 2 آرکیٹیکچر اور بنیادیات
sidebar_position: 2
---

# ہفتہ 1-2: ROS 2 آرکیٹیکچر اور بنیادیات

## نیویگیشن

[تعارف](./intro.md) | [ہفتہ 3](./week3.md) | [ماڈیول ہوم](./intro.md)

ان دو ہفتوں میں، ہم ROS 2 کے بنیادی تصورات کا احاطہ کریں گے، بشمول اس کی آرکیٹیکچر، nodes، topics، اور بنیادی communication patterns۔

## سیکھنے کے مقاصد

اس ہفتے کے اختتام تک، آپ:

- ROS 2 client library آرکیٹیکچر اور اس کے اجزاء کی وضاحت کر سکیں گے
- Python میں بنیادی nodes بنا اور چلا سکیں گے
- Asynchronous communication کے لیے publishers اور subscribers لاگو کر سکیں گے
- Synchronous request/response communication کے لیے services تیار کر سکیں گے
- طویل چلنے والے tasks کے لیے feedback کے ساتھ actions استعمال کر سکیں گے
- Runtime پر parameters کنفیگر کر سکیں گے
- ROS 2 ایپلیکیشنز debug اور visualize کر سکیں گے

## ROS 2 آرکیٹیکچر کا جائزہ

ROS 2 ایک client library آرکیٹیکچر پر بنایا گیا ہے جو روبوٹ ڈیولپمنٹ کے لیے language-specific APIs فراہم کرتی ہے۔ اہم اجزاء ہیں:

- **DDS Implementation**: Data Distribution Service بنیادی communication layer فراہم کرتی ہے
- **ROS Middleware (RMW)**: DDS تفصیلات کو ایک مستقل interface کے پیچھے چھپاتا ہے
- **Client Libraries**: Language-specific APIs (rclcpp, rclpy) جو programming آسان بناتی ہیں
- **ROS APIs**: Client libraries پر بنی higher-level abstractions

### DDS (Data Distribution Service)

DDS distributed computing کے لیے ایک communications protocol اور API standard ہے۔ ROS 2 میں، DDS implementations فراہم کرتی ہیں:

- **Discovery**: Nodes اور topics کا خودکار پتہ لگانا
- **Transport**: اجزاء کے درمیان قابل اعتماد message delivery
- **Quality of Service**: Latency، reliability وغیرہ کے لیے قابل ترتیب رویہ
- **Security**: محفوظ communication کے لیے encryption اور authentication

## Nodes

Nodes ROS 2 میں computation کی بنیادی اکائیاں ہیں۔ ہر node ایک process کی نمائندگی کرتا ہے جو computation کرتا ہے اور دوسرے nodes کے ساتھ بات چیت کرتا ہے۔

### Node Lifecycle

ایک ROS 2 node عام طور پر اس lifecycle کی پیروی کرتا ہے:

1. **Unconfigured**: Node بنایا گیا لیکن ابھی configured نہیں
2. **Inactive**: Node configured لیکن ابھی activated نہیں
3. **Active**: Node چل رہا ہے اور callbacks process کر رہا ہے
4. **Finalized**: Node بند ہو رہا ہے اور resources destroy کر رہا ہے

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # سیکنڈز
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

## Topics اور Message Passing

Topics publish-subscribe pattern استعمال کرتے ہوئے nodes کے درمیان asynchronous communication فراہم کرتی ہیں۔

### Publishers

ایک publisher node ایک topic پر messages بھیجتا ہے:

```python
import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(String, 'topic_name', 10)

    msg = String()
    msg.data = 'Publisher سے Hello'
    publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()
```

### Subscribers

ایک subscriber node ایک topic سے messages وصول کرتا ہے:

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

    def listener_callback(self, msg):
        self.get_logger().info('میں نے سنا: "%s"' % msg.data)

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

Services synchronous request-response communication فراہم کرتی ہیں:

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
        self.get_logger().info('آنے والی درخواست\na: %d b: %d' % (request.a, request.b))
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

Actions طویل چلنے والے tasks کے لیے استعمال ہوتی ہیں جو feedback فراہم کرتی ہیں:

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
        self.get_logger().info('Goal انجام دیا جا رہا ہے...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Feedback شائع ہو رہا: {feedback_msg.sequence}')
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

Parameters runtime پر nodes کو configure کرنے کا طریقہ فراہم کرتی ہیں:

```python
import rclpy
from rclpy.node import Node

class ParamNode(Node):

    def __init__(self):
        super().__init__('param_node')

        # Default values کے ساتھ parameters declare کریں
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('other_param', 10)

        # Parameter values حاصل کریں
        param_val = self.get_parameter('param_name').get_parameter_value().string_value
        other_val = self.get_parameter('other_param').get_parameter_value().integer_value

        self.get_logger().info(f'param_name: {param_val}, other_param: {other_val}')

def main(args=None):
    rclpy.init(args=args)
    param_node = ParamNode()
    rclpy.spin(param_node)
    param_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS)

QoS profiles کنٹرول کرتی ہیں کہ messages reliability اور durability کے لحاظ سے کیسے deliver ہوں:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Reliable communication کے لیے QoS profile بنائیں
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# مخصوص QoS کے ساتھ Publisher
publisher = node.create_publisher(String, 'topic_name', qos_profile)

# مخصوص QoS کے ساتھ Subscriber
subscriber = node.create_subscription(String, 'topic_name', callback, qos_profile)
```

## Launch Files

Launch files متعدد nodes کو مخصوص configurations کے ساتھ ایک ساتھ شروع کرنے کی اجازت دیتی ہیں:

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

چلانے کے لیے: `ros2 launch demo_nodes_py example_launch.py`

## Debugging اور Visualization

ROS 2 میں debugging اور visualization کے لیے طاقتور tools شامل ہیں:

- **rqt**: Topics، services، اور nodes کا معائنہ کرنے کے لیے graphical interface
- **RViz2**: Robot sensor data اور state دکھانے کے لیے 3D visualization tool
- **ros2 topic**: Topics کا معائنہ کرنے کے لیے command-line tools
- **ros2 service**: Services استعمال کرنے کے لیے command-line tools
- **ros2 bag**: ROS data record اور replay کرنے کے لیے tools

### عام Debugging Commands

```bash
# تمام active topics کی فہرست
ros2 topic list

# ایک topic پر messages echo کریں
ros2 topic echo /topic_name std_msgs/msg/String

# تمام active services کی فہرست
ros2 service list

# ایک service call کریں
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# تمام active nodes کی فہرست
ros2 node list
```

## عملی مشق: Publisher-Subscriber جوڑا

Node creation اور message passing کی مشق کے لیے ایک سادہ publisher-subscriber جوڑا بنائیں:

1. ایک publisher node بنائیں جو counter values publish کرے
2. ایک subscriber node بنائیں جو counter values وصول اور print کرے
3. Node connection visualize کرنے کے لیے rqt_graph استعمال کریں
4. Published messages دیکھنے کے لیے ros2 topic echo استعمال کریں

## ہوم ورک اسائنمنٹ

1. ایک ROS 2 node لاگو کریں جو sensor topic کو subscribe کرے اور processed data کو دوسرے topic پر publish کرے
2. ایک launch file بنائیں جو sensor simulator اور آپ کا processing node دونوں شروع کرے
3. Runtime پر processing رویے کو configure کرنے کے لیے parameters استعمال کریں
4. استعمال شدہ QoS settings اور ان کے انتخاب کی وجہ دستاویز کریں

## نیویگیشن

[← پچھلا: ROS 2 تعارف](./intro.md) | [اگلا: ہفتہ 3: ایڈوانسڈ ROS 2 تصورات](./week3.md) | [ماڈیول ہوم](./intro.md)

ان بنیادیات پر مزید ایڈوانسڈ topics کے ساتھ بنانے کے لیے [ہفتہ 3: ایڈوانسڈ ROS 2 تصورات](./week3.md) پر جاری رکھیں۔
