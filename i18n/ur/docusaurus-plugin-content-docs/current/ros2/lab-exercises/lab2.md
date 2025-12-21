---
title: لیب 2 - Services اور Actions
sidebar_position: 2
---

# لیب 2: ROS 2 میں Services اور Actions

## مقصد

اس لیب میں، آپ ROS 2 میں service اور action-based communication لاگو کریں گے تاکہ روبوٹکس ایپلیکیشنز میں استعمال ہونے والے ایڈوانسڈ communication patterns سمجھ سکیں۔

## سیکھنے کے مقاصد

اس لیب کے بعد آپ:
- ROS 2 service server اور client بنا سکیں گے
- ROS 2 action server اور client لاگو کر سکیں گے
- Services اور actions کے use cases کا موازنہ کر سکیں گے
- ہر communication pattern کب استعمال کرنا ہے یہ سمجھ سکیں گے

## پیش شرائط

- ROS 2 انسٹالیشن (Humble Hawksbill یا بعد کا)
- لیب 1 کی تکمیل
- ہفتہ 3 کے ایڈوانسڈ تصورات کی سمجھ

## لیب کے مراحل

### مرحلہ 1: پیکج بنائیں

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_advanced_lab
cd robot_advanced_lab
mkdir robot_advanced_lab
```

### مرحلہ 2: Service Server بنائیں

`math_service.py` فائل بنائیں:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MathService(Node):

    def __init__(self):
        super().__init__('math_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('Math Service شروع ہو گیا')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'درخواست: {request.a} + {request.b} = {response.sum}')
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

### مرحلہ 3: Service Client بنائیں

`math_client.py` فائل بنائیں:

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
            self.get_logger().info('Service دستیاب نہیں، انتظار...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        future = self.client.call_async(self.request)
        return future


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('استعمال: ros2 run robot_advanced_lab math_client <int1> <int2>')
        return

    math_client = MathClient()
    future = math_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    while rclpy.ok():
        rclpy.spin_once(math_client)
        if future.done():
            response = future.result()
            math_client.get_logger().info(f'نتیجہ: {response.sum}')
            break

    math_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### مرحلہ 4: Action Server بنائیں

`fibonacci_action_server.py` فائل بنائیں:

```python
import time
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
            execute_callback=self.execute_callback)
        self.get_logger().info('Fibonacci Action Server شروع ہو گیا')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Goal انجام دیا جا رہا ہے...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal منسوخ ہو گیا')
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### مرحلہ 5: Action Client بنائیں

`fibonacci_action_client.py` فائل بنائیں:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal مسترد ہو گیا')
            return

        self.get_logger().info('Goal قبول ہو گیا')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'نتیجہ: {result.sequence}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

### مرحلہ 6: Build اور Test کریں

```bash
cd ~/ros2_ws
colcon build --packages-select robot_advanced_lab
source install/setup.bash
```

### مرحلہ 7: Service Test کریں

Terminal 1:
```bash
ros2 run robot_advanced_lab math_service
```

Terminal 2:
```bash
ros2 run robot_advanced_lab math_client 5 3
```

### مرحلہ 8: Action Test کریں

Terminal 1:
```bash
ros2 run robot_advanced_lab fibonacci_action_server
```

Terminal 2:
```bash
ros2 run robot_advanced_lab fibonacci_action_client
```

## متوقع نتائج

- Math service دو integers لے کر ان کا جوڑ واپس کرے
- Fibonacci action مخصوص order کی Fibonacci sequence بنائے
- Action execution کے دوران feedback ملے

## Services vs Actions

| Service | Action |
|---------|--------|
| Synchronous | Asynchronous |
| فوری جواب | طویل ٹاسکس |
| کوئی feedback نہیں | Feedback ملتا ہے |
| Cancel نہیں ہو سکتا | Cancel ہو سکتا ہے |

## خلاصہ

اس لیب نے آپ کو ROS 2 میں ایڈوانسڈ communication patterns سکھائے۔ آپ نے services اور actions دونوں لاگو کیے جو پیچیدہ روبوٹک سسٹمز بنانے کے لیے ضروری ہیں۔
