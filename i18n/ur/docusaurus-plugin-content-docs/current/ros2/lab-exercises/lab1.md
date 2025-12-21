---
title: لیب 1 - بنیادی Node Communication
sidebar_position: 1
---

# لیب 1: ROS 2 میں بنیادی Node Communication

## مقصد

اس لیب میں، آپ ROS 2 میں ایک سادہ publisher-subscriber سسٹم بنائیں گے تاکہ node communication کے بنیادی تصورات سمجھ سکیں۔

## سیکھنے کے مقاصد

اس لیب کے بعد آپ:
- اپنے پروجیکٹ کے لیے ROS 2 پیکج بنا سکیں گے
- کسٹم messages بھیجنے والا publisher node لاگو کر سکیں گے
- Messages وصول کرنے والا subscriber node لاگو کر سکیں گے
- اپنا publisher-subscriber سسٹم launch اور test کر سکیں گے
- اپنے nodes کا معائنہ کرنے کے لیے ROS 2 command-line tools استعمال کر سکیں گے

## پیش شرائط

- ROS 2 انسٹالیشن (Humble Hawksbill یا بعد کا)
- بنیادی Python programming کا علم
- ROS 2 تصورات کی سمجھ (ہفتہ 1-2 میں شامل)

## ضروری آلات

- ROS 2 انسٹال شدہ کمپیوٹر
- Terminal/shell تک رسائی
- Text editor یا IDE

## لیب کے مراحل

### مرحلہ 1: ROS 2 پیکج بنائیں

1. نئی workspace directory بنائیں:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. `robot_basics_lab` نامی نیا ROS 2 پیکج بنائیں:
   ```bash
   ros2 pkg create --build-type ament_python robot_basics_lab
   ```

3. پیکج directory میں جائیں:
   ```bash
   cd robot_basics_lab
   ```

### مرحلہ 2: Publisher Node بنائیں

1. Python scripts کے لیے directory بنائیں:
   ```bash
   mkdir robot_basics_lab
   cd robot_basics_lab
   ```

2. `talker.py` نامی فائل بنائیں:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String


   class TalkerNode(Node):

       def __init__(self):
           super().__init__('talker')
           self.publisher_ = self.create_publisher(String, 'chatter', 10)
           timer_period = 0.5  # سیکنڈز
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Hello World: {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'شائع ہو رہا: "{msg.data}"')
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

### مرحلہ 3: Subscriber Node بنائیں

1. `listener.py` نامی فائل بنائیں:
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

       def listener_callback(self, msg):
           self.get_logger().info(f'میں نے سنا: "{msg.data}"')


   def main(args=None):
       rclpy.init(args=args)
       listener = ListenerNode()
       rclpy.spin(listener)
       listener.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()
   ```

### مرحلہ 4: Scripts Executable بنائیں

1. پیکج root پر جائیں:
   ```bash
   cd ~/ros2_ws/src/robot_basics_lab
   ```

2. Python files executable بنائیں:
   ```bash
   chmod +x robot_basics_lab/talker.py
   chmod +x robot_basics_lab/listener.py
   ```

3. `setup.py` فائل میں entry points شامل کریں:
   ```python
   entry_points={
       'console_scripts': [
           'talker = robot_basics_lab.talker:main',
           'listener = robot_basics_lab.listener:main',
       ],
   },
   ```

### مرحلہ 5: Build اور Test کریں

1. Workspace root directory میں جائیں:
   ```bash
   cd ~/ros2_ws
   ```

2. پیکج build کریں:
   ```bash
   colcon build --packages-select robot_basics_lab
   ```

3. Setup file source کریں:
   ```bash
   source install/setup.bash
   ```

### مرحلہ 6: سسٹم چلائیں

1. ایک terminal میں publisher چلائیں:
   ```bash
   ros2 run robot_basics_lab talker
   ```

2. دوسرے terminal میں subscriber چلائیں:
   ```bash
   ros2 run robot_basics_lab listener
   ```

3. Messages کا تبادلہ دیکھیں۔

### مرحلہ 7: ROS 2 Tools سے Explore کریں

```bash
# Active nodes کی فہرست
ros2 node list

# Topics کی فہرست
ros2 topic list

# Chatter topic پر messages دیکھیں
ros2 topic echo /chatter
```

## متوقع نتائج

- Talker node ہر 0.5 سیکنڈ میں "Hello World" messages publish کرے
- Listener node یہ messages وصول اور print کرے
- ROS 2 tools سے nodes اور topics کا معائنہ کر سکیں

## مسائل کا حل

- اگر "command not found" error آئے تو ROS 2 source کریں
- اگر nodes communicate نہیں کر رہے تو ROS_DOMAIN_ID چیک کریں
- اگر build fail ہو تو package.xml میں dependencies چیک کریں

## اضافی سرگرمیاں

1. Publisher میں مختلف قسم کے messages بھیجیں (integers, floats)
2. دوسرا publisher یا subscriber شامل کریں
3. مختلف QoS settings کے ساتھ تجربہ کریں

## خلاصہ

اس لیب نے آپ کو ROS 2 میں بنیادی nodes بنانا سکھایا۔ آپ نے publisher-subscriber pattern کامیابی سے لاگو کیا جو ROS 2 میں بنیادی communication طریقوں میں سے ایک ہے۔
