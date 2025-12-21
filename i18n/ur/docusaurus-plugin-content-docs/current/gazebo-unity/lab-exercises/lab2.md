---
title: Gazebo/Unity لیب مشق 2
---

# Gazebo/Unity لیب مشق 2

## وضاحت

اس لیب مشق کا مقصد ایک مکمل سیمولیٹڈ روبوٹ سسٹم تیار کرنا ہے جو سینسر ڈیٹا کو سنبھال سکے اور اس کے مطابق حرکت کر سکے۔ آپ Gazebo یا Unity میں ایک معیاری روبوٹ (جیسے TurtleBot3) کو اسپون کریں گے اور اس پر متحرک کنٹرول الگورتھم نافذ کریں گے۔

## تعلّم کے اہداف

- معیاری روبوٹ ماڈلز کا استعمال
- متحرک سینسر ڈیٹا کی تنصیف
- کنٹرول الگورتھم کی نفاذ کاری
- سیمولیشن کا تجزیہ

## تقاضے

- Ubuntu 20.04 اور ROS 2 Humble
- Gazebo Classic یا Unity 2021.3 LTS
- TurtleBot3 packages (اگر Gazebo استعمال کر رہے ہیں)
- Unity Robotics Package (اگر Unity استعمال کر رہے ہیں)

## ہدایات

### 1. معیاری روبوٹ کا استعمال

Gazebo میں TurtleBot3 کو اسپون کریں:

```bash
# TurtleBot3 ماڈل کو سیٹ کریں
export TURTLEBOT3_MODEL=burger

# خالی دنیا میں TurtleBot3 کو اسپون کریں
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 2. LiDAR ڈیٹا کا استعمال

LiDAR ڈیٹا کو سنبھالیں:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # LiDAR سے ڈیٹا حاصل کرنے کے لیے سبسکرائبر
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # روبوٹ کو حرکت دینے کے لیے پبلشر
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # ٹائمر برائے کنٹرول لوپ
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # ریکارڈ کیا ہوا ڈیٹا
        self.lidar_data = None
        self.obstacle_detected = False

    def scan_callback(self, msg):
        """LiDAR اسکین ڈیٹا کو ہینڈل کریں"""
        self.lidar_data = msg.ranges
        
        # رکاوٹ کی جانچ 
        min_distance = min(self.lidar_data[:90] + self.lidar_data[-90:])  # آگے کی طرف دیکھنا
        self.obstacle_detected = min_distance < 1.0  # 1 میٹر سے کم فاصلہ پر رکاوٹ

    def control_loop(self):
        """کنٹرول الگورتھم لوپ"""
        if self.lidar_data is None:
            return
            
        cmd_vel_msg = Twist()
        
        if self.obstacle_detected:
            # گھومائیں
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.5
        else:
            # سیدھا چلائیں
            cmd_vel_msg.linear.x = 0.5
            cmd_vel_msg.angular.z = 0.0
            
        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. ہارڈویئر انضمام کی تیاری

سیمولیشن اور حقیقی ہارڈویئر کے درمیان منتقلی کی تیاری:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import yaml

class SimulationToHardwareAdapter(Node):

    def __init__(self, config_file=None):
        super().__init__('simulation_to_hardware_adapter')
        
        # کنفیگریشن لوڈ کریں
        if config_file:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = {
                'linear_scaling': 1.0,
                'angular_scaling': 1.0,
                'min_linear_speed': 0.0,
                'max_linear_speed': 1.0,
                'min_angular_speed': -1.0,
                'max_angular_speed': 1.0
            }
        
        # سبسکرائبر اور پبلشر
        self.sim_scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sim_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # اضافی: ہارڈویئر کے لیے ٹاپکس
        # self.hw_cmd_pub = self.create_publisher(HardwareCmd, '/hw_cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.lidar_data = None

    def scan_callback(self, msg):
        self.lidar_data = msg

    def control_loop(self):
        """کنٹرول الگورتھم لوپ"""
        if self.lidar_data is None:
            return
            
        # کنٹرول الگورتھم
        cmd_vel_msg = self.apply_control_algorithm(self.lidar_data)
        
        # اسکیلنگ ایڈجسٹ کریں
        scaled_cmd = Twist()
        scaled_cmd.linear.x = cmd_vel_msg.linear.x * self.config['linear_scaling']
        scaled_cmd.angular.z = cmd_vel_msg.angular.z * self.config['angular_scaling']
        
        # حدیں لگائیں
        scaled_cmd.linear.x = max(
            self.config['min_linear_speed'], 
            min(scaled_cmd.linear.x, self.config['max_linear_speed'])
        )
        scaled_cmd.angular.z = max(
            self.config['min_angular_speed'], 
            min(scaled_cmd.angular.z, self.config['max_angular_speed'])
        )
        
        # کمانڈ پبلش کریں
        self.sim_cmd_pub.publish(scaled_cmd)

    def apply_control_algorithm(self, lidar_data):
        """کنٹرول الگورتھم نافذ کریں"""
        # یہاں کنٹرول الگورتھم نافذ کریں
        cmd_vel = Twist()
        
        # مثال: رکاوٹ سے گریز
        front_distances = lidar_data.ranges[:30] + lidar_data.ranges[-30:]
        min_front_dist = min(front_distances)
        
        if min_front_dist < 0.8:
            cmd_vel.angular.z = 0.5  # گھومنا
        else:
            cmd_vel.linear.x = 0.3  # آگے بڑھنا
        
        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    node = SimulationToHardwareAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. RViz2 کا استعمال

RViz2 کو تیار کریں تاکہ سیمولیشن کا تجزیہ کیا جا سکے:

1. RViz2 لانچ کریں:

```bash
ros2 run rviz2 rviz2
```

2. RViz2 میں مندرجہ ذیل شامل کریں:
   - RobotModel: `/robot_description` 
   - LaserScan: `/scan`
   - TF: `/tf`
   - Odometry: `/odom`

3. فیمل کو `turtlebot3_world` سیٹ کریں

## جائزہ سوالات

1. TurtleBot3 کا کیا فائدہ ہے؟
2. LiDAR ڈیٹا کو کیسے تیز کیا جا سکتا ہے؟
3. سیمولیشن اور حقیقی ہارڈویئر کے درمیان کیا فرق ہے؟
4. کنٹرول الگورتھم کو کیسے تبدیل کیا جا سکتا ہے؟

## تکمیل کی مثالیں

- [TurtleBot3 simulation tutorials](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- [ROS 2 navigation with TurtleBot3](https://navigation.ros.org/tutorials/docs/get_back_to_first_index.html)

## نتائج

اس لیب مشق سے، آپ نے ایک معیاری روبوٹ کا استعمال، سینسر ڈیٹا کو سنبھالنا، اور کنٹرول الگورتھم نافذ کرنا سیکھا ہے۔