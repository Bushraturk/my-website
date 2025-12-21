---
title: لیب ایکسرسائز - ویژن-لینگویج-ایکشن انٹیگریشن
sidebar_position: 21
---

# لیب ایکسرسائز: ویژن-لینگویج-ایکشن انٹیگریشن

## مقصد

اس لیب ایکسرسائز میں، آپ ایک مکمل Vision-Language-Action (VLA) سسٹم لاگو کریں گے جو قدرتی زبان کے احکامات کی تشریح کر سکے، ماحول کو سمجھ سکے، اور مناسب روبوٹک ایکشنز انجام دے سکے۔ آپ ایک ایسا سسٹم بنائیں گے جو کورس کے دوران سیکھے گئے تمام تصورات کو ایک کام کرنے والے embodied AI ایجنٹ میں یکجا کرے۔

## سیکھنے کے مقاصد

اس لیب کو مکمل کرنے کے بعد، آپ:
- ویژن، لینگویج، اور ایکشن اجزاء کو ایک متحد سسٹم میں ضم کر سکیں گے
- قدرتی زبان کے احکامات کو قابل عمل روبوٹ ایکشنز میں تبدیل کر سکیں گے
- زبان کو ادراک سے جوڑنے کے grounding میکانزم لاگو کر سکیں گے
- سمولیٹڈ اور/یا فزیکل ماحول میں VLA سسٹم کی کارکردگی کا جائزہ لے سکیں گے
- ملٹی-موڈل سسٹمز میں عام انٹیگریشن چیلنجز حل کر سکیں گے

## پیش شرائط

- تمام پچھلے ماڈیولز کی تکمیل (ROS 2، Gazebo/Unity، NVIDIA Isaac، VLA)
- کیمرہ اور بنیادی manipulation صلاحیتوں والے روبوٹ پلیٹ فارم تک رسائی (سمولیٹڈ یا فزیکل)
- VLA ماڈل آرکیٹیکچرز اور ٹریننگ اپروچز کی سمجھ
- ROS 2 میسجنگ اور ایکشن سرورز کا تجربہ

## ضروری آلات

- کیمرہ اور manipulation صلاحیت والا روبوٹ پلیٹ فارم (سمولیٹڈ یا فزیکل)
- NVIDIA GPU اور CUDA سپورٹ والا کمپیوٹر
- انسٹال شدہ ROS 2، Isaac ROS پیکجز، اور VLA ماڈل dependencies
- Gazebo سمولیشن ماحول (اگر سمولیشن استعمال کر رہے ہیں)

## لیب کے مراحل

### مرحلہ 1: ماحول کی ترتیب اور تصدیق

1. تمام ضروری سافٹ ویئر اجزاء کی تصدیق کریں:
   ```bash
   # ROS 2 انسٹالیشن چیک کریں
   printenv | grep ROS

   # Isaac ROS پیکجز کی تصدیق کریں
   ros2 pkg list | grep isaac

   # GPU کی دستیابی چیک کریں
   nvidia-smi

   # VLA ماڈل dependencies کی تصدیق کریں
   python -c "import transformers; import torch; print('Dependencies OK')"
   ```

2. انٹیگریشن پروجیکٹ کے لیے workspace سیٹ اپ کریں:
   ```bash
   mkdir -p ~/vla_integration_ws/src
   cd ~/vla_integration_ws/src
   git clone https://github.com/nvidia/cuvl_benchmark.git
   cd ..
   colcon build --symlink-install
   source install/setup.bash
   ```

### مرحلہ 2: VLA سسٹم آرکیٹیکچر لاگو کریں

مین انٹیگریشن نوڈ بنائیں جو زبان کی سمجھ کو ایکشن ایگزیکیوشن سے جوڑے:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from transformers import CLIPProcessor, CLIPModel
from typing import List, Dict, Any, Optional


class VLASystemNode(Node):
    """
    Vision-Language-Action سسٹم جو قدرتی زبان کے احکامات کی تشریح کرتا ہے،
    ماحول کو سمجھتا ہے، اور مناسب روبوٹک ایکشنز انجام دیتا ہے۔
    """
    def __init__(self):
        super().__init__('vla_system_node')

        # امیج پروسیسنگ کے لیے CV bridge شروع کریں
        self.cv_bridge = CvBridge()

        # ادراک کے اجزاء شروع کریں
        self.setup_perception()

        # زبان پروسیسنگ کے اجزاء شروع کریں
        self.setup_language_processing()

        # ضروری topics کو subscribe کریں
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.command_subscription = self.create_subscription(
            String, '/natural_language_command', self.command_callback, 10
        )

        # سسٹم سٹیٹس کے لیے publisher
        self.status_publisher = self.create_publisher(String, '/vla_system/status', 10)

        self.latest_image = None
        self.detected_objects = []

        self.get_logger().info("VLA سسٹم کامیابی سے شروع ہو گیا")

    def setup_perception(self):
        """ادراک کے اجزاء سیٹ اپ کریں"""
        try:
            self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
            self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
            self.get_logger().info("CLIP ماڈل لوڈ ہو گیا")
        except Exception as e:
            self.get_logger().warn(f"CLIP ماڈل لوڈ نہیں ہو سکا: {e}")

    def setup_language_processing(self):
        """زبان پروسیسنگ کے اجزاء شروع کریں"""
        self.object_keywords = {
            "water bottle": ["water", "bottle", "پانی", "بوتل"],
            "cup": ["cup", "mug", "کپ", "پیالہ"],
            "book": ["book", "کتاب"],
        }

    def image_callback(self, msg: Image):
        """کیمرہ تصاویر پروسیس کریں"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"تصویر پروسیسنگ میں خرابی: {e}")

    def command_callback(self, msg: String):
        """قدرتی زبان کے حکم کو پروسیس کریں"""
        command = msg.data
        self.get_logger().info(f"حکم موصول: '{command}'")

        parsed = self.parse_language_command(command)
        if parsed:
            success = self.execute_action_plan(parsed)
            status = "کامیاب" if success else "ناکام"
            self.get_logger().info(f"حکم {status}: {command}")

    def parse_language_command(self, command: str) -> Optional[Dict[str, Any]]:
        """قدرتی زبان کے حکم کو پارس کریں"""
        command_lower = command.lower()

        action = None
        if any(w in command_lower for w in ["go to", "navigate", "جاؤ"]):
            action = "navigate"
        elif any(w in command_lower for w in ["pick up", "grasp", "اٹھاؤ"]):
            action = "pick"
        elif any(w in command_lower for w in ["put", "place", "رکھو"]):
            action = "place"

        return {"action": action, "command": command} if action else None

    def execute_action_plan(self, action_plan: Dict[str, Any]) -> bool:
        """ایکشن پلان انجام دیں"""
        action = action_plan.get("action")
        self.get_logger().info(f"ایکشن انجام دیا جا رہا ہے: {action}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = VLASystemNode()
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

### مرحلہ 3: سسٹم کی جانچ

1. روبوٹ سمولیشن شروع کریں:
   ```bash
   ros2 launch your_robot_gazebo your_robot_world.launch.py
   ```

2. VLA سسٹم شروع کریں:
   ```bash
   ros2 run vla_integration_pkg vla_system_node
   ```

3. ٹیسٹ کمانڈز بھیجیں:
   ```bash
   ros2 topic pub /natural_language_command std_msgs/String "data: 'Go to the water bottle'"
   ros2 topic pub /natural_language_command std_msgs/String "data: 'Pick up the cup'"
   ```

### مرحلہ 4: کارکردگی کا جائزہ

اپنے سسٹم کا جائزہ لیں:

| میٹرک | وضاحت |
|-------|--------|
| کامیابی کی شرح | کامیاب احکامات کا فیصد |
| درستگی | اشیاء کی درست شناخت |
| جوابی وقت | پروسیسنگ کی رفتار |
| مضبوطی | مشکل حالات میں کارکردگی |

## لیب رپورٹ

جمع کروائیں:
1. سسٹم ڈیزائن کی وضاحت
2. نفاذ کی تفصیلات
3. جانچ کے نتائج
4. درپیش چیلنجز
5. بہتری کی تجاویز

## عام مسائل

| مسئلہ | حل |
|-------|-----|
| GPU میموری ختم | batch size کم کریں |
| اشیاء نہیں پہچانی جا رہیں | روشنی بہتر کریں |
| احکامات پارس نہیں ہو رہے | کی ورڈز بڑھائیں |

## خلاصہ

اس لیب میں آپ نے ایک مکمل VLA سسٹم بنایا جو قدرتی زبان کے احکامات سمجھ کر روبوٹک ایکشنز انجام دیتا ہے۔ یہ embodied AI کا عملی مظاہرہ ہے۔
