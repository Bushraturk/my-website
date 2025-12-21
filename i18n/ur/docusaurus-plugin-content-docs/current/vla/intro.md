---
title: Vision-Language-Action (VLA) Models کا تعارف
sidebar_position: 13
---

# Vision-Language-Action (VLA) Models کا تعارف

میں آپ کا خیر مقدم کرتا ہوں ماڈیول 4، Vision-Language-Action (VLA) Models کے ماڈیول میں، جو امبیڈیڈ انٹیلی جنس اور روبوٹکس کے جدید کنارے کو کور کرتا ہے۔ آپ VLA ماڈلز کے بارے میں سیکھیں گے جو تصویر، زبان، اور کارروائی کو ضم کرتے ہیں تاکہ روبوٹس قدرتی زبان کے حکم کو ایڈاپٹ کر سکیں اور جسمانی دنیا میں کارروائی کر سکیں۔

## سیکھنے کے اہداف

اس ماڈیول کے اختتام تک، آپ درج ذیل کر سکیں گے:

- ویژن-لینگویج-ایکشن ماڈلز کے تصورات کو سمجھنا
- امبیڈیڈ انٹیلی جنس ایپلی کیشنز کے لیے VLA ماڈلز کو نافذ کرنا
- قدرتی زبان کے حکم کو عملی کارروائیوں میں تبدیل کرنا
- ایڈاپٹیو روبوٹکس سسٹم تیار کرنا
- AI-robot coordination کی جانچ کرنا

## Vision-Language-Action (VLA) کا تعارف

VLA ماڈلز ایک نئی نسل کے AI ماڈلز ہیں جو تصویر، زبان، اور کارروائی کو ایک ہی ٹرانسفارمر آرکیٹیکچر میں ضم کرتے ہیں۔ یہ ماڈلز ایمبیڈیڈ انٹیلی جنس کے لیے اہم ہیں کیونکہ یہ روبوٹس کو اس طرح کے ان پٹ کے ساتھ کام کرنے کے قابل بناتے ہیں جو قدرتی زبان کے حکم، بصارتی ان پٹ، اور جسمانی کارروائیوں کو ضم کرتے ہیں۔

### VLA ماڈل کی خصوصیات

- **Multi-Modal Integration**: تصویر، زبان، اور ایکشن کو ایک ہی ماڈل میں ضم کرنا
- **End-to-End Trainable**: قدرتی ان پٹ سے کارروائی تک کا سیمی مappings
- **Generalization**: مختلف ماحول اور کاموں کے لیے عام طور پر استعمال کرنا
- **Embodied Learning**: فیزیکل دنیا کے ساتھ تعامل کے ذریعے سیکھنا

### OpenVLA

OpenVLA ایک کھلی ذمہ داری ہے VLA ماڈل جو امبیڈیڈ انٹیلی جنس کے لیے ہے:

```bash
# OpenVLA کو انسٹال کرنا
pip install openvla-py
```

```python
import torch
from transformers import AutoModel, AutoProcessor

# OpenVLA ماڈل لوڈ کرنا
model = AutoModel.from_pretrained("openvla/openvla-9b")
processor = AutoProcessor.from_pretrained("openvla/openvla-9b")

# تصویر اور لفظی حکم کا استعمال کر کے ایکشن کے لیے پریڈکٹ کرنا
def predict_action(image, instruction):
    inputs = processor(image, instruction, return_tensors="pt")
    with torch.no_grad():
        action = model(**inputs).logits.argmax(dim=-1)
    return action
```

## VLA سسٹم کا انضمام

VLA سسٹم کو نافذ کرنے کے لیے، ہمیں تصویر، زبان، اور ایکشن کے ماڈلز کو ضم کرنا ہے:

```python
import cv2
import numpy as np
import torch
from transformers import CLIPProcessor, CLIPModel
from diffusers import StableDiffusionPipeline

class VLASystem:
    def __init__(self):
        # CLIP ماڈل تصویر اور لفظی ان پٹ کو ضم کرنے کے لیے
        self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
        self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
        
        # VAE ماڈل تصویر کو مربوط کرنے کے لیے
        self.vae = None  # Robot's visual system
        
        # Action planning model
        self.action_planner = None  # Task planning model
    
    def process_command(self, image, command):
        """تصویر اور لفظی حکم کو ضم کر کے ایکشن کی منصوبہ بندی کریں"""
        
        # تصویر اور لفظی ان پٹ کو ضم کریں
        inputs = self.clip_processor(images=image, text=command, return_tensors="pt", padding=True)
        
        # CLIP ماڈل کے ذریعے تصویر اور لفظی ان پٹ کو ضم کریں
        outputs = self.clip_model(**inputs)
        
        # مناسب ایکشن تیار کریں
        action = self.generate_action(outputs)
        
        return action
    
    def generate_action(self, embeddings):
        """Embedded representation کے ذریعے ایکشن تیار کریں"""
        # This would involve a more complex action generation model
        # Simplified for demonstration
        action_vector = np.random.rand(7)  # 7-DOF robot arm action
        return action_vector

# مثال: VLA سسٹم کو استعمال کرنا
vla_system = VLASystem()

# تصویر لوڈ کریں
image = cv2.imread('robot_scene.jpg')
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# لفظی حکم
command = "Pick up the red cup on the table"

# ایکشن تیار کریں
action = vla_system.process_command(image_rgb, command)
```

## VLA اور Robot Control

VLA ماڈل کو اصل روبوٹ کنٹرول میں ضم کرنا:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class VLARobotController(Node):
    def __init__(self):
        super().__init__('vla_robot_controller')
        
        # VLA سسٹم کو شروع کریں
        self.vla_system = VLASystem()
        self.cv_bridge = CvBridge()
        
        # سبسکرائبرز
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(String, 'voice_command', self.command_callback, 10)
        
        # پبلشر
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # آخری حکم اور تصویر کو محفوظ کریں
        self.last_image = None
        self.last_command = None
        
    def image_callback(self, msg):
        self.last_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        
        # اگر حکم بھی دستیاب ہو تو ایکشن تیار کریں
        if self.last_command:
            self.process_vla_input()
    
    def command_callback(self, msg):
        self.last_command = msg.data
        
        # اگر تصویر بھی دستیاب ہو تو ایکشن تیار کریں
        if self.last_image:
            self.process_vla_input()
    
    def process_vla_input(self):
        if self.last_image is not None and self.last_command is not None:
            # VLA ماڈل کا استعمال کریں
            action = self.vla_system.process_command(self.last_image, self.last_command)
            
            # ایکشن کو روبوٹ میں ضابط کریں
            cmd_vel_msg = self.convert_action_to_cmd_vel(action)
            self.cmd_vel_pub.publish(cmd_vel_msg)
    
    def convert_action_to_cmd_vel(self, action):
        # VLA ایکشن کو cmd_vel_msg میں تبدیل کریں
        cmd_vel = Twist()
        cmd_vel.linear.x = float(action[0])
        cmd_vel.angular.z = float(action[-1])
        return cmd_vel

def main(args=None):
    rclpy.init(args=args)
    
    vla_controller = VLARobotController()
    
    try:
        rclpy.spin(vla_controller)
    except KeyboardInterrupt:
        pass
    finally:
        vla_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## VLA تربیت کا عمل

VLA ماڈلز کی تربیت کے لیے، ہمیں بصورت، لفظی، اور ایکشن ڈیٹا کے ٹرائیوں کی ضرورت ہوتی ہے:

```python
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

class VLADataset(Dataset):
    def __init__(self, image_paths, texts, actions):
        self.image_paths = image_paths
        self.texts = texts
        self.actions = actions
    
    def __len__(self):
        return len(self.image_paths)
    
    def __getitem__(self, idx):
        # تصویر لوڈ کریں
        image = cv2.imread(self.image_paths[idx])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # لفظی حکم
        text = self.texts[idx]
        
        # ایکشن
        action = self.actions[idx]
        
        return {
            'image': torch.tensor(image, dtype=torch.float32),
            'text': text,
            'action': torch.tensor(action, dtype=torch.float32)
        }

def train_vla_model(model, dataloader, epochs=10):
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    criterion = nn.MSELoss()
    
    for epoch in range(epochs):
        model.train()
        total_loss = 0
        
        for batch in dataloader:
            optimizer.zero_grad()
            
            # فارورڈ پاس
            predicted_actions = model(batch['image'], batch['text'])
            
            # نقصان کا حساب
            loss = criterion(predicted_actions, batch['action'])
            
            # بیک ورڈ پاس
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
        
        print(f'Epoch [{epoch+1}/{epochs}], Loss: {total_loss/len(dataloader):.4f}')

# استعمال کی مثال
dataset = VLADataset(image_paths, text_commands, robot_actions)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

# VLA ماڈل تربیت دیں
train_vla_model(vla_model, dataloader)
```

## خلاصہ

VLA ماڈلز فزیکل AI کے لیے ایک اہم قدم ہیں کیونکہ یہ ایمبیڈیڈ انٹیلی جنس کے لیے بصورت، لفظی، اور ایکشن کو ضم کرتے ہیں۔ یہ ماڈلز روبوٹس کو قدرتی زبان کے حکم کو سمجھنے اور جسمانی دنیا میں مناسب کارروائی کرنے کے قابل بناتے ہیں۔

اس ماڈیول میں، آپ نے VLA ماڈلز کے تصورات، انضمام، اور نافذ کاری کے بارے میں سیکھا۔

[← پچھلا: VLA کا تعارف](./intro.md) | [اگلا: ہفتہ 10-11: Vision-Language Integration اور Foundation Models](./week10-11.md) | [ماڈیول کا صفحہ](./intro.md)

[ہفتہ 10-11: Vision-Language Integration اور Foundation Models](./week10-11.md) پر جائیں جہاں آپ تصویر-زبان انضمام کے بارے میں سیکھیں گے۔