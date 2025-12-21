---
title: "ہفتہ 12: کارروائی منصوبہ بندی کے ساتھ LLMs"
sidebar_position: 15
---

# ہفتہ 12: کارروائی منصوبہ بندی کے ساتھ LLMs

اس ہفتے، ہم VLA ماڈلز کو بڑے زبانی ماڈلز (LLMs) کے ساتھ ضم کریں گے تاکہ بصری-زبانی-ایکشن کے ساتھ کارروائیوں کو منصوبہ بند کیا جا سکے۔ یہ کتاب کا آخری ہفتہ ہے، جہاں آپ ایمبیڈیڈ انٹیلی جنس کے لیے ویژن-لینگویج-ایکشن ماڈلز کو ضم کریں گے۔

## سیکھنے کے اہداف

اس ہفتے کے اختتام تک، آپ درج ذیل کر سکیں گے:

- Vision-Language-Action ماڈلز کو LLMs کے ساتھ ضم کرنا
- Natural language instructions کو robot actions میں تبدیل کرنا
- Action planning systems کو نافذ کرنا
- VLA systems کو جامع روبوٹکس ایپلی کیشنز میں ضم کرنا
- VLA systems کی کارکردگی کا جائزہ لینا

## LLMs کے ساتھ VLA انضمام

LLMs کو VLA ماڈلز کے ساتھ ضم کرنا روبوٹکس کے لیے اہم ہے کیونکہ یہ روبوٹس کو قدرتی زبان کے حکم کو سمجھنے کے قابل بناتا ہے:

```python
import openai
from transformers import CLIPProcessor, CLIPModel
import torch

class VLAWithLLM:
    def __init__(self, clip_model_name="openai/clip-vit-base-patch32"):
        # CLIP ماڈل لوڈ کریں
        self.clip_model = CLIPModel.from_pretrained(clip_model_name)
        self.clip_processor = CLIPProcessor.from_pretrained(clip_model_name)
        
        # LLM کا تعلق
        openai.api_key = "your-api-key-here"
    
    def interpret_command(self, image, natural_language_command):
        """بصورتی اور لفظی حکم کی تشریح کریں"""
        
        # تصویر کو بصورتی ویژن سسٹم کے ذریعے سمجھیں
        vision_analysis = self.analyze_vision(image)
        
        # LLM سے حکم کی تشریح حاصل کریں
        action_description = self.llm_interpret_command(vision_analysis, natural_language_command)
        
        # ایکشن تیار کریں
        action = self.generate_action_from_description(action_description)
        
        return action
    
    def analyze_vision(self, image):
        """تصویر کا تجزیہ کریں"""
        inputs = self.clip_processor(images=image, return_tensors="pt")
        with torch.no_grad():
            image_features = self.clip_model.get_image_features(**inputs)
        
        # ایک سادہ تجزیہ ( حقیقی سسٹم میں، یہ پیچیدہ ہوگا)
        return {"objects": ["cup", "table", "robot"], "positions": [(1, 1), (2, 2), (0, 0)]}
    
    def llm_interpret_command(self, vision_analysis, command):
        """LLM کے ذریعے حکم کی تشریح کریں"""
        # LLM کے لیے پرومس یہاں تیار کریں
        prompt = f"""
        Vision Analysis: {vision_analysis}
        Natural Language Command: "{command}"
        
        Interpret the command in the context of the visual scene and provide:
        1. Target object to manipulate
        2. Specific action to perform
        3. Location coordinates if relevant
        4. Any safety considerations
        """
        
        # OpenAI API کال (متبادل کے طور پر Hugging Face ماڈل استعمال کیا جا سکتا ہے)
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=150
        )
        
        return response.choices[0].message.content
    
    def generate_action_from_description(self, description):
        """تشریح سے ایکشن تیار کریں"""
        # تشریح کی بنیاد پر ایکشن ویکٹر تیار کریں
        # (یہ مثال کے لیے سادہ ہے، حقیقی سسٹم میں یہ پیچیدہ ہوگا)
        return [0.1, 0.2, 0.3, 0.1, 0.0, 0.1]  # 6-DOF action

# استعمال کی مثال
vla_llm_system = VLAWithLLM()

# تصویر اور لفظی حکم
image = "robot_environment.jpg"  # در حقیقت، یہ ایک PIL تصویر ہوگی
command = "Please pick up the red cup on the table and place it in the sink."

# ایکشن تیار کریں
action = vla_llm_system.interpret_command(image, command)
print(f"Generated action: {action}")
```

## VLA ماڈلز کے ساتھ Action Planning

VLA ماڈلز کو کارروائی کی منصوبہ بندی کے لیے استعمال کرنا:

```python
class VLAActionPlanner:
    def __init__(self, vla_model):
        self.vla_model = vla_model
        self.action_sequence = []
    
    def create_plan(self, image, instruction):
        """تصویر اور حکم کی بنیاد پر منصوبہ تیار کریں"""
        # کارروائیوں کی منصوبہ بندی کے لیے VLA ماڈل استعمال کریں
        plan = self.vla_model.interpret_command(image, instruction)
        
        # کارروائیوں کی ترتیب تیار کریں
        action_sequence = self.parse_plan_to_actions(plan)
        
        return action_sequence
    
    def parse_plan_to_actions(self, plan):
        """پلان کو کارروائیوں کی ترتیب میں تبدیل کریں"""
        # یہاں پلان کو کارروائیوں میں تبدیل کیا جاتا ہے
        # (مثلاً، "go to location X", "pick up object Y", "move to location Z", "place object Y")
        actions = []
        
        # سادہ تشریح کے لیے، ہم یہ فرض کرتے ہیں کہ ہر کارروائی ایک 6-DOF ویکٹر ہے
        for action_desc in plan.split('\n'):
            if 'approach' in action_desc.lower():
                # تقرب کی کارروائی
                actions.append([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
            elif 'grasp' in action_desc.lower():
                # ت grasping کارروائی
                actions.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.1])
            elif 'move' in action_desc.lower() or 'go to' in action_desc.lower():
                # حرکت کی کارروائی
                actions.append([0.05, 0.05, 0.0, 0.0, 0.0, 0.0])
            elif 'release' in action_desc.lower() or 'place' in action_desc.lower():
                # چھوٹانے کی کارروائی
                actions.append([0.0, 0.0, 0.0, 0.0, 0.0, -0.1])
        
        return actions
    
    def execute_plan(self, action_sequence):
        """کارروائیوں کی ترتیب کو انجام دیں"""
        for action in action_sequence:
            # روبوٹ کو ایکشن بھیجیں
            self.send_action_to_robot(action)
    
    def send_action_to_robot(self, action):
        """کارروائی کو روبوٹ پر بھیجیں"""
        # ROS 2 کے ذریعے کارروائی کو روبوٹ پر بھیجیں
        # (یہاں اس کا تصور کیا جاتا ہے)
        print(f"Sending action to robot: {action}")

# سسٹم کے استعمال کی مثال
vla_planner = VLAActionPlanner(vla_llm_system)

# کارروائیوں کا منصوبہ تیار کریں
image = "workspace_image.jpg"
instruction = "Pick up the red cup on the table and place it in the sink."

action_plan = vla_planner.create_plan(image, instruction)
print(f"Generated action plan: {action_plan}")

# انجام دیں
vla_planner.execute_plan(action_plan)
```

## Safety and Validation

VLA systems کے ساتھ کام کرتے وقت محفوظ عمل کے لیے معتبر ترین ضروریات:

```python
class SafeVLAExecution:
    def __init__(self, vla_model):
        self.vla_model = vla_model
        self.safety_checker = SafetyChecker()
    
    def execute_with_safety(self, image, instruction):
        """محفوظ کارروائی کے ساتھ انجام دیں"""
        # ایکشن تیار کریں
        proposed_action = self.vla_model.interpret_command(image, instruction)
        
        # محفوظ چیک کریں
        if self.safety_checker.is_safe(proposed_action, image):
            # اگر محفوظ ہے تو انجام دیں
            self.execute_action(proposed_action)
        else:
            # اگر محفوظ نہیں تو متبادل منصوبہ استعمال کریں
            self.fallback_plan(proposed_action)
    
    def fallback_plan(self, unsafe_action):
        """غیر محفوظ کارروائی کے لیے متبادل منصوبہ"""
        # محفوظ متبادل کارروائی کو انجام دیں
        safe_action = self.safety_checker.suggest_alternative(unsafe_action)
        self.execute_action(safe_action)

class SafetyChecker:
    def is_safe(self, action, image):
        """کارروائی کی حفاظت کا جائزہ لیں"""
        # سادہ حفاظت چیک (اصلی سسٹم میں یہ پیچیدہ ہوگا)
        if abs(action[0]) > 1.0 or abs(action[1]) > 1.0:
            return False  # کارروائی بہت زیادہ ہے
        return True
    
    def suggest_alternative(self, unsafe_action):
        """غیر محفوظ کارروائی کے لیے متبادل تجویز کریں"""
        # محفوظ کارروائی تیار کریں
        safe_action = [min(max(val, -0.5), 0.5) for val in unsafe_action]  # حد کارروائی
        return safe_action
```

## خلاصہ

اس ہفتے میں، آپ نے Vision-Language-Action ماڈلز کو LLMs کے ساتھ ضم کرنے کے بارے میں سیکھا، جو فزیکل AI کے لیے اہم ہے:

- LLMs کی مدد سے قدرتی زبان کے حکم کی تشریح
- VLA ماڈلز کا استعمال کر کے تصویر اور لفظ کی ایک کے مطابق کارروائی تیار کرنا
- کارروائیوں کی منصوبہ بندی اور انجام دینا
- محفوظ عمل کے لیے توثیق کا اطلاق

اس کتاب کے اس آخری ہفتے کے ساتھ، آپ کے پاس اب 13 ہفتوں کا جامع علم ہے Physical AI اور Humanoid Robotics کے بارے میں۔

[← پچھلا: ہفتہ 10-11: Vision-Language Integration اور Foundation Models](./week10-11.md) | [اگلا: ہفتہ 13: کورس کا اختتام اور کیپسٹون پراجیکٹ](./week13.md) | [ماڈیول کا صفحہ](./intro.md)

اگلے [ہفتہ 13: کورس کا اختتام اور کیپسٹون پراجیکٹ](./week13.md) پر جائیں جہاں آپ کورس کا جائزہ لیں گے اور کیپسٹون پراجیکٹ کا تصور کریں گے۔