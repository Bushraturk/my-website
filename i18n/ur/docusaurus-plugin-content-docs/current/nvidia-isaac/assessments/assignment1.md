---
title: NVIDIA Isaac تفویض 1
---

# NVIDIA Isaac Assignment 1

## تفصیل

اس تفویض کا مقصد perception system تیار کرنا ہے جو Isaac ROS packages کو استعمال کرتا ہو اور NVIDIA Jetson پلیٹ فارم پر چلتا ہو۔

## تعلّم کے اہداف

- Isaac ROS perception packages کو نافذ کرنا
- AI model deployment کے لیے TensorRT کا استعمال کرنا
- Perception pipeline کو ROS 2 nodes کے ساتھ ضم کرنا
- Jetson hardware پر AI inference کو ٹیسٹ کرنا

## تقاضے

- Ubuntu 20.04 اور ROS 2 Humble
- NVIDIA Jetson Xavier NX یا Orin Development Kit
- Isaac ROS packages
- TensorRT for model optimization

## ہدایات

### 1. Perception System تیار کریں (15 نمبر)

Isaac ROS packages کا استعمال کرتے ہوئے perception system تیار کریں:

- AI model کو Isaac ROS DNN Image Encoding package کے ساتھ ضم کریں
- Camera feed کو perception pipeline سے منسلک کریں
- TensorRT کا استعمال کر کے model optimization کریں
- Results کو visualization کے لیے RViz میں publish کریں

### 2. VSLAM Implementation (15 نمبر)

- Isaac ROS Visual SLAM package استعمال کریں
- Robot localization کو ماحول کے نقشے کے ساتھ ضم کریں
- SLAM pipeline کو perception system کے ساتھ انٹیگریٹ کریں
- Map building کو جاری رکھیں اور مقام کاری کی تصدیق کریں

### 3. AI-robot Brain (10 نمبر)

- Perception output کو robot control system کے ساتھ ضم کریں
- Decision-making logic کو نافذ کریں
- AI-brain کو ROS 2 navigation stack کے ساتھ ضم کریں

### 4. Documentation (5 نمبر)

- Perception system کی تفصیل فراہم کریں
- Performance metrics کو record کریں
- چیلنجوں اور ان کے حل کو بیان کریں

## گریڈنگ کا طریقہ

- **Perception System**: 15 - 100% functionality کے ساتھ
- **VSLAM Implementation**: 15 - 100% functionality کے ساتھ
- **AI-robot Integration**: 10 - 100% functionality کے ساتھ
- **Documentation**: 5 - Clear and comprehensive

## جمع کروانے کے اوزار

1. GitHub ریپوزٹری میں اپنے کوڈ کو push کریں
2. تفصیلی رپورٹ جمع کرائیں جس میں:
   - Perception system کا ڈیزائن
   - Performance metrics
   - چیلنجوں اور ان کے حل
   - مستقبل کے لیے تجاویز

### رپورٹ کا ڈھانچہ

```markdown
# Perception System Report

## ڈیزائن

## Results

## چیلنج

## مستقبل کے لیے تجاویز
```

## ڈیڈ لائن

یہ تفویض 15 جنوری 2025 کو ہے۔