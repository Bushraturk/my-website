---
title: انسٹرکٹر گائیڈ - Troubleshooting گائیڈ
sidebar_position: 26
---

# انسٹرکٹر گائیڈ - Troubleshooting گائیڈ

یہ جامع troubleshooting گائیڈ فزیکل AI اور ہیومنوائڈ روبوٹکس کورس کے دوران پیش آنے والے عام تکنیکی مسائل کا حل کرتا ہے۔

## عمومی Troubleshooting اپروچ

### منظم مسئلہ حل کرنے کے مراحل
1. **مسئلہ شناخت کریں**: واضح کریں کہ کیا کام نہیں کر رہا
2. **مسئلہ دوہرائیں**: تصدیق کریں کہ مسئلہ مستقل طور پر دوہرایا جا سکتا ہے
3. **اجزاء الگ کریں**: تعین کریں کون سا سسٹم جزو مسئلہ پیدا کر رہا ہے
4. **پیش شرائط چیک کریں**: تصدیق کریں کہ تمام dependencies صحیح ہیں
5. **حل تحقیق کریں**: دستاویزات اور کمیونٹی وسائل سے مشورہ کریں
6. **حل لاگو کریں**: سب سے مناسب حل لگائیں
7. **حل کی تصدیق کریں**: تصدیق کریں کہ مسئلہ مکمل طور پر حل ہو گیا ہے

## ROS 2 Troubleshooting

### عام Node مسائل

#### Node شروع نہیں ہو رہا
**علامات**:
- Node واضح error message کے بغیر شروع ہونے میں ناکام
- Node شروع ہوتا ہے لیکن `ros2 node list` میں ظاہر نہیں ہوتا

**تشخیصی مراحل**:
```bash
# Syntax errors چیک کریں
python3 -m py_compile your_node.py

# ROS 2 environment تصدیق کریں
echo $ROS_DISTRO

# بڑھتی ہوئی verbosity کے ساتھ چلائیں
python3 your_node.py --ros-args --log-level debug
```

**حل**:
- یقینی بنائیں workspace sourced ہے: `source install/setup.bash`
- Python syntax یا import errors درست کریں
- گمشدہ package dependencies انسٹال کریں

#### Topic Communication ناکامیاں
**علامات**:
- Publishers اور subscribers communicate نہیں کر رہے
- Messages موصول نہیں ہو رہے

**تشخیصی مراحل**:
```bash
ros2 topic list
ros2 topic info /your_topic
```

**حل**:
- یقینی بنائیں topic names بالکل میل کھاتے ہیں
- Message types consistent تصدیق کریں
- Publisher اور subscriber کے درمیان QoS profiles ملائیں

### کارکردگی کے مسائل

#### زیادہ CPU استعمال
**علامات**: ROS 2 nodes ضرورت سے زیادہ CPU وسائل استعمال کر رہے ہیں

**حل**:
- Control loops میں مناسب sleep statements شامل کریں
- Message publishing کے لیے rate limiters استعمال کریں
- بہتر computational efficiency کے لیے algorithms optimize کریں

## Simulation Environment مسائل (Gazebo/Unity)

### Gazebo Troubleshooting

#### Graphics Rendering مسائل
**علامات**:
- Gazebo interface سیاہ یا بگڑا ہوا ظاہر ہوتا ہے
- Textures صحیح load نہیں ہو رہے

**حل**:
- GPU drivers تازہ ترین version میں اپڈیٹ کریں
- مناسب graphics libraries انسٹال کریں: `sudo apt install mesa-utils`
- Headless environments کے لیے software rendering استعمال کریں

#### Physics Simulation مسائل
**علامات**: Objects غیر حقیقت پسندانہ رویہ کر رہے ہیں

**حل**:
- World file میں physics engine parameters ایڈجسٹ کریں
- Model collision اور inertial properties تصدیق کریں

## NVIDIA Isaac مسائل

### Detection ناکامیاں
**علامات**: Object detection کام نہیں کر رہا یا غلط نتائج

**حل**:
- Isaac calibration tools استعمال کرتے ہوئے camera calibration تصدیق کریں
- Detection thresholds اور parameters ایڈجسٹ کریں
- بہتر detection کے لیے lighting conditions بہتر بنائیں

### SLAM سسٹم مسائل
**علامات**: Map building ناکام یا غلط، Robot localization drifting

**حل**:
- ماحول میں کافی texture اور features یقینی بنائیں
- SLAM operation سے پہلے sensors کو صحیح طریقے سے calibrate کریں

## ہارڈویئر Troubleshooting

### روبوٹ Communication مسائل

#### نیٹ ورک کنیکٹیوٹی
**علامات**: روبوٹ commands کا جواب نہیں دے رہا

**تشخیصی مراحل**:
```bash
ping robot_ip_address
```

**حل**:
- جہاں ممکن ہو wired connections استعمال کریں
- مناسب network QoS configure کریں
- Robot network کو دوسری traffic سے الگ کریں

### Sensor مسائل

#### Camera مسائل
**علامات**: کوئی image data نہیں یا خراب image quality

**حل**:
- اگر ضروری ہو camera drivers دوبارہ انسٹال کریں
- USB bandwidth limitations چیک کریں
- اگر mounting بدلی ہو تو camera recalibrate کریں

## عام طلباء کے مسائل

### ابتدائی سیٹ اپ مسائل
**عام مسائل**:
- ROS 2 installation failures
- Workspace creation مسائل
- Environment variable configuration

**فوری حل**:
- Pre-configured VM یا container images فراہم کریں
- تفصیلی installation scripts بنائیں
- سیٹ اپ verification کے لیے منظم checklist استعمال کریں

## Recovery طریقہ کار

### سسٹم Restore Points
- بڑی تبدیلیوں سے پہلے system images برقرار رکھیں
- موجودہ working configurations document کریں
- تیز system recovery کے لیے scripts بنائیں

### Data Backup اور Recovery
- اہم project data کا باقاعدہ backup
- تمام code اور configurations کے لیے version control
- Critical system images کے لیے cloud storage

## سپورٹ وسائل

### Documentation
- Official ROS 2 documentation
- Hardware manufacturer documentation
- مخصوص equipment کے لیے troubleshooting guides

### کمیونٹی وسائل
- ROS Discourse forums
- Packages کے لیے GitHub issue trackers
- Robotics Stack Exchange

## نیویگیشن

[← پچھلا: تدریسی نوٹس](./pedagogical-notes.md) | [انسٹرکٹر گائیڈ ہوم →](./intro.md)
