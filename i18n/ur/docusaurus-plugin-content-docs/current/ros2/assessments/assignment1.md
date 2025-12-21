---
title: اسائنمنٹ 1 - Node Implementation اور Parameters
sidebar_position: 2
---

# اسائنمنٹ 1: Node Implementation اور Parameters

## ہدایات

اس اسائنمنٹ میں آپ کو nodes، parameters، اور communication patterns کے ساتھ ایک مکمل ROS 2 سسٹم لاگو کرنا ہے۔ آپ ایک robot configuration node بنائیں گے جو parameters کا انتظام کرے اور دوسرے nodes کے ساتھ interact کرے۔

## سیکھنے کے مقاصد

اس اسائنمنٹ کے بعد آپ:
- Parameter management کے ساتھ ROS 2 nodes لاگو کر سکیں گے
- متعدد nodes کو coordinate کرنے کے لیے launch files استعمال کر سکیں گے
- مناسب Quality of Service (QoS) policies لگا سکیں گے
- متعدد communication patterns کے ساتھ ایک مربوط سسٹم بنا سکیں گے

## اسائنمنٹ کی ضروریات

### حصہ 1: Robot Configuration Node (40 نمبر)

`robot_config` نامی ROS 2 node بنائیں جو:

1. درج ذیل parameters declare کرے:
   - `robot_name` (string، default: "default_robot")
   - `max_velocity` (double، default: 1.0)
   - `safety_distance` (double، default: 0.5)
   - `operating_mode` (string، default: "autonomous")
   - `sensor_enabled` (boolean، default: True)

2. ایک publisher بنائے جو وقتاً فوقتاً robot configuration معلومات `robot_config_status` topic پر publish کرے۔

3. ایک service server لاگو کرے جو بیرونی nodes کو configuration parameters اپڈیٹ کرنے کی اجازت دے۔

4. Parameter تبدیلیوں کی نگرانی کرے اور اہم تبدیلیوں کو console پر log کرے۔

### حصہ 2: Robot Controller Node (30 نمبر)

`robot_controller` نامی دوسرا node بنائیں جو:

1. اپنے رویے کو configure کرنے کے لیے parameters استعمال کرے

2. Configuration updates وصول کرنے کے لیے `robot_config_status` topic کو subscribe کرے

3. مخصوص حالات میں configuration parameters اپڈیٹ کرنے کے لیے service client لاگو کرے

4. Reliable communication کے لیے مناسب QoS policies استعمال کرے

### حصہ 3: Launch File (20 نمبر)

ایک launch file بنائیں جو دونوں nodes کو مخصوص parameter configurations کے ساتھ شروع کرے۔

### حصہ 4: دستاویزات (10 نمبر)

واضح دستاویزات فراہم کریں:
- اپنے nodes کیسے build اور run کریں
- آپ کے nodes کون سے parameters قبول کرتے ہیں
- اپنے سسٹم کے ساتھ کیسے interact کریں

## Implementation ہدایات

1. Implementation کے لیے Python استعمال کریں
2. ROS 2 best practices کی پیروی کریں
3. Parameter declarations اور services کے لیے error handling شامل کریں
4. اپنے nodes میں مناسب logging استعمال کریں
5. معیاری ROS 2 پیکج ساخت کی پیروی کریں

## مثال ساخت

```
robot_assignment/
├── robot_assignment/
│   ├── __init__.py
│   ├── robot_config.py
│   ├── robot_controller.py
│   └── config_message.py
├── launch/
│   └── robot_system_launch.py
├── CMakeLists.txt
├── package.xml
└── setup.py
```

## جائزہ کا معیار

### حصہ 1: Robot Configuration Node
- [ ] Parameters درست طریقے سے declare اور manage ہیں (10 نمبر)
- [ ] Publisher درست طریقے سے configuration status publish کرتا ہے (10 نمبر)
- [ ] Service server parameter updates لاگو کرتا ہے (10 نمبر)
- [ ] Parameter تبدیلیاں مناسب طریقے سے log ہوتی ہیں (10 نمبر)

### حصہ 2: Robot Controller Node
- [ ] Node parameters درست استعمال کرتا ہے (10 نمبر)
- [ ] Config status کی subscription کام کرتی ہے (10 نمبر)
- [ ] Updates کے لیے Service client کام کرتا ہے (10 نمبر)

### حصہ 3: Launch File
- [ ] Launch file دونوں nodes درست شروع کرتی ہے (10 نمبر)
- [ ] Parameters مناسب طریقے سے configure ہیں (10 نمبر)

### حصہ 4: دستاویزات
- [ ] واضح build اور run ہدایات (5 نمبر)
- [ ] Parameter documentation (5 نمبر)

## جمع کرانے کی ضروریات

1. تمام source code files جمع کروائیں
2. Build اور run ہدایات کے ساتھ README.md شامل کریں
3. اپنی QoS policy choices کی مختصر وضاحت فراہم کریں
4. Implementation کے دوران کیے گئے کسی بھی assumptions کو document کریں

## معیار

- کل نمبر: 100
- پاس ہونے کا سکور: 70/100 (70%)
- دیر سے جمع کرانے کا جرمانہ: فی دن 5%
