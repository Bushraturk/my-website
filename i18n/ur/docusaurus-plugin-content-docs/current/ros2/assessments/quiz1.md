---
title: کوئز 1 - ROS 2 آرکیٹیکچر کی بنیادیات
sidebar_position: 1
---

# کوئز 1: ROS 2 آرکیٹیکچر کی بنیادیات

## ہدایات

یہ کوئز ہفتہ 1-2 میں شامل ROS 2 آرکیٹیکچر کی بنیادیات کی آپ کی سمجھ کا جائزہ لیتا ہے۔ ہر سوال کا بہترین جواب منتخب کریں۔ آپ اس open-book کوئز کے دوران کورس کے مواد سے رجوع کر سکتے ہیں۔

## سوالات

### سوال 1: ROS 2 کے سیاق میں DDS کا کیا مطلب ہے؟

A) Data Definition System
B) Data Distribution Service
C) Dynamic Discovery System
D) Distributed Data Service

### سوال 2: درج ذیل میں سے کون سا ROS 2 میں بنیادی communication primitive نہیں ہے؟

A) Topics
B) Services
C) Parameters
D) Databases

### سوال 3: ROS 2 node کا بنیادی مقصد کیا ہے؟

A) ڈیٹا کو مستقل طور پر ذخیرہ کرنا
B) گرافیکل یوزر انٹرفیس فراہم کرنا
C) بنیادی execution اکائی جو computation کرتی ہے
D) نیٹ ورک کنیکشنز کا انتظام کرنا

### سوال 4: ROS 2 میں، publisher-subscriber تعلق کیا ہے؟

A) ایک سسٹم جہاں nodes server سے ڈیٹا کی درخواست کرتے ہیں
B) ایک one-to-many communication pattern جہاں ایک node بہت سے listeners کو messages بھیجتا ہے
C) Configuration ڈیٹا ذخیرہ کرنے کا طریقہ
D) Nodes کی تصدیق کے لیے ایک security mechanism

### سوال 5: تمام active ROS 2 nodes کی فہرست دیکھنے کے لیے کون سا command استعمال ہوتا ہے؟

A) `ros2 list nodes`
B) `ros2 show nodes`
C) `ros2 node list`
D) `ros2 get nodes`

### سوال 6: ROS 2 آرکیٹیکچر میں "DDS" کیا فراہم کرتا ہے؟

A) روبوٹ ڈیٹا کے لیے ڈیٹابیس سسٹم
B) ایک middleware جو nodes کے درمیان message passing کو سنبھالتا ہے
C) Debugging کے لیے visualization tool
D) ایک Integrated Development Environment (IDE)

### سوال 7: ROS 2 میں Quality of Service (QoS) policy کیا ہے؟

A) روبوٹ ڈیٹا کی حفاظت کے لیے security protocol
B) قواعد کا ایک سیٹ جو reliability، durability وغیرہ کے لحاظ سے messages کی delivery کی وضاحت کرتا ہے
C) Performance measurement tool
D) ROS 2 packages کے لیے coding standard

### سوال 8: Publisher-subscriber pattern میں، publisher کا کردار کیا ہے؟

A) دوسرے nodes سے ڈیٹا کی درخواست کرنا
B) ایک topic پر messages بھیجنا جو subscribers وصول کر سکیں
C) متعدد nodes کو coordinate کرنا
D) ڈیٹابیس میں ڈیٹا ذخیرہ کرنا

### سوال 9: ROS 2 client library (RCL) اور DDS کے درمیان تعلق کیا ہے؟

A) RCL، DDS کا متبادل ہے
B) RCL اور DDS ایک جیسے ہیں
C) RCL، ROS 2 اور DDS middleware کے درمیان abstraction layer فراہم کرتا ہے
D) DDS، ROS 2 اور RCL کے درمیان abstraction layer فراہم کرتا ہے

### سوال 10: ROS 2 پیکج کیا ہے؟

A) روبوٹ سافٹ ویئر پر مشتمل compressed فائل
B) مل کر کام کرنے والے nodes کا مجموعہ
C) بنیادی building block جس میں software nodes، libraries، اور دیگر وسائل شامل ہیں
D) ROS 2 کے انتظام کے لیے command-line tool

---

## جوابات

| سوال | صحیح جواب |
|------|-----------|
| 1 | B |
| 2 | D |
| 3 | C |
| 4 | B |
| 5 | C |
| 6 | B |
| 7 | B |
| 8 | B |
| 9 | C |
| 10 | C |

## معیار

- سوالات 1-10: ہر ایک 1 نمبر
- کل: 10 نمبر
- پاس ہونے کا سکور: 7/10 (70%)

## اضافی وسائل

مزید معلومات کے لیے، جائزہ لیں:
- ROS 2 آرکیٹیکچر پر ہفتہ 1-2 کا مواد
- DDS اور QoS پر ROS 2 دستاویزات
- عملی implementation کے لیے لیب 1
