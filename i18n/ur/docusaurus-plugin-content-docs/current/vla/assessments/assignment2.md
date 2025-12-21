---
title: اسائنمنٹ 2 - ایڈوانسڈ VLA سسٹم
sidebar_position: 23
---

# اسائنمنٹ 2: ایڈوانسڈ ویژن-لینگویج-ایکشن سسٹم

## مقصد

ایک ایڈوانسڈ Vision-Language-Action (VLA) سسٹم ڈیزائن اور لاگو کریں جو پہلی اسائنمنٹ کی بنیادی فعالیت کو بڑھائے۔ یہ اسائنمنٹ پیچیدہ scene understanding، multi-step task execution، اور dynamic ماحول میں مضبوط error handling پر توجہ دیتی ہے۔

## سیکھنے کے نتائج

اس اسائنمنٹ کے بعد آپ:
- پیچیدہ scenes کو سنبھالنے والے ایڈوانسڈ perception سسٹم لاگو کر سکیں گے
- conditional execution کے ساتھ multi-step action planning ڈیزائن کر سکیں گے
- ماحولیاتی تبدیلیوں اور ناکامیوں کو سنبھالنے والے مضبوط سسٹم بنا سکیں گے
- مختلف مشکل حالات میں سسٹم کی کارکردگی کا جائزہ لے سکیں گے

## اسائنمنٹ کی ضروریات

### ایڈوانسڈ Perception (25 نمبر)
1. متعدد اشیاء اور ان کے spatial تعلقات کی شناخت
2. اشیاء کے ساتھ ممکنہ actions کی affordance prediction
3. Dynamic scene تبدیلی کا پتہ لگانا اور جواب دینا
4. متعدد sensor modalities کا انضمام

### Multi-Step Task Execution (30 نمبر)
1. درمیانی اہداف کے ساتھ multi-step tasks کی planning
2. Perception feedback پر مبنی conditional execution
3. ناکام action execution کے لیے error recovery behaviors
4. Task interruption اور resumption کی صلاحیتیں

### مضبوطی اور موافقت (20 نمبر)
1. Task execution کے دوران ماحولیاتی تبدیلیوں کا سامنا
2. نئی اشیاء یا حالات کے مطابق ڈھلنا
3. Perception systems کی ناکامی پر graceful degradation
4. مختلف failure modes سے recovery

### جائزہ اور دستاویزات (25 نمبر)
1. متعدد scenarios میں جامع testing
2. کامیابی کی شرح، execution time، اور مضبوطی کے metrics
3. سسٹم ڈیزائن اور implementation کی تفصیلی دستاویزات
4. سسٹم کی حدود اور مستقبل کی بہتری کا تجزیہ

## تکنیکی تفصیلات

### سسٹم آرکیٹیکچر
- اجزاء کے درمیان واضح interfaces کے ساتھ modular ڈیزائن
- Task execution کے لیے مناسب frame rates کے ساتھ real-time performance
- نقصان دہ روبوٹ رویے کو روکنے کے لیے safety measures
- سسٹم تجزیہ کے لیے logging اور debugging صلاحیتیں

### کارکردگی کی ضروریات
- متعین tasks کے لیے 80% سے زیادہ کامیابی کی شرح
- Perception-action cycles کے لیے 2 سیکنڈ سے کم response time
- 30 سیکنڈ کے اندر عام failure modes سے recovery
- Testing کے دوران 95% سے زیادہ سسٹم availability

## جمع کرانے کی چیزیں

### 1. Implementation (50 نمبر)
- ایڈوانسڈ VLA سسٹم کا مکمل source code
- Configuration files اور setup ہدایات
- اہم اجزاء کے لیے unit tests

### 2. جائزہ رپورٹ (30 نمبر)
- تفصیلی experimental setup
- شماریاتی تجزیہ کے ساتھ مقداری نتائج
- سسٹم صلاحیتوں کا معیاری جائزہ

### 3. دستاویزات (20 نمبر)
- سسٹم آرکیٹیکچر اور ڈیزائن فیصلے
- آپریشن کے لیے user manual
- عام مسائل کے لیے troubleshooting گائیڈ

### 4. پریزنٹیشن (15 نمبر)
- سسٹم صلاحیتوں کی 15 منٹ کی پریزنٹیشن
- سسٹم آپریشن کا live demonstration یا ویڈیو
- تکنیکی چیلنجز اور حل کی بحث

## درجہ بندی کا معیار

| جزو | نمبر | معیار |
|-----|------|--------|
| ایڈوانسڈ Perception | 25 | پیچیدہ scenes کی جدید سمجھ |
| Multi-Step Execution | 30 | پیچیدہ tasks کی کامیاب planning اور execution |
| مضبوطی | 20 | سسٹم recovery اور موافقت کی صلاحیتیں |
| جائزہ رپورٹ | 30 | مکمل تجزیہ اور نتائج کی پیشکش |
| دستاویزات | 20 | دستاویزات کا معیار اور مکمل پن |
| پریزنٹیشن | 15 | سسٹم صلاحیتوں کی واضح پیشکش |
| **کل** | **140** | |

## اضافی سرگرمیاں (اختیاری - اضافی نمبروں کے لیے)

غیر معمولی implementation کے لیے:
1. نئے tasks کے لیے demonstration سے سیکھنا
2. نئی اشیاء/ماحول پر zero-shot generalization
3. انسانی آپریٹرز کے ساتھ collaborative behaviors
4. بہتر صلاحیتوں کے لیے cloud services کا انضمام

15% تک اضافی نمبر دستیاب ہیں۔

## جمع کرانے کی تفصیلات

کورس مینجمنٹ سسٹم کے ذریعے جمع کروائیں:
- مکمل implementation کے ساتھ Git repository
- PDF میں جائزہ رپورٹ
- سسٹم صلاحیتوں کی ویڈیو demonstration
