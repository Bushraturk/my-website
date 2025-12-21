---
title: Content Download اور آف لائن رسائی
sidebar_position: 3
---

# Content Download اور آف لائن رسائی

## جائزہ

فزیکل AI اور ہیومنوائڈ روبوٹکس ٹیکسٹ بک download capabilities فراہم کرتی ہے جو طلباء کو آف لائن content تک رسائی کی اجازت دیتی ہیں۔ یہ خصوصیت خاص طور پر محدود انٹرنیٹ رسائی والے طلباء یا ان لوگوں کے لیے مفید ہے جو connectivity کے بغیر مطالعہ کرنا پسند کرتے ہیں۔

## یہ کیسے کام کرتا ہے

طلباء کسی بھی module یا ہفتہ وار content صفحے پر "Download for Offline Use" بٹن استعمال کر کے اس content کو اپنے ڈیوائس پر download کر سکتے ہیں۔ Content PDF format میں فراہم کیا جاتا ہے اور اس میں تمام text content اور diagrams شامل ہیں۔

## طلباء کے لیے

1. جو content آپ download کرنا چاہتے ہیں اس پر navigate کریں
2. "Download for Offline Use" بٹن پر کلک کریں
3. فائل کو اپنے ڈیوائس میں محفوظ کریں
4. انٹرنیٹ کنکشن کے بغیر content تک رسائی حاصل کریں

## اساتذہ کے لیے

اساتذہ پورے modules کے لیے downloads تیار کر سکتے ہیں تاکہ محدود انٹرنیٹ رسائی والے طلباء کو تقسیم کیا جا سکے۔ پہلے سے content کے PDF versions تیار کرنے کے لیے content generation scripts استعمال کریں۔

### Downloads تیار کرنا

مخصوص content کے لیے downloads تیار کرنے کے لیے:

```bash
npm run download:content [module] [week?option]
```

مثالیں:
- `npm run download:content ros2 week1-2` - ROS 2 module کے Week 1-2 کے لیے download تیار کریں
- `npm run download:content ros2` - پورے ROS 2 module کے لیے download تیار کریں
- `npm run download:content ros2 manifest` - دستیاب downloads کی فہرست حاصل کریں

## تکنیکی Implementation

Download functionality درج ذیل کا استعمال کرتے ہوئے نافذ ہے:

1. ایک Node.js script (`scripts/download-content.js`) جو downloadable content تیار کر سکتی ہے
2. ایک React component (`DownloadButton.tsx`) جو downloading کے لیے user interface فراہم کرتا ہے
3. ایک static file serving mechanism جو downloads کو `/static/downloads/` پر دستیاب کرتا ہے

## بہترین طریقے

- طلباء کو حوصلہ افزائی کریں کہ جب ان کے پاس اچھی connectivity ہو تو پہلے سے content download کر لیں
- Storage requirements کے بارے میں رہنمائی فراہم کریں (ہر module 50-100MB ہو سکتا ہے)
- طلباء کو downloaded content کے لیے update policy کے بارے میں مطلع کریں
- Content updates schedule کرتے وقت time zones اور رسائی کے patterns پر غور کریں

## حدود

- Downloaded content میں interactive elements کام نہیں کر سکتے
- PDF میں کچھ diagrams اور code examples کی formatting محدود ہو سکتی ہے
- Updated content کے لیے طلباء کو نئے versions download کرنے ہوں گے
- سست connections پر بڑے downloads میں وقت لگ سکتا ہے

## Troubleshooting

**سوال: Download بٹن grey ہے**
جواب: ابھی تمام content download کے لیے دستیاب نہیں ہے۔ بعد میں چیک کریں یا اپنے instructor سے رابطہ کریں۔

**سوال: Downloaded فائل نہیں کھل رہی**
جواب: PDF viewer کے ساتھ کھولنے کی کوشش کریں یا support سے رابطہ کریں۔

**سوال: کیا میں پوری ٹیکسٹ بک download کر سکتا ہوں؟**
جواب: فی الحال، downloads module یا week کے لحاظ سے دستیاب ہیں۔ مکمل ٹیکسٹ بک downloads مستقبل کی releases میں دستیاب ہو سکتے ہیں۔
