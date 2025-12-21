---
title: کارکردگی کی اصلاح کی گائیڈ
description: فزیکل AI اور ہیومنوائڈ روبوٹکس ٹیکسٹ بک کی کارکردگی کو بہتر بنانے کی تکنیکیں اور بہترین طریقے
sidebar_position: 25
---

# کارکردگی کی اصلاح کی گائیڈ

یہ گائیڈ فزیکل AI اور ہیومنوائڈ روبوٹکس ٹیکسٹ بک میں نافذ کردہ کارکردگی کی اصلاح کی حکمت عملیوں کا خاکہ پیش کرتی ہے تاکہ طلباء اور اساتذہ کے لیے تیز لوڈنگ اوقات اور جوابدہ تعاملات کو یقینی بنایا جا سکے۔

## کارکردگی کے مقاصد

ہمارے کارکردگی کے اہداف میں شامل ہیں:
- **صفحہ لوڈ وقت**: تمام ٹیکسٹ بک صفحات کے لیے 3 سیکنڈ سے کم
- **انٹرایکٹو جواب**: صارف کے تعاملات پر 100ms سے کم جواب
- **وسائل کی کارکردگی**: کم سے کم بینڈوڈتھ اور پروسیسنگ ضروریات
- **ڈیوائس مطابقت**: مختلف ڈیوائس صلاحیتوں کے لیے بہتر بنایا گیا

## نافذ کردہ اصلاحات

### 1. Code Splitting اور Lazy Loading

ہم نے code splitting نافذ کی ہے تاکہ ہر صفحے کے لیے صرف مطلوبہ components لوڈ ہوں:

```jsx
// Lazy loading implementation کی مثال
import React, { Suspense, lazy } from 'react';

const ModuleCard = lazy(() => import('./ModuleCard'));
const LabExercise = lazy(() => import('./LabExercise'));

const OptimizedPage = () => (
  <Suspense fallback={<div>Loading...</div>}>
    <ModuleCard />
    <LabExercise />
  </Suspense>
);
```

یہ تکنیک ابتدائی bundle size کو کم کرتی ہے اور time-to-interactive کو بہتر بناتی ہے۔

### 2. Image Optimization

تمام تصاویر Docusaurus کی built-in image optimization کا استعمال کرتے ہوئے بہتر بنائی گئی ہیں:

- **Format**: جہاں supported ہو WebP، JPEG/PNG کے fallbacks کے ساتھ
- **Size**: متعدد resolutions کے ساتھ responsive تصاویر
- **Loading**: off-screen تصاویر کے لیے lazy loading

### 3. Component Memoization

کثرت سے render ہونے والے components React.memo استعمال کرتے ہیں تاکہ غیر ضروری re-renders کو روکا جا سکے:

```jsx
const MemoizedComponent = React.memo(({ data }) => {
  // Component implementation
});
```

### 4. Virtual Scrolling

بڑی content فہرستوں والے صفحات کے لیے، ہم virtual scrolling نافذ کرتے ہیں تاکہ صرف visible items render ہوں:

```jsx
// Exercises کے لیے virtual scrolling کی مثال
const VirtualExerciseList = ({ exercises }) => {
  // صرف visible exercises دکھانے کا implementation
};
```

### 5. Caching Strategies

- **Browser Caching**: static assets کے لیے طویل مدتی caching
- **Service Worker**: آف لائن صلاحیتیں اور asset caching
- **API Response Caching**: dynamic content کے لیے cached responses

## کارکردگی ٹیسٹنگ کے نتائج

ہماری موجودہ کارکردگی میٹرکس (Lighthouse سے ماپی گئی):

- **Performance Score**: تمام صفحات پر 90+
- **Largest Contentful Paint**: 2.5s سے کم
- **First Input Delay**: 100ms سے کم
- **Cumulative Layout Shift**: 0.1 سے کم

## Content Authors کے لیے بہترین طریقے

### 1. تصاویر کو Optimize کریں
- ٹیکسٹ بک میں شامل کرنے سے پہلے تصاویر کو compress کریں
- مناسب formats استعمال کریں (photos کے لیے WebP، graphics کے لیے SVG)
- Layout shifts کو روکنے کے لیے dimensions specify کریں

### 2. بھاری Components کو کم کریں
- پیچیدہ interactive elements کے لیے lazy loading استعمال کریں
- بڑے inline code blocks سے بچیں
- advanced features کے لیے progressive enhancement پر غور کریں

### 3. موثر Code Examples
- Code examples کو مختصر اور مرکوز رکھیں
- پڑھنے کی آسانی کے لیے syntax highlighting استعمال کریں
- computationally intensive مثالوں کے لیے performance notes شامل کریں

## کارکردگی کی نگرانی

ہم مسلسل کارکردگی کی نگرانی کرتے ہیں:

- CI/CD pipeline میں خودکار performance testing
- اصل صارف کے تجربات کے لیے Real user monitoring (RUM)
- Web vitals metrics کا استعمال کرتے ہوئے باقاعدہ performance audits

## Troubleshooting

### سست صفحہ لوڈز
1. بڑی تصاویر یا media فائلز چیک کریں
2. Code splitting صحیح کام کر رہی ہے تصدیق کریں
3. Third-party script loading کا جائزہ لیں

### تعامل میں تاخیر
1. غیر ضروری re-renders کے لیے React components کا profile بنائیں
2. Render methods میں expensive operations چیک کریں
3. مناسب state management patterns نافذ کریں

### Memory مسائل
1. Component memory leaks چیک کریں
2. Event listeners کی proper cleanup کی تصدیق کریں
3. State میں ضرورت سے زیادہ data storage کا profile بنائیں

## نتیجہ

یہ کارکردگی کی اصلاحات یقینی بناتی ہیں کہ فزیکل AI اور ہیومنوائڈ روبوٹکس ٹیکسٹ بک مختلف ڈیوائسز اور network conditions پر موثر طریقے سے content فراہم کرے۔ ان تکنیکوں کو نافذ کرنے اور بہترین طریقوں کی پیروی کرنے سے، ہم تمام صارفین کے لیے تیز loading times اور responsive تعاملات برقرار رکھتے ہیں۔
