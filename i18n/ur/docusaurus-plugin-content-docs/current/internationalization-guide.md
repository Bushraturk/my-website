---
title: بین الاقوامیت کی رہنما
---

# بین الاقوامیت کی رہنما

## بین الاقوامیت کیوں اہم ہے؟

بین الاقوامیت (i18n) یقینی بناتی ہے کہ آپ کا کوائف عالمی سطح پر دستیاب ہے اور مختلف زبانوں اور ثقافتوں کے صارفین کے لیے مناسب ہے۔

## Docusaurus i18n کا استعمال

Docusaurus ایک طاقتور i18n نظام فراہم کرتا ہے:

### 1. کنفیگریشن میں ترتیب

`docusaurus.config.js` میں:

```js
module.exports = {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },
};
```

### 2. دستاویزات کی ترتیب

- انگریزی دستاویزات: `docs/`
- اردو ترجمہ: `i18n/ur/docusaurus-plugin-content-docs/current/`

### 3. مثال: دستاویز کا ترجمہ

اصل فائل (`docs/example.md`):

```md
---
title: Example Page
sidebar_position: 1
---

# Example Title

This is example content in English.
```

ترجمہ فائل (`i18n/ur/docusaurus-plugin-content-docs/current/example.md`):

```md
---
title: مثال کا صفحہ
sidebar_position: 1
---

# مثال کا عنوان

یہ انگریزی میں مثال کا مواد ہے۔
```

## RTL (Right-to-Left) ترتیب

### 1. CSS تبدیلیاں

RTL لی آؤٹ کے لیے:

```css
/* src/css/custom.css */
html[dir="rtl"] .navbar {
  text-align: right;
}

html[dir="rtl"] .menu {
  text-align: right;
}

html[dir="rtl"] .pagination-nav {
  text-align: center;
}

/* RTL for specific components */
html[dir="rtl"] .hero__title {
  text-align: right;
}

html[dir="rtl"] .hero__subtitle {
  text-align: right;
}
```

### 2. فونٹ کی ترتیب

اردو کے لیے مناسب فونٹ استعمال کریں:

```css
/* For Urdu text */
html[dir="ur"] body {
  font-family: 'Jameel Noori Nastaleeq', 'Urdu Typesetting', sans-serif;
}
```

## ترجمہ کے لیے بہترین طریقے

### 1. تسلسل کو برقرار رکھنا

- وہی IDs اور permalinks استعمال کریں
- سائڈ بار کی ترتیب ایک جیسی رکھیں
- لنکس کو درست رکھیں

### 2. میٹا ڈیٹا کو اپ ڈیٹ کرنا

ہر ترجمہ شدہ صفحہ میں:

- عنوان کا ترجمہ
- میٹا تفصیل کا ترجمہ
- ہر کی ورڈز کو اردو میں

### 3. تصاویر کے لیے Alt ٹیکسٹ

اردو میں تصاویر کے لیے alt ٹیکسٹ کا ترجمہ کریں:

```md
![اردو میں تفصیل](/path/to/image.png)
```

## ترجمہ کی جانچ

### 1. ہر ترجمہ کی جانچ

- کیا تمام لنک کام کر رہے ہیں؟
- کیا تصاویر ٹھیک سے لوڈ ہو رہی ہیں؟
- کیا CSS مناسب طریقے سے کام کر رہا ہے؟
- کیا تمام کمپوننٹس کام کر رہے ہیں؟

### 2. اردو کی جانچ

- کیا متن درست RTL ہے؟
- کیا فونٹ درست طریقے سے ڈسپلے ہو رہا ہے؟
- کیا کوائف کی ساخت درست ہے؟
- کیا کارکردگی متاثر نہیں ہوئی؟

## ٹیسٹنگ کے لیے چیک لسٹ

- [ ] ترجمہ شدہ صفحات 404 نہیں دے رہے
- [ ] متن درست RTL ہے
- [ ] تصاویر کے alt ٹیکسٹ ترجمہ شدہ ہیں
- [ ] تفصیل اور کی ورڈز ترجمہ شدہ ہیں
- [ ] کمپوننٹس کام کر رہے ہیں
- [ ] نیوی گیشن کام کر رہی ہے
- [ ] تلاش کی سہولت کام کر رہی ہے
- [ ] کارکردگی متاثر نہیں ہوئی

## سمتھ بار کی ترتیب

سائڈ بار کو دونوں زبانوں میں ایک جیسا رکھیں:

```js
// sidebars.js
module.exports = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',  // اس لیبل کا ترجمہ اردو میں کیا جائے گا
      items: [
        'intro',  // یہ ID دونوں زبانوں میں ایک جیسا ہے
        'home-test',
        // ... دیگر اشیاء
      ],
    },
  ],
};
```

## استعمال کی مثال

### اردو صفحہ

```md
---
title: جسمانی AI اور ہیومنوائڈ روبوٹکس کا تعارف
sidebar_position: 1
---

# جسمانی AI اور ہیومنوائڈ روبوٹکس کا تعارف

یہ جامع کوائف فزکل AI اور ہیومنوائڈ روبوٹکس کے موضوعات کو کور کرتا ہے...

[← پچھلا](./prev-page.md) | [اگلا](./next-page.md)
```

## ٹربل شوٹنگ

### متداول مسائل

1. **404 Errors**: ترجمہ کی فائلیں موجود نہیں ہیں
2. **RTL Issues**: CSS RTL ترتیب نہیں ہے
3. **Font Issues**: مناسب فونٹ نہیں لوڈ ہو رہا
4. **Navigation Issues**: نیوی گیشن لنکس درست نہیں ہیں

### حل

1. تمام ترجمہ کی فائلیں بنائیں
2. RTL CSS شامل کریں
3. مناسب فونٹ شامل کریں
4. نیوی گیشن کو چیک کریں

## خلاصہ

i18n Docusaurus کا ایک اہم حصہ ہے جو عالمی آڈیئن تک رسائی کو یقینی بناتا ہے:

- [ ] مناسب کنفیگریشن
- [ ] مناسب ڈائریکٹری سٹرکچر
- [ ] RTL ترتیب
- [ ] میٹا ڈیٹا کا ترجمہ
- [ ] تصاویر کا ترجمہ

[← پچھلا: دستاویزات کا ڈھانچہ](./documentation-guide.md) | [اگلا: تکرسی گائیڈ](./accessibility-guide.md) | [ہوم](./intro.md)