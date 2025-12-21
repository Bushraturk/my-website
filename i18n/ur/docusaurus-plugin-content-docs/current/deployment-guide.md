---
title: ڈیپلائمنٹ گائیڈ
---

# ڈیپلائمنٹ گائیڈ

## ڈیپلائمنٹ کی تیاری

### 1. ٹیسٹ سیٹ اپ

آپ کی کتاب کو ڈیپلائی کرنے سے پہلے، اس کی تیاری کریں:

- [ ] تمام یونٹ ٹیسٹس فیل نہ ہوں
- [ ] تمام انٹیگریشن ٹیسٹس فیل نہ ہوں
- [ ] پرفارمنس ٹیسٹس مطلوبہ معیارات پر پورا اترتے ہوں
- [ ] تکرسی ٹیسٹس کام کرتے ہوں
- [ ] تمام لنکس ٹوٹل نہ ہوں

### 2. ڈیویلپمنٹ کی ترتیبات

`docusaurus.config.js` میں:

```js
module.exports = {
  // ... دیگر کنفیگریشن
  url: 'https://your-domain.github.io',  // آپ کی مطلوبہ ڈومین
  baseUrl: '/your-repository-name/',     // GitHub Pages کے لیے
  trailingSlash: false,                  // SEO کے لیے اہم
  // ... باقی کنفیگریشن
};
```

## GitHub Pages ڈیپلائمنٹ

### 1. CI/CD سیٹ اپ

`.github/workflows/deploy.yml` فائل:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      # GitHub Pages deployment step
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
          publish_branch: gh-pages
```

### 2. ڈیپلائمنٹ کمینڈ

`package.json` میں:

```json
{
  "scripts": {
    // ... دیگر اسکرپٹس
    "deploy": "docusaurus deploy"
  }
}
```

### 3. GitHub میں سیٹ اپ

1. ریپو میں جائیں
2. Settings > Pages 
3. Source: Deploy from a branch
4. Branch: gh-pages, root

## ڈیپلائمنٹ کے لیے کمینڈز

### 1. ڈیویلپمنٹ کا ایچنمنٹ

```bash
# لوکل کے لیے
npm run start
```

### 2. بلڈ بنانے کے لیے

```bash
# اسٹیٹک بلڈ بنائیں
npm run build
```

### 3. ڈیپلائی کرنے کے لیے

```bash
# GitHub Pages پر ڈیپلائی کریں
npm run deploy
```

## CDN ڈیپلائمنٹ (اختیاری)

### Cloudflare Pages کے لیے

1. Cloudflare میں نیا پروجیکٹ بنائیں
2. GitHub ریپو کو کنیکٹ کریں
3. Build settings:

- Build command: `npm run build`
- Build output directory: `build`
- Environment variables: NODE_VERSION = 18

## SEO اور Meta Tags

### Meta Tags کنفیگریشن

`docusaurus.config.js` میں:

```js
module.exports = {
  // ... باقی کنفیگریشن
  themeConfig: {
    metadata: [
      {name: 'keywords', content: 'robotics, AI, textbook, embodied intelligence'},
      {name: 'description', content: 'Comprehensive textbook on Physical AI & Humanoid Robotics'},
      {property: 'og:image', content: '/img/robotics-social-card.jpg'},
      {name: 'twitter:card', content: 'summary_large_image'},
    ],
    // ... باقی تھیم کنفیگریشن
  },
};
```

## کارکردگی کی بہتری

### 1. کوائف کو تیز کرنا

- [ ] Code splitting کا استعمال کرنا
- [ ] Image optimization کرنا
- [ ] Bundle size کو کم کرنا
- [ ] Lazy loading کا استعمال کرنا

### 2. SEO بہتری

- [ ] Sitemap کی تیاری
- [ ] Meta tags کی درستی
- [ ] Structured data
- [ ] Fast loading times

## ڈیپلائمنٹ کے لیے چیک لسٹ

### قبل ڈیپلائمنٹ کی جانچ

- [ ] ڈیویلپمنٹ سرور کام کر رہا ہے: `npm run start`
- [ ] بلڈ کام کر رہا ہے: `npm run build`
- [ ] کوائف کے تمام لنکس کام کر رہے ہیں
- [ ] ترجمہ کے تمام صفحات دستیاب ہیں
- [ ] تصاویر ٹھیک طریقے سے لوڈ ہو رہی ہیں
- [ ] تمام کمپوننٹس کام کر رہے ہیں

### ڈیپلائمنٹ کے بعد

- [ ] صفحات کام کر رہے ہیں
- [ ] HTTPS کام کر رہا ہے
- [ ] CDN صحیح طریقے سے کام کر رہا ہے
- [ ] SEO tags دکھائی دے رہے ہیں
- [ ] Sitemap دستیاب ہے
- [ ] Google Search Console میں کتاب شامل ہے

## ٹربل شوٹنگ

### 404 Errors

- GitHub Pages کی ترتیب صحیح ہے
- baseUrl صحیح ہے
- Sitemap فائل دستیاب ہے

### ترجمہ کے مسائل

- i18n فائلیں دستیاب ہیں
- Locale کنفیگریشن صحیح ہے
- تمام صفحات دونوں زبانوں میں دستیاب ہیں

### کارکردگی کے مسائل

- Bundle size چھوٹا ہے
- Images optimized ہیں
- Unused code ہٹا دیا گیا ہے

## نیوی گیشن

[← پچھلا: کارکردگی کی بہتری](./performance-optimization.md) | [اگلا: ٹیسٹنگ حکمت عمل](./testing-strategy.md) | [ہوم](./intro.md)