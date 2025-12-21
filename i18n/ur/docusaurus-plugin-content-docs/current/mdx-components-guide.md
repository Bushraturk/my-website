---
title: ترجمہ عمل گائیڈ (منسوخ شدہ)
sidebar_position: 100
---

# ترجمہ عمل گائیڈ (منسوخ شدہ)

یہ گائیڈ پہلے بتاتی تھی کہ ترجمہ کی فعالیت کے لیے custom MDX components کیسے نافذ کیے جائیں۔ تاہم، پروجیکٹ اب custom approach کی بجائے Docusaurus کے سرکاری i18n سسٹم کا استعمال کرتا ہے۔

## موجودہ Approach

سائٹ اب Docusaurus کے built-in internationalization (i18n) سسٹم کا استعمال کرتی ہے:
- انگریزی content `docs/` directory میں ہے
- اردو ترجمے `i18n/ur/` directory میں ہیں
- سسٹم ہر زبان کے لیے سائٹ کے الگ، static versions بناتا ہے

## Migration کی تفصیلات

- تمام documentation فائلز سے custom DOMTranslate components ہٹا دیے گئے
- Codebase سے custom translation functionality ہٹا دی گئی
- `docusaurus.config.ts` میں سرکاری Docusaurus i18n configure کی گئی
- Locale-specific content کے لیے مناسب directory structure بنائی گئی

نیا ترجمہ شدہ content شامل کرنے کے لیے، براہ کرم documentation میں Translation Process Guide دیکھیں۔
