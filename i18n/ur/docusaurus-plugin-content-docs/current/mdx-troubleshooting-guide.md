---
title: ترجمہ Troubleshooting گائیڈ (منسوخ شدہ)
sidebar_position: 101
---

# ترجمہ Troubleshooting گائیڈ (منسوخ شدہ)

یہ گائیڈ پہلے custom MDX component translation approach کے ساتھ عام مسائل کے حل فراہم کرتی تھی۔ تاہم، پروجیکٹ اب Docusaurus کے سرکاری i18n سسٹم کا استعمال کرتا ہے۔

## موجودہ Troubleshooting

سرکاری i18n سسٹم کے ساتھ، troubleshooting مختلف ہے:

### 1. ترجمہ فائلز ظاہر نہیں ہو رہیں

**مسئلہ**: ترجمہ شدہ content منتخب زبان میں نہیں دکھ رہا۔

**حل**:
- تصدیق کریں کہ ترجمہ فائلز `i18n/[locale]/` directory میں موجود ہیں
- چیک کریں کہ فائل کے نام انگریزی اور ترجمہ شدہ content کے درمیان میل کھاتے ہیں
- یقینی بنائیں کہ دونوں فائلوں میں frontmatter میں matching slugs ہیں

### 2. Language Switcher ظاہر نہیں ہو رہا

**مسئلہ**: Navbar میں language selector نہیں دکھ رہا۔

**حل**:
- تصدیق کریں کہ `docusaurus.config.ts` میں متعدد locales defined ہیں
- تصدیق کریں کہ configuration تبدیلیوں کے بعد سائٹ دوبارہ build ہوئی ہے

### 3. ملا جلا Language Content

**مسئلہ**: کچھ content انگریزی میں ظاہر ہو رہا ہے جبکہ دوسرے حصے اردو میں ہیں۔

**حل**:
- چیک کریں کہ تمام مطلوبہ content مناسب locale directories میں موجود ہے
- یقینی بنائیں کہ تمام documentation فائلز کا ترجمہ ہو گیا ہے

## Debugging Steps

### i18n مسائل کے لیے
1. چیک کریں کہ `docusaurus.config.ts` میں locale configuration درست ہے
2. تصدیق کریں کہ directory structure Docusaurus i18n conventions سے میل کھاتی ہے
3. یقینی بنائیں کہ ہر صفحے کے لیے انگریزی اور ترجمہ شدہ دونوں content موجود ہیں

## تصدیق کی چیک لسٹ

نئے i18n سسٹم کے ساتھ deploy کرنے سے پہلے:

- [ ] `docusaurus.config.ts` میں متعدد locales defined ہیں
- [ ] `i18n/[locale]/` directories میں ترجمہ فائلز موجود ہیں
- [ ] انگریزی اور ترجمہ شدہ content کے درمیان فائل کے نام میل کھاتے ہیں
- [ ] Sidebar configurations میں دونوں زبانیں صحیح طریقے سے شامل ہیں
- [ ] Navbar میں language switcher ظاہر ہو رہا ہے
- [ ] Build بغیر errors کے چلتی ہے
- [ ] دونوں زبانوں میں content صحیح طریقے سے display ہو رہا ہے
