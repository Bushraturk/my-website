---
title: Translation Troubleshooting Guide (Deprecated)
sidebar_position: 101
---

# Translation Troubleshooting Guide (Deprecated)

This guide previously provided solutions to common issues with the custom MDX component translation approach. However, the project now uses Docusaurus's official i18n system.

## Current Troubleshooting

With the official i18n system, troubleshooting is different:

### 1. Translation Files Not Appearing

**Problem**: Translated content is not showing in the selected language.

**Solutions**:
- Verify that translation files exist in the `i18n/[locale]/` directory
- Check that file names match between English and translated content
- Make sure both files have matching slugs in their frontmatter

### 2. Language Switcher Not Appearing

**Problem**: The language selector is not showing in the navbar.

**Solutions**:
- Confirm multiple locales are defined in `docusaurus.config.ts`
- Verify that the site has been rebuilt after configuration changes

### 3. Mixed Language Content

**Problem**: Some content appears in English while other parts are in Urdu.

**Solutions**:
- Check that all required content exists in the appropriate locale directories
- Ensure all documentation files have been translated

## Debugging Steps

### For i18n Issues
1. Check that locale configuration is correct in `docusaurus.config.ts`
2. Verify the directory structure matches Docusaurus i18n conventions
3. Ensure both English and translated content exist for each page

## Verification Checklist

Before deploying with the new i18n system:

- [ ] Multiple locales are defined in `docusaurus.config.ts`
- [ ] Translation files exist in the `i18n/[locale]/` directories
- [ ] File names match between English and translated content
- [ ] Sidebar configurations include both languages properly
- [ ] Language switcher appears in the navbar
- [ ] Build runs without errors
- [ ] Content displays correctly in both languages