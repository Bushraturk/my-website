# Adding Urdu Translation to New Content (Deprecated)

## Overview
This document previously explained how to add Urdu translation support to new content using the custom DOMTranslate component approach. However, the project has transitioned to using Docusaurus's official i18n system instead.

## Current Approach

The site now uses Docusaurus's built-in internationalization (i18n) system:

- English content is in the `docs/` directory
- Urdu translations are in the `i18n/ur/docusaurus-plugin-content-docs/default/` directory
- The system creates separate, static versions of the site for each language

## Process for Adding New Translated Content

### 1. Create English Content
First, create your English content in the `docs/` directory as you normally would.

### 2. Create Urdu Translation
Create the corresponding Urdu translation in `i18n/ur/docusaurus-plugin-content-docs/default/` with the same filename.

### 3. Maintain Consistent Structure
- Keep the same frontmatter structure (title, sidebar_position, etc.)
- Ensure both files have matching slugs in their frontmatter
- Maintain the same sidebar position for consistent navigation

## Best Practices

- Keep translated content synchronized with the English version
- Maintain the same sidebar_position values in translated files
- Use consistent terminology across translations
- Test translation functionality after each update

## What to Translate

- All textual content that should appear in Urdu
- Headers, paragraphs, lists, and other textual elements
- Navigation elements and UI text that should be localized

## What Not to Translate

- Code blocks and technical code examples (should remain in English)
- Technical terms that don't have standard Urdu equivalents
- File paths and directory names
- External URLs and references

## Troubleshooting

If translations are not appearing:

1. Verify that the Urdu translation exists in the correct directory (`i18n/ur/docusaurus-plugin-content-docs/default/`)
2. Check that the file has the same name as the English version
3. Ensure you've rebuilt the site after adding translation files
4. Look for any configuration errors in `docusaurus.config.ts`