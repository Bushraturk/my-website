---
title: Translation Process Guide
sidebar_position: 102
---

# Translation Process Guide

This guide explains how to maintain and update translations in the Physical AI & Humanoid Robotics textbook using Docusaurus's built-in internationalization (i18n) system.

## Overview

The textbook uses Docusaurus's official i18n approach, which creates separate versions of content for each language. This ensures reliable translation functionality and proper SEO.

## Directory Structure

Translations are organized in the `i18n/` directory:

```
i18n/
└── ur/                    # Urdu locale
    └── docusaurus-plugin-content-docs/
        └── default/       # Default version for docs
            ├── intro.md
            ├── home-test.md
            ├── ros2/
            ├── gazebo-unity/
            ├── nvidia-isaac/
            └── vla/
```

## Adding New Translated Content

When adding new English content in the `docs/` directory, you should also create a corresponding Urdu translation:

1. Create the English content in `docs/` (e.g., `docs/new-topic.md`)
2. Create the Urdu translation in `i18n/ur/docusaurus-plugin-content-docs/default/` (e.g., `i18n/ur/docusaurus-plugin-content-docs/default/new-topic.md`)
3. Ensure both files use the same slug in their frontmatter

## Updating UI Translations

For translating UI elements (navigation, buttons, etc.), update the JSON files in:

```
i18n/ur/docusaurus-theme-classic/
├── translation.json           # General UI translations
└── navbar.json              # Navigation-specific translations
```

## Building the Site with Translations

To build the site with all translations:

```bash
npm run build
```

To start the development server:

```bash
npm run start
```

## Testing Translations

1. Verify that the language switcher appears in the navbar
2. Test navigation between English and Urdu versions
3. Ensure all content is properly translated
4. Check that the RTL (right-to-left) layout works for Urdu

## Best Practices

- Keep translated content synchronized with the English version
- Maintain the same sidebar_position values in translated files
- Use consistent terminology across translations
- Test translation functionality after each update

## Troubleshooting

### Translations not appearing
- Verify the locale is properly configured in `docusaurus.config.ts`
- Check that the directory structure matches the expected pattern
- Ensure the file names match between English and translated content

### Language switcher missing
- Confirm multiple locales are defined in `docusaurus.config.ts`
- Verify that the site has been rebuilt after configuration changes

This approach ensures reliable translation functionality across the entire textbook, avoiding the timing and DOM manipulation issues of the previous custom approach.