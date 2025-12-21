---
title: Translation Process Guide (Deprecated)
sidebar_position: 100
---

# Translation Process Guide (Deprecated)

This guide previously explained how to implement custom MDX components for translation functionality. However, the project has transitioned to using Docusaurus's official i18n system instead of the custom approach.

## Current Approach

The site now uses Docusaurus's built-in internationalization (i18n) system:
- English content is in the `docs/` directory
- Urdu translations are in the `i18n/ur/` directory
- The system creates separate, static versions of the site for each language

## Migration Details

- Removed custom DOMTranslate components from all documentation files
- Removed custom translation functionality from the codebase
- Configured official Docusaurus i18n in `docusaurus.config.ts`
- Created proper directory structure for locale-specific content

For adding new translated content, please refer to the Translation Process Guide in the documentation.