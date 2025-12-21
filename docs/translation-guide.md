# Translation System Guide (Deprecated)

This document previously provided a comprehensive overview of the custom translation system used in the Physical AI & Humanoid Robotics Textbook. However, the project has transitioned to using Docusaurus's official i18n system instead.

## Previous Architecture

The previous system consisted of:

1. **DOMTranslate Component**: A React component that wrapped translatable content and added necessary attributes
2. **DOMTranslateHandler**: A React component that monitored for language changes and updated the DOM accordingly
3. **LanguageContext**: A React context that managed the current language state
4. **Translation Dictionary**: A comprehensive mapping of English phrases to Urdu equivalents
5. **LanguageToggleButton**: A UI component in the navbar for toggling between languages

## Current Architecture

The site now uses Docusaurus's built-in internationalization (i18n) system:

- English content is created in the `docs/` directory
- Urdu translations are created in the `i18n/ur/` directory
- The system generates separate, static versions of the site for each language
- Language switching loads the appropriate pre-built version of the site

## Benefits of the New System

- More reliable translation functionality
- Better SEO for each language version
- Improved performance with static content
- Native Docusaurus support for i18n features
- Simpler maintenance and updates

## Migration Details

- Removed custom DOMTranslate components from all documentation files
- Removed custom translation handler and related components
- Removed custom translation context and functions
- Configured official Docusaurus i18n in `docusaurus.config.ts`
- Created proper directory structure for locale-specific content

## Extending the System

To add new translated content:

- Create the English content in the `docs/` directory as usual
- Create the corresponding Urdu translation in `i18n/ur/docusaurus-plugin-content-docs/default/`
- Make sure both files have matching slugs in their frontmatter
- Add both files to the appropriate sidebar configuration

For more details on adding translation to new content, see the Adding Translations Guide (Deprecated).