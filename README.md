# Physical AI & Humanoid Robotics Textbook

This comprehensive textbook on Physical AI & Humanoid Robotics is built using [Docusaurus](https://docusaurus.io/), a modern static website generator. It covers the complete curriculum for embodied AI systems spanning 13 weeks across four core modules.

## Features

- **Complete 13-Week Curriculum**: From ROS 2 fundamentals to Vision-Language-Action models
- **Multi-Modal Learning**: Integration of perception, language understanding, and physical action
- **Simulation and Real-World Applications**: Coverage of both simulated and physical robotic platforms
- **Advanced AI Integration**: Using NVIDIA Isaac and large language models for embodied intelligence
- **Urdu Translation Support**: Content available in both English and Urdu languages using Docusaurus official i18n system

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Building with Urdu Translation

To enable Urdu translation functionality:
1. Navigate to any page in the textbook
2. Use the language toggle button in the top navigation bar to switch between English and Urdu
3. The system will load the appropriate language version of the site

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Adding Content with Translation Support

To add new content with translation support:

### For New Content
1. Create the English content in the `docs/` directory as usual
2. Create the corresponding Urdu translation in `i18n/ur/docusaurus-plugin-content-docs/default/`
3. Make sure both files have matching slugs in their frontmatter

### Translation Management
The site uses Docusaurus's official i18n system:
- English content is in the `docs/` directory
- Urdu translations are in the `i18n/ur/` directory
- The system creates separate, static versions of the site for each language

### Best Practices
- Keep translated content synchronized with the English version
- Maintain the same sidebar_position values in translated files
- Use consistent terminology across translations
- Test translation functionality after each update

### Adding New Translated Pages
1. Create your new English page in the `docs/` directory
2. Create the Urdu translation in `i18n/ur/docusaurus-plugin-content-docs/default/` with the same filename
3. Add both to the appropriate sidebar configuration

The language switcher will appear automatically in the navbar when multiple locales are configured.
