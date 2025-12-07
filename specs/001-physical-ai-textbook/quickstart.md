# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This guide provides a quick start for developers contributing to the Physical AI & Humanoid Robotics textbook project.

## Prerequisites

### System Requirements
- Node.js 18.x or higher
- npm or yarn package manager
- Git version control system
- A code editor (VS Code recommended)

### Optional for Advanced Features
- Python 3.8+ (for AI content generation scripts)
- Access to Claude Code API (for AI-driven content generation)

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Install Dependencies
```bash
npm install
# or
yarn install
```

### 3. Local Development
```bash
npm run start
# or
yarn start
```

This command starts a local development server and opens the textbook in your default browser. Most changes are reflected live without restarting the server.

## Project Structure
```
my-website/                  # Repository root
├── docusaurus.config.ts     # Docusaurus configuration
├── sidebars.ts             # Navigation sidebar configuration
├── package.json            # Project dependencies and scripts
├── src/
│   ├── components/         # Reusable React components
│   ├── pages/              # Static pages
│   └── theme/              # Custom theme overrides
├── docs/                   # Textbook content organized by modules
│   ├── intro.md
│   ├── ros2/               # ROS 2 module content
│   ├── gazebo-unity/       # Gazebo/Unity module content
│   ├── nvidia-isaac/       # NVIDIA Isaac module content
│   ├── vla/                # Vision-Language-Action module content
│   └── conclusion.md
├── static/                 # Static assets
└── scripts/                # Content generation and management scripts
```

## Adding Content

### Creating a New Week of Content
1. Navigate to the appropriate module directory in `/docs/`
2. Create a new markdown file with a descriptive name (e.g., `week5-6.md`)
3. Add frontmatter with the required properties:

```markdown
---
title: Week 5-6 - Advanced Perception
description: Advanced perception techniques in robotics
sidebar_position: 2
---

# Week 5-6: Advanced Perception

## Learning Objectives
- Understand perception algorithms
- Implement basic perception in simulation
```

### Creating a Lab Exercise
1. In the appropriate module directory, create a subdirectory called `lab-exercises`
2. Add your lab exercise as a markdown file:

```markdown
---
title: Lab Exercise 3 - Perception in Gazebo
description: Implement perception algorithms in Gazebo simulation
sidebar_position: 4
---

# Lab Exercise 3: Perception in Gazebo

## Objective
This lab will teach you how to implement perception algorithms in a simulated environment.

## Prerequisites
- Completion of Week 5-6 content
- Gazebo simulation environment
```

## Running Tests

### Unit Tests
```bash
npm run test
# or
yarn test
```

### End-to-End Tests
```bash
npm run test:e2e
# or
yarn test:e2e
```

### Content Validation
```bash
npm run validate:content
# or
yarn validate:content
```

## Building the Site

### For Production
```bash
npm run build
# or
yarn build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

### Local Preview
```bash
npm run serve
# or
yarn serve
```

## AI Content Generation

### Setup API Access
1. Get access to Claude Code API
2. Create a `.env` file in the project root:
```env
CLAUDE_API_KEY=your_api_key_here
```

### Generate Content
```bash
npm run generate:content -- --module ros2 --weeks 1-2
```

## Deployment

### GitHub Pages (Default)
The site is automatically deployed via GitHub Actions when changes are pushed to the `main` branch.

### Manual Deployment
```bash
npm run deploy
# or
yarn deploy
```

## API Integration

### Progress Tracking
The textbook integrates with a backend API to track student progress:

1. Set the API endpoint in `docusaurus.config.ts`:
```js
module.exports = {
  // ... other config
  themeConfig: {
    textbookApi: {
      baseUrl: process.env.TEXTBOOK_API_URL || 'https://api.textbook.example.com',
      version: 'v1'
    }
  }
};
```

2. Use the progress tracking components in your content:
```jsx
import {ProgressTracker} from '@site/src/components/ProgressTracker';

<ProgressTracker contentId="ros2-week1-lecture" />
```

## Best Practices

1. **Content Organization**: Follow the 4 modules × 13 weeks structure
2. **Accessibility**: All content should meet WCAG 2.1 AA standards
3. **Performance**: Optimize images and keep page load time under 3 seconds
4. **Testing**: Write tests for new components and features
5. **Documentation**: Document any new features or changes to the architecture