#!/usr/bin/env node

/**
 * Content generation script for AI-assisted textbook content creation
 * This script provides a framework for generating educational content using AI
 */

require('dotenv').config();

const fs = require('fs');
const path = require('path');

// Configuration for content generation
const config = {
  // Claude API endpoint (placeholder - would be replaced with actual API)
  apiEndpoint: process.env.CLAUDE_API_URL || 'https://api.anthropic.com/v1/messages',
  apiKey: process.env.CLAUDE_API_KEY,
  // Content templates for different types of content
  templates: {
    week: {
      frontmatter: {
        title: '{{title}}',
        description: '{{description}}',
        sidebar_position: '{{position}}',
      },
      structure: `# {{title}}

## Learning Objectives
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Content
{{content}}

## Summary
- Key point 1
- Key point 2
- Key point 3
`
    },
    lab: {
      frontmatter: {
        title: '{{title}}',
        description: '{{description}}',
        sidebar_position: '{{position}}',
      },
      structure: `# {{title}}

## Objective
{{objective}}

## Prerequisites
- [Prerequisite 1]
- [Prerequisite 2]

## Equipment Required
- [Equipment 1]
- [Equipment 2]

## Lab Steps
1. [Step 1]
2. [Step 2]
3. [Step 3]

## Expected Results
{{expectedResults}}

## Troubleshooting
- Issue: [Common issue]
  Solution: [Solution]
`
    },
    assessment: {
      frontmatter: {
        title: '{{title}}',
        description: '{{description}}',
        sidebar_position: '{{position}}',
      },
      structure: `# {{title}}

## Instructions
{{instructions}}

## Questions

1. **Question text**
   - Answer choices (for multiple choice)

2. **Question text**
   - Short answer or code question

## Rubric
- [Criteria for grading]
`
    }
  }
};

// Function to generate content based on module and week
function generateContent(options) {
  const { module, weeks, type, title, description } = options;
  
  console.log(`Generating ${type} content for ${module}, weeks ${weeks}...`);
  
  // Determine the target directory based on module
  const targetDir = path.join(__dirname, '..', 'docs', module);
  
  // Ensure the directory exists
  if (!fs.existsSync(targetDir)) {
    fs.mkdirSync(targetDir, { recursive: true });
  }
  
  // Generate file name based on content type
  let fileName;
  if (type === 'week') {
    fileName = `week${weeks}.md`;
  } else if (type === 'lab') {
    fileName = `lab${weeks.replace('-', '')}.md`;
  } else if (type === 'assessment') {
    fileName = `assessment${weeks.replace('-', '')}.md`;
  }
  
  const filePath = path.join(targetDir, fileName);
  
  // Get the template for this content type
  const template = config.templates[type];
  if (!template) {
    throw new Error(`Unknown content type: ${type}`);
  }
  
  // Replace placeholders in the template with actual values
  let content = template.structure
    .replace('{{title}}', title)
    .replace('{{description}}', description)
    .replace('{{content}}', 'Content to be generated here...')
    .replace('{{objective}}', 'Objective of the lab...')
    .replace('{{expectedResults}}', 'Expected results description...')
    .replace('{{instructions}}', 'Assessment instructions...');
  
  // Create frontmatter
  const frontmatter = `---
title: ${title}
description: ${description}
sidebar_position: ${weeks.replace('-', '')}
---

`;
  
  // Write the file
  fs.writeFileSync(filePath, frontmatter + content);
  
  console.log(`✅ Content generated at: ${filePath}`);
  
  return filePath;
}

// Parse command line arguments
function parseArgs() {
  const args = process.argv.slice(2);
  const options = {};
  
  for (let i = 0; i < args.length; i++) {
    if (args[i] === '--module') {
      options.module = args[i + 1];
      i++;
    } else if (args[i] === '--weeks') {
      options.weeks = args[i + 1];
      i++;
    } else if (args[i] === '--type') {
      options.type = args[i + 1];
      i++;
    } else if (args[i] === '--title') {
      options.title = args[i + 1];
      i++;
    } else if (args[i] === '--description') {
      options.description = args[i + 1];
      i++;
    }
  }
  
  return options;
}

// Main function
function main() {
  console.log('AI Content Generation Script');
  console.log('============================\n');
  
  const options = parseArgs();
  
  // Validate required options
  if (!options.module || !options.weeks || !options.type || !options.title) {
    console.log('Usage: node generate-content.js --module <module> --weeks <weeks> --type <type> --title <title> --description <description>');
    console.log('\nExample: node generate-content.js --module ros2 --weeks 1-2 --type week --title "Introduction to ROS 2" --description "Basic concepts of ROS 2"');
    process.exit(1);
  }
  
  // Set default description if not provided
  if (!options.description) {
    options.description = `Content for ${options.module} module, weeks ${options.weeks}`;
  }
  
  try {
    const filePath = generateContent(options);
    console.log(`\n🎉 Content generation completed successfully!`);
    console.log(`📝 Created file: ${filePath}`);
  } catch (error) {
    console.error(`❌ Error generating content: ${error.message}`);
    process.exit(1);
  }
}

// Export for testing
if (require.main === module) {
  main();
} else {
  module.exports = { generateContent };