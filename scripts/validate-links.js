#!/usr/bin/env node

/**
 * Content validation script for checking internal links in Docusaurus documentation
 * Validates that all internal links reference existing markdown files
 */

const fs = require('fs');
const path = require('path');
const glob = require('glob');

// Function to extract internal links from markdown content
function extractInternalLinks(content) {
  // Regular expression to match markdown links that start with /docs or ../ or ./
  const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
  const links = [];
  let match;

  while ((match = linkRegex.exec(content)) !== null) {
    const link = match[2];
    // Only consider internal links (starting with /docs, ../, ./, or #)
    if (link.startsWith('/docs') || link.startsWith('../') || link.startsWith('./') || link.startsWith('#')) {
      links.push(link);
    }
  }

  return links;
}

// Function to resolve relative paths
function resolveRelativePath(currentFile, relativeLink) {
  const currentDir = path.dirname(currentFile);
  return path.resolve(currentDir, relativeLink);
}

// Function to validate if a file exists
function validateFileExists(filePath) {
  return fs.existsSync(filePath) || fs.existsSync(filePath + '.md') || fs.existsSync(path.join(filePath, 'index.md'));
}

// Main function
function validateLinks() {
  console.log('Starting content validation...\n');

  // Get all markdown files in docs directory
  const files = glob.sync('docs/**/*.md', { cwd: process.cwd() });
  
  let totalErrors = 0;
  let totalLinks = 0;

  files.forEach(file => {
    console.log(`Checking file: ${file}`);
    
    const content = fs.readFileSync(file, 'utf8');
    const links = extractInternalLinks(content);
    
    links.forEach(link => {
      totalLinks++;
      
      // Skip anchor links for now (would require more complex validation)
      if (link.startsWith('#')) {
        return;
      }
      
      let resolvedPath;
      
      if (link.startsWith('/docs')) {
        // Convert /docs/... to docs/...
        resolvedPath = path.join(process.cwd(), link.substring(1));
      } else {
        resolvedPath = resolveRelativePath(file, link);
      }
      
      // Remove any query parameters or anchors
      resolvedPath = resolvedPath.split('#')[0].split('?')[0];
      
      // Check if the file exists
      if (!validateFileExists(resolvedPath)) {
        console.log(`  ❌ Broken link: ${link} (resolved to: ${resolvedPath})`);
        totalErrors++;
      } else {
        console.log(`  ✅ Valid link: ${link}`);
      }
    });
    
    console.log(''); // Empty line for readability
  });

  console.log(`\nValidation complete!`);
  console.log(`Total links checked: ${totalLinks}`);
  console.log(`Errors found: ${totalErrors}`);

  if (totalErrors > 0) {
    console.log(`\n❌ Validation failed with ${totalErrors} broken link(s)`);
    process.exit(1);
  } else {
    console.log(`\n✅ All links are valid!`);
    process.exit(0);
  }
}

// Run validation
validateLinks();